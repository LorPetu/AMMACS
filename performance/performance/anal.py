import rclpy
from rosgraph_msgs.msg import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from gbeam2_interfaces.msg import *
from gbeam2_interfaces.srv import *
from gbeam2_interfaces.action import *
from nav_msgs.msg import Odometry  #/msg/odometry.hpp"
import math
import numpy as np
import pandas as pd
import re
import csv

# ---CONFIGURATION ---
LOG_FILE = "/home/lor/rosbags/results.csv" #os.path.expanduser("~/simulation_log.csv")  # Log file path
PARENT_FOLDER = "/home/lor/rosbags"  # Replace with the desired parent folder path

class ExplorationAnalyzer(Node):
    def __init__(self):
        super().__init__("exploration_analyzer")

        self.declare_parameter("namespace", "robot0")
        self.declare_parameter("bag_name", "")
        self.namespace = self.get_parameter("namespace").get_parameter_value().string_value
        self.bag_name = self.get_parameter("bag_name").get_parameter_value().string_value
        self.get_logger().info(f"Running analyzer for {self.bag_name}")

        self.update_cell_in_csv(self.bag_name,"NAMESPACE",self.namespace)

        # Extract the numeric index from the namespace
        match = re.search(r'\d+', self.namespace)  # Find digits in the namespace
        self.robot_index = int(match.group()) if match else 0  # Convert to int

        # Subscribers with namespace support
        self.create_subscription(Graph, f"{self.namespace}/gbeam/reachability_graph", self.graph_callback, 1)
        self.create_subscription(GlobalMap, f"{self.namespace}/gbeam/merged_graph", self.global_callback, 1)
        self.create_subscription(GlobalMap, f"{self.namespace}/gbeam/merged_graph", self.global_callback, 1)
        self.create_subscription(Odometry, f"{self.namespace}/odom", self.odomCallback, 1)

        # Service for exploration timing
        self.timer_active = False
        self.start_time = None
        self.total_time = 0  # Store time in nanoseconds (int)
        self.timer_stopped = False  # Flag to prevent multiple stops
        self.create_service(Trigger, "/start_timer", self.start_timer_callback)
        self.create_service(Trigger, "/stop_timer", self.stop_timer_callback)
        

        # Storage for computation
        self.graph_data = Graph()
        self.global_map = GlobalMap()
        self.explored_nodes = []
        self.null_expl_nodes =[]
        self.pos_vert = Vertex()
        self.start_pos = None

    @staticmethod
    def dist(v: Vertex, u: Vertex) -> float:
        """compute distance between 2 vertices"""
        return math.sqrt((v.x - u.x) ** 2 + (v.y - u.y) ** 2)

    def odomCallback(self,odom: Odometry):
        self.pos_vert.x = odom.pose.pose.position.x
        self.pos_vert.y = odom.pose.pose.position.y

        if not self.start_pos:
            self.start_pos = self.pos_vert
            self.get_logger().info(f"STARTING POS pos x:{self.pos_vert.x:.2} y:{self.pos_vert.y:.2}")

    def update_cell_in_csv(self, bag_name, column_name, new_value):
        """Updates a cell in the CSV file corresponding to the bag_name and column_name, 
        while ensuring unique namespace values create new rows instead of overwriting."""

        # Load CSV into a DataFrame
        df = pd.read_csv(LOG_FILE)

        # Ensure column exists
        if column_name not in df.columns:
            #self.get_logger().info(f"Column '{column_name}' does not exist in the CSV file.")
            return

        # Find all rows matching the bag_name
        #matching_rows =df.query('')
        matching_rows = df[df["BAG_NAME"] == bag_name]

        if matching_rows.empty:
            #self.get_logger().info(f"No row found with BAG_NAME '{bag_name}'.")
            return

        if column_name == "NAMESPACE":
            # Check if there is already a row with the same namespace
            if (matching_rows["NAMESPACE"] == new_value).any():
                #self.get_logger().info(f"Namespace '{new_value}' already exists for BAG_NAME '{bag_name}'. No changes made.")
                return

            # Duplicate the first matching row and update its namespace
            new_row = matching_rows.iloc[0].copy()
            new_row["NAMESPACE"] = new_value
            df = pd.concat([df, new_row.to_frame().T], ignore_index=True)

        else:
            # Update all occurrences of the bag_name in the given column
            df.loc[(df["BAG_NAME"] == bag_name )& (df["NAMESPACE"] == self.namespace), column_name] = new_value

        # Save the updated DataFrame back to CSV
        df.to_csv(LOG_FILE, index=False)
        #self.get_logger().info(f"Cell updated successfully for BAG_NAME '{bag_name}', Column '{column_name}'.")
    
    def stop_timer(self):
        """Stop the exploration timer and log total time."""
        if self.timer_active:
            elapsed = self.get_clock().now() - self.start_time
            self.total_time += elapsed.nanoseconds
            self.timer_active = False
            total_seconds = self.total_time / 1e9

            #self.get_logger().info(f"Exploration timer stopped. Total time: {total_seconds:.2f} seconds.")

            self.update_cell_in_csv(self.bag_name,"EXPL_TIME",total_seconds)
            
            # Trigger evaluation
            self.evaluate_exploration()
    


    def start_timer_callback(self, request, response):
        """Start the timer for tracking exploration duration using ROS time."""
        if not self.timer_active:
            self.start_time = self.get_clock().now()
            self.timer_active = True
            #self.get_logger().info("Exploration timer started (ROS time).")
        response.success = True
        return response

    def stop_timer_callback(self, request, response):
        """Stop the timer and compute total exploration time in ROS time."""
        if self.timer_active:
            end_time = self.get_clock().now()
            elapsed = end_time - self.start_time  # Get duration as rclpy Duration
            self.total_time += elapsed.nanoseconds
            self.timer_active = False
            total_seconds = self.total_time / 1e9  # Convert nanoseconds to seconds
            #self.get_logger().info(f"Exploration timer stopped. Total simulation time: {total_seconds:.2f} seconds.")

            self.update_cell_in_csv(self.bag_name,"EXPL_TIME",total_seconds)
            
            # Trigger evaluation
            self.evaluate_exploration()

        response.success = True
        return response

    
    def global_callback(self, map):
        """Receive global map data and compare with the previous graph."""
        
        # Ensure that self.global_map is initialized before accessing it
        if not hasattr(self, 'global_map'):
            self.global_map = map
            #self.get_logger().info("Received first global map.")
            return

        # Check if robot_index is valid before accessing the map
        if self.robot_index >= len(map.map):
            self.get_logger().error(f"Invalid robot index {self.robot_index}, available maps: {len(map.map)}")
            return
        
        if self.robot_index >= len(self.global_map.map):
            self.get_logger().error(f"Previous map does not have index {self.robot_index}. Skipping comparison.")
            self.global_map = map
            return
        
        my_old_graph = self.global_map.map[self.robot_index]
        my_graph = map.map[self.robot_index]

        # Ensure graphs are not None
        if my_old_graph is None or my_graph is None:
            self.get_logger().error("Graph data is None. Skipping comparison.")
            self.global_map = map
            return

        # Compare with previous graph
        self.catchGraphChange(my_old_graph, my_graph)

        # Store the current map for future comparisons
        self.global_map = map
        #self.get_logger().info("Updated global map.")


    def graph_callback(self, graph):
        """Process graph data to analyze exploration per agent and detect changes."""
        
        if not self.timer_active:
            self.start_time = self.get_clock().now()
            self.timer_active = True
            #self.get_logger().info("Exploration timer started (ROS time).")

        # # Compare with previous graph
        # if hasattr(self, 'graph_data'):
        #     self.catchGraphChange(self.graph_data, graph)

        # Store the current graph as the new previous state
        self.graph_data = graph


    def catchGraphChange(self, prev: Graph, curr: Graph):
        """Detect changes in the 'gain' attribute for each node in the graph."""
        
        if prev is None:
            #self.get_logger().info("No previous graph data available.")
            return
        
        # Create a dictionary mapping node IDs to their 'gain' values from the previous graph
        prev_dict = {node.id: node.gain for node in prev.nodes}

        for node in curr.nodes:
            prev_gain = prev_dict.get(node.id, None)  # Get previous gain if the node existed

            if prev_gain is not None and prev_gain != node.gain:
                #self.get_logger().info(f"distance from node {node.id} : {self.dist(self.pos_vert, node)}")
                if self.dist(self.pos_vert, node)<= 0.30:
                    self.explored_nodes.append(node)
                    #self.get_logger().info("I explored a node")
                #self.get_logger().info(f"Node {node.id}: gain changed from {prev_gain:.2f} to {node.gain:.2f}")

            # Check if the node is newly added and has an initial gain of 0.0
            elif prev_gain is None and node.gain == 0.0:
                #self.get_logger().info(f"Node {node.id}: newly added with initial gain 0.0")
                self.null_expl_nodes.append(node)




    def evaluate_exploration(self):
        """Evaluate exploration efficiency based on collected data."""
        if self.graph_data is None or self.global_map is None:
            self.get_logger().warning("Cannot evaluate exploration: missing data.")
            return
        
        # Compute Local Edge Density
        tot_nodes=len(self.graph_data.nodes)
        tot_edges=len(self.graph_data.edges)
        tot_clusters =len(self.graph_data.cluster_graph.clusters)

        #for node in explored_area:

        edge_density = 2.0*tot_edges/(tot_nodes*(tot_nodes-1))
        cluster_avg_size = 0.0
        cluster_2_size = 0.0
        cluster_1_size = 0.0
        for cluster in self.graph_data.cluster_graph.clusters:
            size = len(cluster.nodes)
            if  size < 3: 
                cluster_2_size
            if size < 2: 
                cluster_1_size
            cluster_avg_size += size
        
        cluster_avg_size = cluster_avg_size/tot_clusters

        self.update_cell_in_csv(self.bag_name,"NODES",tot_nodes)
        self.update_cell_in_csv(self.bag_name,"EDGES",tot_edges)
        self.update_cell_in_csv(self.bag_name,"EDGE_DENSITY",edge_density)
        self.update_cell_in_csv(self.bag_name,"CLUSTERS",tot_clusters)
        self.update_cell_in_csv(self.bag_name,"EXPLORED_N", len(self.explored_nodes))
        self.update_cell_in_csv(self.bag_name,"INSIDE_N", len(self.null_expl_nodes))

        # CLUSTERS_MIN_2	CLUSTERS_MIN_1	CLUSTERS_AVG_SIZE
        self.update_cell_in_csv(self.bag_name,"CLUSTERS_MIN_2", cluster_2_size)
        self.update_cell_in_csv(self.bag_name,"CLUSTERS_MIN_1", cluster_1_size)
        self.update_cell_in_csv(self.bag_name,"CLUSTERS_AVG_SIZE", cluster_avg_size)

        self.update_cell_in_csv(self.bag_name,"POS_X",self.start_pos.x)
        self.update_cell_in_csv(self.bag_name,"POS_Y",self.start_pos.x)
        



        # Example: Compute total explored nodes
        num_explored_nodes = len(self.explored_nodes)
        self.get_logger().info(f"Total explored nodes: {num_explored_nodes}")
        self.get_logger().info(f"Total explorable nodes: {tot_nodes - len(self.null_expl_nodes)}")

        # Example: Compute coverage percentage (Placeholder logic)
        explored_area = num_explored_nodes * 1.0  # Assume each node covers a unit area
        total_area = 100.0  # Placeholder for total map area
        coverage_index = explored_area / total_area

        self.get_logger().info(f"Coverage Index: {coverage_index:.2%}")

def main(args=None):
    rclpy.init(args=args)
    node = ExplorationAnalyzer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
