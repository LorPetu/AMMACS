import rclpy
from rosgraph_msgs.msg import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from shapely.geometry import Point
from shapely.ops import unary_union
from gbeam2_interfaces.msg import *
from gbeam2_interfaces.srv import *
from gbeam2_interfaces.action import *
from nav_msgs.msg import Odometry 
import math
import numpy as np
import pandas as pd
import re
import csv

# ---CONFIGURATION ---
LOG_FILE = "/home/lor/rosbags/results.csv" #os.path.expanduser("~/simulation_log.csv")  # Log file path
PARENT_FOLDER = "/home/lor/rosbags"  # Replace with the desired parent folder path

MAP_SIZES ={
    "turtlebot3_dqn_stage4EMPTY.world" : 21.13,
    "turtlebot3_dqn_stage4.world": 21.13,
}

class ExplorationAnalyzer(Node):
    def __init__(self):
        super().__init__("exploration_analyzer")

        self.declare_parameter("namespace", "robot0")
        self.declare_parameter("bag_name", "")
        self.namespace = self.get_parameter("namespace").get_parameter_value().string_value
        self.bag_name = self.get_parameter("bag_name").get_parameter_value().string_value
        self.get_logger().info(f"Running analyzer for {self.bag_name}")

        self.update_cell_in_csv(self.bag_name,"NAMESPACE",self.namespace)
        self.map_area = MAP_SIZES[self.get_cell_value_from_csv(self.bag_name,"MAP")]
        self.update_cell_in_csv(self.bag_name,"MAP_AREA",self.map_area)

        # Extract the numeric index from the namespace
        match = re.search(r'\d+', self.namespace)  # Find digits in the namespace
        self.robot_index = int(match.group()) if match else 0  # Convert to int
        self.N_robots = self.get_cell_value_from_csv(self.bag_name,"N_ROBOTS")
        self.CONFIG = self.get_cell_value_from_csv(self.bag_name,"CONFIG")

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
    
    @staticmethod
    def compute_union_area(nodes, radius):
        coordinates = [(node.x, node.y) for node in nodes]  # Extract coordinates from nodes
        circles = [Point(x, y).buffer(radius) for x, y in coordinates]  # Create circle geometries
        union_shape = unary_union(circles)  # Compute the union of all circles
        return union_shape.area  # Get the area of the union



    def odomCallback(self, odom: Odometry):
        self.pos_vert.x = odom.pose.pose.position.x
        self.pos_vert.y = odom.pose.pose.position.y

        if self.start_pos is None:
            self.start_pos = Vertex()
            self.start_pos.x = odom.pose.pose.position.x
            self.start_pos.y = odom.pose.pose.position.y
            self.get_logger().info(f"STARTING POS pos x:{self.start_pos.x:.2} y:{self.start_pos.y:.2}")

    def get_cell_value_from_csv(self, bag_name, column_name):
        """Retrieve the value of a cell in the CSV file corresponding to the bag_name and column_name."""
        # Load CSV into a DataFrame
        df = pd.read_csv(LOG_FILE)

        # Ensure column exists
        if column_name not in df.columns:
            self.get_logger().info(f"Column '{column_name}' does not exist in the CSV file.")
            return None

        # Find the row matching the bag_name
        matching_row = df[df["BAG_NAME"] == bag_name]

        if matching_row.empty:
            self.get_logger().info(f"No row found with BAG_NAME '{bag_name}'.")
            return None

        # Retrieve the value from the specified column
        value = matching_row.iloc[0][column_name]
        return value

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

            # Check if the corresponding cell is empty
            empty_namespace_rows = matching_rows[matching_rows["NAMESPACE"].isna() | (matching_rows["NAMESPACE"] == "")]
            if not empty_namespace_rows.empty:
                # Update the first empty cell with the new value
                df.loc[empty_namespace_rows.index[0], "NAMESPACE"] = new_value
            else:
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

    
    def global_callback(self, map_msg:GlobalMap):
        """Receive global map data and compare with the previous graph."""
        
        # Ensure that self.global_map is initialized before accessing it
        if not hasattr(self, 'global_map'):
            self.global_map = map_msg
            #self.get_logger().info("Received first global map.")
            return

        # Check if robot_index is valid before accessing the map
        if self.robot_index >= len(map_msg.map):
            self.get_logger().error(f"Invalid robot index {self.robot_index}, available maps: {len(map.map)}")
            return
        
        if self.robot_index >= len(self.global_map.map):
            self.get_logger().error(f"Previous map does not have index {self.robot_index}. Skipping comparison.")
            self.global_map = map_msg
            return
        
        old_other_graphs = [map for i, map in enumerate(self.global_map.map) if i != self.robot_index]
        my_old_graph = self.global_map.map[self.robot_index]
        other_graphs = [map for i, map in enumerate(map_msg.map) if i != self.robot_index]
        my_graph = map_msg.map[self.robot_index]

        # Ensure graphs are not None
        if my_old_graph is None or my_graph is None:
            self.get_logger().error("Graph data is None. Skipping comparison.")
            self.global_map = map_msg
            return

        # Compare with previous graph
        self.catchGraphChange(my_old_graph, my_graph)

        for old_graph in old_other_graphs:
            for new_graph in other_graphs:
                self.catchGraphChange(old_graph, new_graph)

        # Store the current map for future comparisons
        self.global_map = map_msg
        #self.get_logger().info("Updated global map.")


    def graph_callback(self, graph: Graph):
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

    def computeOverlappingNodes(self) -> int:
        """Compute the overlapping area between the global maps of two agents."""
        if self.global_map is None:
            self.get_logger().warning("Cannot compute overlapping nodes: missing data.")
            return 0
        
        # Check if robot_index is valid before accessing the map
        if self.robot_index >= len(self.global_map.map):
            self.get_logger().error(f"Robot index {self.robot_index} is out of range.")
            return 0
        
        # Extract the global maps of the two agents
        my_graph = self.global_map.map[self.robot_index]
        other_graphs = [map for i, map in enumerate(self.global_map.map) if i != self.robot_index]
        N_overlapping = 0

        # Ensure that both maps are not None
        if my_graph is None or other_graphs is None:
            self.get_logger().warning("Cannot compute overlapping nodes: missing data.")
            return 0

        # Compute the overlapping nodes
        for my_node in my_graph.nodes:
            for other_graph in other_graphs:
                for other_node in other_graph.nodes:
                        distance = self.dist(my_node, other_node)
                        if distance <= 0.25:
                            self.get_logger().info(f"Node {my_node.id} is overlapping with robot{other_graph.robot_id} at distance {distance:.2f}") #{other_graph.robot_id}
                            N_overlapping += 1
                            
        return N_overlapping




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

        edge_density = 2.0 * tot_edges / (tot_nodes * (tot_nodes - 1))
        self.get_logger().info(f"Edge density:{edge_density:.2}")
        cluster_avg_size = 0.0
        cluster_2_size = 0.0
        cluster_1_size = 0.0
        total_nodes_in_clusters = 0

        for cluster in self.graph_data.cluster_graph.clusters:
            size = len(cluster.nodes)
            total_nodes_in_clusters += size
            if size < 3:
                cluster_2_size += 1
            if size < 2:
                cluster_1_size += 1
            cluster_avg_size += size * size  # Weight by size

        if total_nodes_in_clusters > 0:
            cluster_avg_size = cluster_avg_size / total_nodes_in_clusters
        


        self.update_cell_in_csv(self.bag_name,"NODES",tot_nodes)
        self.update_cell_in_csv(self.bag_name,"EDGES",tot_edges)
        self.update_cell_in_csv(self.bag_name,"EDGE_DENSITY",edge_density)
        self.update_cell_in_csv(self.bag_name,"CLUSTERS",tot_clusters)
        self.update_cell_in_csv(self.bag_name,"EXPLORED_N", len(self.explored_nodes))
        count = 0
        for n in self.null_expl_nodes:
            if n.belong_to == self.robot_index:
                count +=1
        self.update_cell_in_csv(self.bag_name,"INSIDE_N", count)

        # CLUSTERS_MIN_2	CLUSTERS_MIN_1	CLUSTERS_AVG_SIZE
        self.update_cell_in_csv(self.bag_name,"CLUSTERS_MIN_2", cluster_2_size)
        self.update_cell_in_csv(self.bag_name,"CLUSTERS_MIN_1", cluster_1_size)
        self.update_cell_in_csv(self.bag_name,"CLUSTERS_AVG_SIZE", cluster_avg_size)

        self.get_logger().info(f"STARTING POS pos x:{self.start_pos.x:.2} y:{self.start_pos.y:.2}")
        self.update_cell_in_csv(self.bag_name,"POS_X",self.start_pos.x)
        self.update_cell_in_csv(self.bag_name,"POS_Y",self.start_pos.y)


        self.update_cell_in_csv(self.bag_name,"GRAPH_COVERAGE",self.compute_union_area(self.graph_data.nodes, 0.25))

        self.update_cell_in_csv(self.bag_name,"OVERLAP_N",self.computeOverlappingNodes())
        



        # Example: Compute total explored nodes
        num_explored_nodes = len(self.explored_nodes)
        self.get_logger().info(f"Total explored nodes: {num_explored_nodes}")
        self.get_logger().info(f"Total explorable nodes: {tot_nodes - len(self.null_expl_nodes)}")

def main(args=None):
    rclpy.init(args=args)
    node = ExplorationAnalyzer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
