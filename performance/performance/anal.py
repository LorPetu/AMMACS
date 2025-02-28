import rclpy
from rosgraph_msgs.msg import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from shapely.geometry import Point
from shapely.ops import unary_union
from gbeam2_interfaces.msg import *
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
    "turtlebot3_dqn_stage4UNEVEN.world":20.48,
    "turtlebot3_dqn_stage4HOUSE.world": 30.0,
    "turtlebot3_world.world": 50.0    
}

#src/multi_turtlebot_sim/worlds/turtlebot3_dqn_stage4HOUSE.world src/multi_turtlebot_sim/worlds/turtlebot3_dqn_stage4UNEVEN.world

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

        self.get_logger().info(f"Bag analysis for robot{self.robot_index} in {self.CONFIG} for {self.N_robots} robots")

        # Subscribers with namespace support
        self.create_subscription(Graph, f"{self.namespace}/gbeam/reachability_graph", self.graph_callback, 1)
        self.create_subscription(GlobalMap, f"{self.namespace}/gbeam/merged_graph", self.global_callback, 1)
        self.create_subscription(GraphUpdate, f"{self.namespace}/external_nodes", self.external_nodes_callback, 1)
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
        self.global_map = GlobalMap(map=[Graph() for _ in range(self.N_robots)])
        self.explored_nodes  = []
        self.null_expl_nodes = []
        self.pos_vert = Vertex()
        self.start_pos = None

        # Initialize variables for tracking message intervals
        self.last_received_time = None
        self.intervals = []
        self.average_update_size   = 0.0
        self.tot_updates_received = 0

        # Initialize variables for tracking graph size
        self.graph_size_data = []
        self.graph_size_time_data = []  # 2D array to store [total_size_nodes, nanoseconds]


        # Additional variable for GBEAM 2 robots config 
        if self.CONFIG == "GBEAM":
            self.get_logger().info("GBEAM configuration detected.")
            other_namespace = "robot1" if self.robot_index == 0 else "robot0"
            self.create_subscription(Graph, f"/{other_namespace}/gbeam/reachability_graph", self.external_graph_callback, 1)
            self.get_logger().info(f"External graph subscribing...  /{other_namespace}/gbeam/reachability_graph")
            self.external_graph_data = Graph()


    @staticmethod
    def dist(v: Vertex, u: Vertex) -> float:
        """compute distance between 2 vertices"""
        return math.sqrt((v.x - u.x) ** 2 + (v.y - u.y) ** 2)
    
    def compute_union_area(self, nodes, radius):
        coordinates = [(node.x, node.y) for node in nodes]  # Extract coordinates from nodes
        circles = [Point(x, y).buffer(radius) for x, y in coordinates]  # Create circle geometries
        union_shape = unary_union(circles)  # Compute the union of all circles

        if self.CONFIG == "GBEAM" and self.N_robots == 2:
            null_coordinates = [(node.x, node.y) for node in self.null_expl_nodes]
            null_circles = [Point(x, y).buffer(radius) for x, y in null_coordinates]
            null_union_shape = unary_union(null_circles)
            union_shape = union_shape.difference(null_union_shape)  # Subtract null nodes area

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

    # def track_graph_size(self):
    #     """Track the number of nodes in the graph and the elapsed time."""
    #     if self.timer_active:
    #         elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds
    #         num_nodes = len(self.graph_data.nodes)
    #         self.graph_size_time_data.append([num_nodes, elapsed_time])
    #         self.get_logger().info(f"Tracked graph size: {num_nodes} nodes at {elapsed_time} nanoseconds")


    def start_timer_callback(self, request, response):
        """Start the timer for tracking exploration duration using ROS time."""
        if not self.timer_active:
            self.start_time = self.get_clock().now()
            self.timer_active = True
            # self.graph_size_data = []
            # self.graph_size_time_data = []
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
            # self.graph_size_data = []
            # self.graph_size_time_data = []
            #self.get_logger().info("Exploration timer started (ROS time).")

        if hasattr(self, 'graph_data') and self.N_robots is not None and (self.N_robots == 1 or (self.N_robots == 2 and self.CONFIG == "GBEAM")):
            # Compare with previous graph
            self.catchGraphChange(self.graph_data, graph)


        # Store the current graph as the new previous state
        self.graph_data = graph

        # Track graph size and time
        # self.track_graph_size()

    

    def external_graph_callback(self, ext_graph: Graph):
        """Process graph data to analyze exploration per agent and detect changes."""
        
        if not self.timer_active:
            self.start_time = self.get_clock().now()
            self.timer_active = True
            #self.get_logger().info("Exploration timer started (ROS time).")

            

        # Store the current graph as the new previous state
        self.external_graph_data = ext_graph

    def external_nodes_callback(self, msg: GraphUpdate):
        """Callback for external nodes topic to track message intervals."""
        current_time = self.get_clock().now()

        self.tot_updates_received += 1

        self.average_update_size = ((self.average_update_size * (self.tot_updates_received - 1)) + len(msg.new_nodes)) / self.tot_updates_received

        self.get_logger().info(f"Received {len(msg.new_nodes)} nodes from external nodes topic. Average: {self.average_update_size:.2f}")

        if self.last_received_time is not None:
            interval = (current_time - self.last_received_time).nanoseconds
            self.intervals.append(interval)
            self.get_logger().info(f"Interval: {interval} nanoseconds")

        self.last_received_time = current_time

    def compute_average_interval(self):
        """Compute the average interval between received messages in seconds."""
        if not self.intervals:
            return 0.0

        total_interval = sum(self.intervals)
        average_interval = total_interval / len(self.intervals)
        average_interval_seconds = average_interval / 1e9  # Convert nanoseconds to seconds
        return average_interval_seconds



    def catchGraphChange(self, prev: Graph, curr: Graph):
        """Detect changes in the 'gain' attribute for each node in the graph."""
        
        if prev is None:
            self.get_logger().info("No previous graph data available.")
            return
        
        # Create a dictionary mapping node IDs to their 'gain' values from the previous graph
        prev_dict = {node.id: node.gain for node in prev.nodes}

        for node in curr.nodes:
            prev_gain = prev_dict.get(node.id, None)  # Get previous gain if the node existed

            if prev_gain is not None and prev_gain != node.gain:
                self.get_logger().info(f"distance from node {node.id} : {self.dist(self.pos_vert, node)}")
                if self.dist(self.pos_vert, node)<= 0.30:
                    self.explored_nodes.append(node)
                    self.get_logger().info("I explored a node")
                self.get_logger().info(f"Node {node.id}: gain changed from {prev_gain:.2f} to {node.gain:.2f}")

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
    
    # def process_graph_size_data(self):
    #     """Process the graph size data to compute the number of nodes at each 10% step."""
    #     if not self.graph_size_time_data:
    #         self.get_logger().warning("No graph size data to process.")
    #         return

    #     total_time = self.graph_size_time_data[-1][1]
    #     step_time = total_time / 10
    #     graph_size_at_steps = []

    #     for step in range(1, 11):
    #         target_time = step * step_time
    #         closest_data_point = min(self.graph_size_time_data, key=lambda x: abs(x[1] - target_time))
    #         graph_size_at_steps.append(closest_data_point[0])
    #         self.get_logger().info(f"Step {step * 10}%: {closest_data_point[0]} nodes at {closest_data_point[1]} nanoseconds")




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
        

        self.update_cell_in_csv(self.bag_name, "NODES", f"{tot_nodes:.2f}")
        self.update_cell_in_csv(self.bag_name, "EDGES", f"{tot_edges:.2f}")
        self.update_cell_in_csv(self.bag_name, "EDGE_DENSITY", f"{edge_density:.2f}")
        self.update_cell_in_csv(self.bag_name, "CLUSTERS", f"{tot_clusters:.2f}")
        self.update_cell_in_csv(self.bag_name, "EXPLORED_N", f"{len(self.explored_nodes):.2f}")

        filtered_explored_nodes = [node for node in self.explored_nodes if node.belong_to == 0]
        self.update_cell_in_csv(self.bag_name, "EXPLORED_N_0", f"{len(filtered_explored_nodes):.2f}")

        filtered_explored_nodes = [node for node in self.explored_nodes if node.belong_to == 1]
        self.update_cell_in_csv(self.bag_name, "EXPLORED_N_1", f"{len(filtered_explored_nodes):.2f}")


        count = 0
        if self.CONFIG == "GBEAM":
            count = len(self.null_expl_nodes)
        else:
            for n in self.null_expl_nodes:
                if n.belong_to == self.robot_index:
                    count += 1

        self.update_cell_in_csv(self.bag_name, "INSIDE_N", f"{count:.2f}")

        self.update_cell_in_csv(self.bag_name, "CLUSTERS_MIN_2", f"{cluster_2_size:.2f}")
        self.update_cell_in_csv(self.bag_name, "CLUSTERS_MIN_1", f"{cluster_1_size:.2f}")
        self.update_cell_in_csv(self.bag_name, "CLUSTERS_AVG_SIZE", f"{cluster_avg_size:.2f}")

        self.get_logger().info(f"STARTING POS pos x:{self.start_pos.x:.2f} y:{self.start_pos.y:.2f}")
        self.update_cell_in_csv(self.bag_name, "POS_X", f"{self.start_pos.x:.2f}")
        self.update_cell_in_csv(self.bag_name, "POS_Y", f"{self.start_pos.y:.2f}")

        self.update_cell_in_csv(self.bag_name, "GRAPH_COVERAGE", f"{self.compute_union_area(nodes=self.graph_data.nodes, radius=0.25):.2f}")

        if self.CONFIG == "GBEAM" and self.N_robots == 2:
            self.global_map.map[self.robot_index] = self.graph_data
            self.global_map.map[1 if self.robot_index == 0 else 0] = self.external_graph_data

        self.update_cell_in_csv(self.bag_name, "OVERLAP_N", f"{self.computeOverlappingNodes():.2f}")

        average_interval = self.compute_average_interval()
        self.update_cell_in_csv(self.bag_name, "UPDATE_T", f"{average_interval:.2f}")

        # Process graph size data
        # self.process_graph_size_data()
        



        # Example: Compute total explored nodes
        num_explored_nodes = len(self.explored_nodes)
        self.get_logger().info(f"Total explored nodes: {num_explored_nodes}")
        self.get_logger().info(f"Total explorable nodes: {tot_nodes - len(self.null_expl_nodes)}")

        # Stop spinning the node
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ExplorationAnalyzer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
