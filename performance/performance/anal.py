import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from gbeam2_interfaces.msg import *
from gbeam2_interfaces.srv import *
from gbeam2_interfaces.action import *
import numpy as np
import re


class ExplorationAnalyzer(Node):
    def __init__(self):
        super().__init__("exploration_analyzer")

        self.declare_parameter("namespace", "robot0")
        self.namespace = self.get_parameter("namespace").get_parameter_value().string_value

        # Extract the numeric index from the namespace
        match = re.search(r'\d+', self.namespace)  # Find digits in the namespace
        self.robot_index = int(match.group()) if match else 0  # Convert to int

        # Enable simulation time
        self.use_sim_time = True  # Ensure ROS uses /clock for time tracking

        # Subscribers with namespace support
        self.create_subscription(Graph, f"{self.namespace}/gbeam/reachability_graph", self.graph_callback, 10)
        self.create_subscription(GlobalMap, f"{self.namespace}/gbeam/merged_graph", self.global_callback, 10)
        self.create_subscription(GlobalMap, f"{self.namespace}/gbeam/merged_graph", self.global_callback, 10)

        # Service for exploration timing
        self.timer_active = False
        self.start_time = None
        self.total_time = 0  # Store time in nanoseconds (int)
        self.create_service(Trigger, "/start_timer", self.start_timer_callback)
        self.create_service(Trigger, "/stop_timer", self.stop_timer_callback)

        # Storage for computation
        self.graph_data = None
        self.global_map = None
        self.explored_nodes = []

    def start_timer_callback(self, request, response):
        """Start the timer for tracking exploration duration using ROS time."""
        if not self.timer_active:
            self.start_time = self.get_clock().now()
            self.timer_active = True
            self.get_logger().info("Exploration timer started (ROS time).")
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
            self.get_logger().info(f"Exploration timer stopped. Total simulation time: {total_seconds:.2f} seconds.")
            
            # Trigger evaluation
            self.evaluate_exploration()

        response.success = True
        return response
    
    def target_callback(self, node):
        """Receive the target node"""
    
    def global_callback(self, map):
        """Receive global map data."""
        self.global_map = map
        self.get_logger().info("Received global map.")

    def graph_callback(self, graph):
        """Process graph data to analyze exploration per agent."""
        self.graph_data = graph
        # for node in graph.nodes:
        #     self.get_logger().info(f"Node: {node.id}")

        self.get_logger().info("Received graph.")

    def evaluate_exploration(self):
        """Evaluate exploration efficiency based on collected data."""
        if self.graph_data is None or self.global_map is None:
            self.get_logger().warning("Cannot evaluate exploration: missing data.")
            return
        
        # Compute Local Edge Density
        

        # 


        # Example: Compute total explored nodes
        num_explored_nodes = len(self.explored_nodes)
        self.get_logger().info(f"Total explored nodes: {num_explored_nodes}")

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
