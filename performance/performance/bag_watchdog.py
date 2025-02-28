import rclpy
from rosgraph_msgs.msg import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
import sys
import os



class BagWatchdog(Node):
    def __init__(self):
        super().__init__("bag_watchdog") 

        self.safe_shutdown_counter = 0
        
        clock_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.clock_sub = self.create_subscription(
            Clock, 
            "/clock", 
            self.clock_callback,
            clock_qos
        )
        self.last_clock_time = None
        self.check_timer = self.create_timer(1.0, self.check_timeout)
        
        # Client to call your analyzer's stop service
        self.stop_client = self.create_client(Trigger, "/stop_timer")
    
    def clock_callback(self, msg):
        self.last_clock_time = self.get_clock().now()
    
    def check_timeout(self):
        if self.last_clock_time is None:
            return
            
        now = self.get_clock().now()
        if (now - self.last_clock_time).nanoseconds > 2e9:
            self.get_logger().info("Bag playback appears to have stopped")
            self.call_stop_timer()
            self.safe_shutdown_counter += 1

        if self.safe_shutdown_counter >= 3:
            self.get_logger().info("Bag playback has stopped for 3 seconds. Shutting down.")
            
            # Destroy node first
            self.destroy_node()
            
            # Ensure rclpy is initialized before shutting down
            if rclpy.ok():
                rclpy.shutdown()
            
            # Use sys.exit(0) in main()
                
    
    def call_stop_timer(self):
        if not self.stop_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Stop timer service not available")
            return
        
        request = Trigger.Request()
        future = self.stop_client.call_async(request)
        # Handle response if needed

def main(args=None):
    rclpy.init(args=args)
    node = BagWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        
        # Ensure shutdown only happens once
        if rclpy.ok():
            rclpy.shutdown()
        
        os._exit(0)  # Ensures the entire process exits properly

if __name__ == '__main__':
    main()
