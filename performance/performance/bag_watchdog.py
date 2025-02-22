import rclpy
from rosgraph_msgs.msg import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger



class BagWatchdog(Node):
    def __init__(self):
        super().__init__("bag_watchdog") 
        
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
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
