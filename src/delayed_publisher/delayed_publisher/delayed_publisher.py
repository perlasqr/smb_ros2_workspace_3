import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool


class DelayedPublisher(Node):

    def __init__(self):
        super().__init__('delayed_publisher_node')

        self.sub_tare = self.create_subscription(PointStamped , 'wp_tare',lambda msg : self.get_priority_waypoint(msg,'wp_tare'), 10)
        self.sub_far  = self.create_subscription(PointStamped , 'wp_far', lambda msg :self.get_priority_waypoint(msg,'wp_far'), 10)
        self.exploration_finished = self.create_subscription(Bool , 'exploration_finish', self.exploration_finish, 10)
        # Declare and get delay parameter (in minutes)
        self.declare_parameter('delay_minutes', 0)
        delay_minutes = self.get_parameter('delay_minutes').get_parameter_value().double_value
        delay_seconds = delay_minutes * 60.0

        self.publisher = self.create_publisher(String, '/way_point', 10)

        self.get_logger().info(f'Will publish after {delay_minutes:.2f} minutes ({delay_seconds:.1f} seconds)')

        # Create one-shot timer
        self.timer = self.create_timer(delay_seconds, self.time_out)
        self.timer.cancel()  # Cancel first, then reset to act like a one-shot
        self.timer.reset()

    def time_out(self):
        self.expl_fn=True

    def get_priority_waypoint(self, msg,publisher):
        

        if self.expl_fn and publisher == 'wp_far':
            self.publisher.publish(msg)
        if not self.expl_fn:
            self.publisher.publish(msg) 
               
    def exploration_finish(self,msg):
        self.expl_fn = True

    

def main(args=None):
    rclpy.init(args=args)
    node = DelayedPublisher()
    rclpy.spin(node)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()