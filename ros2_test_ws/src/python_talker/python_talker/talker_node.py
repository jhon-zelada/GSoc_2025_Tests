import rclpy
from rclpy.node import Node
from std_msgs.msg import String 
import signal
class TalkerNode(Node):
    def __init__(self,node_name= 'talker_python_node', timer_period=0.5):
        """
        Initializes the TalkerNode with a given name and time period.
        :param node_name: Name of the node.
        :param time_period: Time period for the timer callback.
        """
        self.node_name = node_name
        self.timer_period = timer_period
        self.topic_name = 'chatter'
        super().__init__(self.node_name)
        self.declare_parameter('enable_logging', True)
        self.publisher_ = self.create_publisher(String, self.topic_name, 10)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.get_logger().info(f'{self.node_name} has been started')
        self.get_logger().info(f'Publishing on topic: {self.topic_name}')
    
    def timer_callback(self):
        """
        Callback function that gets called at regular intervals.
        Publishes Hello! ROS2 is fun! message to the chatter topic.
        :return: None
        """
        msg = String()
        msg.data = f'Hello! ROS2 is fun.'
        self.publisher_.publish(msg)
        enable_logging = self.get_parameter('enable_logging').get_parameter_value().bool_value
        if enable_logging:
            self.get_logger().info(f'Publishing: {msg.data}')

    

def signal_handler(signal, frame):
    
    """
    Signal handler for graceful shutdown.
    :param signal: The signal number.
    :param frame: The current stack frame.
    """
    print('Signal received, shutting down ...')
    rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    talker_node = TalkerNode()
    signal.signal(signal.SIGINT, signal_handler)
    rclpy.spin(talker_node)


if __name__ == '__main__':
    main()

     