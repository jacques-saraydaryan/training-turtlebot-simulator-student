import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist

import time;
import signal
import sys


class RemapNode(Node):
    

    def __init__(self):
        super().__init__('remapper_node')
        self.is_shutdown=False
        signal.signal(signal.SIGINT, self.signal_handler)
           # init params
        self.declare_parameter('input_topic', '/cmd_vel_fake' )
        input_topic_resolution = self.get_parameter('input_topic').get_parameter_value().string_value

        self.declare_parameter('output_topic','/cmd_vel' )
        param_output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self.get_logger().info(f'-------------------------------------------')
        self.get_logger().info(f'Parameter input_topic_resolution: {input_topic_resolution}')
        self.get_logger().info(f"Parameter output_topic: {param_output_topic} ")
        self.get_logger().info(f'-------------------------------------------')      
        
        self.publisher_ = self.create_publisher(Twist,param_output_topic, 1)
        self.subscription_ = self.create_subscription(
            Twist,
            input_topic_resolution,
            self.listener_callback,
            1)
        

    def listener_callback(self,msg):
        if not self.is_shutdown:
            self.get_logger().debug(f'Incoming msg: {msg}')       
            self.publisher_.publish(msg)
        else:
            self.get_logger().debug(f'Node shutdown..')
            self.publish_stop()
            self.get_logger().debug(f'publish_stop into Callback')
                    
    def publish_stop(self):
        #Publish emty twist for stopping the robot
        twist_zero = Twist()       
        twist_zero.linear.x=0.0
        twist_zero.linear.y=0.0
        twist_zero.angular.x=0.0
        twist_zero.angular.y=0.0
        
        self.publisher_.publish(twist_zero)
        rclpy.spin_once(self)
        
    def signal_handler(self, signum, frame):
        self.get_logger().info('Received SIGINT (Ctrl-C), publishing final message...')
        self.is_shutdown = True
        time.sleep(0.5)
        self.publish_stop()
        time.sleep(3)
        rclpy.shutdown()
        sys.exit(0)
        


def main(args=None):
    rclpy.init(args=args)

    remap_node = RemapNode()
    rclpy.spin(remap_node)
    #try:
    #    rclpy.spin(remap_node)
    #    remap_node.is_shutdown =True
    #    #remap_node.publish_stop()
    #    #remap_node.get_logger().debug(f'publish_stop into Try')
    #except KeyboardInterrupt:
    #    remap_node.is_shutdown =True
    #finally:
    #    remap_node.is_shutdown =True
    #    remap_node.get_logger().debug(f'publish_stop into Finally')
    #    time.sleep(3)
    #    rclpy.try_shutdown()
    #    remap_node.destroy_node()
        


if __name__ == '__main__':
    main()