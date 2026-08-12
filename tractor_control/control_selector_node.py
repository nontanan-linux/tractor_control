#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from autoware_auto_control_msgs.msg import AckermannControlCommand
from std_srvs.srv import SetBool

class ControlSelectorNode(Node):
    def __init__(self):
        super().__init__('control_selector_node')
        
        # State: False = Autoware, True = Custom
        self.use_custom_control = False
        
        # Publishers
        self.control_pub = self.create_publisher(
            AckermannControlCommand, 
            '/control/command/control_cmd', 
            10
        )
        
        # Subscribers
        self.autoware_sub = self.create_subscription(
            AckermannControlCommand,
            '/control/command/autoware_cmd',
            self.autoware_callback,
            10
        )
        
        self.custom_sub = self.create_subscription(
            AckermannControlCommand,
            '/control/command/custom_cmd',
            self.custom_callback,
            10
        )
        
        # Service
        self.srv = self.create_service(
            SetBool, 
            '/control/select_custom_controller', 
            self.select_custom_controller_callback
        )
        
        self.get_logger().info("Control Selector Node Started. Default Mode: Autoware Control.")

    def select_custom_controller_callback(self, request, response):
        self.use_custom_control = request.data
        mode_str = "Custom MPC" if self.use_custom_control else "Autoware"
        
        response.success = True
        response.message = f"Switched control mode to: {mode_str}"
        
        self.get_logger().info(response.message)
        return response

    def autoware_callback(self, msg):
        if not self.use_custom_control:
            self.control_pub.publish(msg)

    def custom_callback(self, msg):
        if self.use_custom_control:
            self.control_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ControlSelectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
