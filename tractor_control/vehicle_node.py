#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from std_msgs.msg import Float64MultiArray, Float64
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_vehicle_msgs.msg import SteeringReport, VelocityReport
from visualization_msgs.msg import MarkerArray
from tf2_ros import TransformBroadcaster
import numpy as np
import math
import os
import yaml
import sys
from ament_index_python.packages import get_package_share_directory

# Import our custom VehicleSimulator
from .vehicle_chassis import VehicleSimulator

class VehicleNode(Node):
    def __init__(self):
        super().__init__('vehicle_plant_node')
        
        # Load Config
        package_share_directory = get_package_share_directory('tracter_control')
        config_path = os.path.join(package_share_directory, 'config/mpc_config.yaml')
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to load config: {e}")
            sys.exit(1)
            
        self.dt = config['system']['dt']
        self.control_topic = config['system']['control_cmd_topic']
        
        # Instantiate Simulator
        self.vehicle = VehicleSimulator(**config['vehicle'], dt=self.dt)
        
        # Internal State
        self.steer_input = 0.0
        self.accel_input = 0.0
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/vehicle_marker', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Telemetry Publishers
        self.state_pub = self.create_publisher(Float64MultiArray, '/vehicle/state', 10)
        self.steer_pub = self.create_publisher(Float64, '/vehicle/steering_angle', 10)
        
        # Autoware Telemetry Publishers
        self.autoware_steer_pub = self.create_publisher(SteeringReport, '/vehicle/status/steering_status', 10)
        self.autoware_vel_pub = self.create_publisher(VelocityReport, '/vehicle/status/velocity_status', 10)
        
        # Subscribers
        self.control_sub = self.create_subscription(AckermannControlCommand, self.control_topic, self.control_callback, 10)
        self.initial_pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initial_pose_callback, 10)
        
        # Timer for plant update (Simulation Loop)
        self.timer = self.create_timer(self.dt, self.timer_callback)
        
        self.get_logger().info(f"Vehicle Plant Node Started. Subscribing to: {self.control_topic}")
        
        self.last_cmd_time = self.get_clock().now()
        self.cmd_timeout = 0.5 # seconds

    def control_callback(self, msg):
        # Using Autoware AckermannControlCommand
        self.steer_input = msg.lateral.steering_tire_angle
        self.accel_input = msg.longitudinal.acceleration
        self.last_cmd_time = self.get_clock().now()

    def initial_pose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # Convert orientation to yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        theta = np.arctan2(siny_cosp, cosy_cosp)
        v = 0.0 # Reset velocity to 0
        self.vehicle.state = np.array([x, y, theta, v])
        self.get_logger().info(f"Vehicle state reset to: x={x:.2f}, y={y:.2f}, theta={theta:.2f}")

    def timer_callback(self):
        # 1. Check Timeout
        if (self.get_clock().now() - self.last_cmd_time).nanoseconds > (self.cmd_timeout * 1e9):
            self.accel_input = 0.0
            # self.steer_input = 0.0 # Optional: keep steering or center? usually keep last valid or center.
            # Keeping last valid steer is safer for smooth stop, but zero accel stops it.
            if (self.get_clock().now() - self.last_cmd_time).nanoseconds < (self.cmd_timeout * 1e9 + 0.2e9): # Log once roughly
                 self.get_logger().warn("Control Timeout! Stopping vehicle.", throttle_duration_sec=2.0)

        # 2. Update Simulation
        self.vehicle.update([self.steer_input, self.accel_input])
        
        # 2. Get Timestamp
        now = self.get_clock().now().to_msg()
        
        # 3. Publish Odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        
        x, y, theta, v = self.vehicle.state
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = np.sin(theta / 2.0)
        odom.pose.pose.orientation.w = np.cos(theta / 2.0)
        odom.twist.twist.linear.x = v
        self.odom_pub.publish(odom)
        
        # 4. Publish TF
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.z = odom.pose.pose.orientation.z
        t.transform.rotation.w = odom.pose.pose.orientation.w
        self.tf_broadcaster.sendTransform(t)
        
        # 5. Publish Wheel TFs (Internal to VehicleSimulator)
        wheel_tfs = self.vehicle.get_wheel_tfs(now, self.steer_input)
        for w_tf in wheel_tfs:
            self.tf_broadcaster.sendTransform(w_tf)
            
        # 6. Publish Visual Markers
        marker_array = self.vehicle.get_marker_array(now, self.steer_input)
        self.marker_pub.publish(marker_array)
        
        # 7. Publish Telemetry
        state_msg = Float64MultiArray()
        state_msg.data = [float(x), float(y), float(theta), float(v)]
        self.state_pub.publish(state_msg)
        
        steer_msg = Float64()
        steer_msg.data = float(self.steer_input)
        self.steer_pub.publish(steer_msg)
        
        # 8. Publish Autoware Status
        s_report = SteeringReport()
        s_report.stamp = now
        s_report.steering_tire_angle = float(self.steer_input)
        self.autoware_steer_pub.publish(s_report)
        
        v_report = VelocityReport()
        v_report.header.stamp = now
        v_report.header.frame_id = "base_link"
        v_report.longitudinal_velocity = float(v)
        v_report.lateral_velocity = 0.0
        # heading_rate = (v * tan(delta)) / L
        v_report.heading_rate = float((v * math.tan(self.steer_input)) / self.vehicle.L)
        self.autoware_vel_pub.publish(v_report)

def main(args=None):
    rclpy.init(args=args)
    node = VehicleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
