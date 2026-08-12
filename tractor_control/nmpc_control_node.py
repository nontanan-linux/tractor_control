#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from autoware_planning_msgs.msg import Trajectory
from autoware_control_msgs.msg import Control
# Autoware External API Messages
try:
    from tier4_external_api_msgs.msg import Heartbeat
    from tier4_external_api_msgs.srv import Engage
    from autoware_vehicle_msgs.msg import GearCommand
except ImportError:
    print("Warning: tier4_external_api_msgs not found. Autoware integration will be disabled.")

import numpy as np
import math
import casadi as ca
import yaml
from ament_index_python.packages import get_package_share_directory

class NMPCController:
    def __init__(self, config):
        self.config = config
        self.N = config['mpc']['prediction_horizon']
        self.dt = config['mpc']['prediction_dt']
        self.L0 = config['vehicle']['wheelbase']
        self.trailers = config.get('trailers', [{'L_bar': 1.0, 'L_trl': 1.2, 'dh_prev': 0.5}])
        self.num_trailers = len(self.trailers)
        self.nx = 3 + 2 * self.num_trailers + 1
        self.nu = 2 
        self.setup_optimizer()

    def setup_optimizer(self):
        self.opti = ca.Opti()
        self.X = self.opti.variable(self.nx, self.N + 1)
        self.U = self.opti.variable(self.nu, self.N)
        self.x_init = self.opti.parameter(self.nx)
        self.ref_traj = self.opti.parameter(self.nx, self.N + 1)
        self.u_prev = self.opti.parameter(self.nu)
        
        q_ey = self.config['mpc']['weights']['lat_error']
        q_etheta = self.config['mpc']['weights']['heading_error']
        q_ev = self.config['mpc']['weights']['velocity_error']
        r_steer = self.config['mpc']['weights']['steering_input']
        r_accel = self.config['mpc']['weights']['acceleration_input']
        rd_steer = self.config['mpc']['weights']['steer_rate']
        rd_accel = self.config['mpc']['weights'].get('steer_acc', 10.0)

        obj = 0
        for k in range(self.N):
            obj += q_ey * (self.X[0, k] - self.ref_traj[0, k])**2
            obj += q_ey * (self.X[1, k] - self.ref_traj[1, k])**2
            obj += q_etheta * (self.X[2, k] - self.ref_traj[2, k])**2
            for i in range(2 * self.num_trailers):
                obj += q_etheta * (self.X[3 + i, k] - self.ref_traj[3 + i, k])**2
            obj += q_ev * (self.X[-1, k] - self.ref_traj[-1, k])**2
            obj += r_steer * self.U[0, k]**2
            obj += r_accel * self.U[1, k]**2
            if k == 0:
                obj += rd_steer * (self.U[0, k] - self.u_prev[0])**2
                obj += rd_accel * (self.U[1, k] - self.u_prev[1])**2
            else:
                obj += rd_steer * (self.U[0, k] - self.U[0, k-1])**2
                obj += rd_accel * (self.U[1, k] - self.U[1, k-1])**2
            self.opti.subject_to(self.X[:, k+1] == self.kinematic_step(self.X[:, k], self.U[:, k]))

        self.opti.subject_to(self.X[:, 0] == self.x_init)
        self.opti.subject_to(self.opti.bounded(-self.config['vehicle']['max_steer'], self.U[0, :], self.config['vehicle']['max_steer']))
        self.opti.subject_to(self.opti.bounded(-self.config['vehicle']['max_accel'], self.U[1, :], self.config['vehicle']['max_accel']))
        self.opti.minimize(obj)
        opts = {'ipopt.print_level': 0, 'print_time': 0, 'ipopt.max_iter': 100}
        self.opti.solver('ipopt', opts)

    def kinematic_step(self, x, u):
        v = x[-1]
        theta0 = x[2]
        delta = u[0]
        a = u[1]
        dx0 = v * ca.cos(theta0)
        dy0 = v * ca.sin(theta0)
        dtheta0 = (v / self.L0) * ca.tan(delta)
        v_prev_vec = ca.vertcat(v, dtheta0)
        theta_prev = theta0
        d_trailer_thetas = []
        for i, config in enumerate(self.trailers):
            theta_db = x[3 + 2*i]
            theta_tr = x[3 + 2*i + 1]
            delta_theta_a = theta_prev - theta_db
            v_db_vec = ca.vertcat(
                ca.cos(delta_theta_a) * v_prev_vec[0] + config['dh_prev'] * ca.sin(delta_theta_a) * v_prev_vec[1],
                (1.0/config['L_bar']) * ca.sin(delta_theta_a) * v_prev_vec[0] - (config['dh_prev']/config['L_bar']) * ca.cos(delta_theta_a) * v_prev_vec[1]
            )
            d_trailer_thetas.append(v_db_vec[1])
            delta_theta_b = theta_db - theta_tr
            v_tr_vec = ca.vertcat(ca.cos(delta_theta_b) * v_db_vec[0], (1.0/config['L_trl']) * ca.sin(delta_theta_b) * v_db_vec[0])
            d_trailer_thetas.append(v_tr_vec[1])
            v_prev_vec = v_tr_vec
            theta_prev = theta_tr
        dv = a
        f = ca.vertcat(dx0, dy0, dtheta0, ca.vertcat(*d_trailer_thetas), dv)
        return x + f * self.dt

    def solve(self, x0, ref_traj, u_prev):
        self.opti.set_value(self.x_init, x0)
        self.opti.set_value(self.ref_traj, ref_traj)
        self.opti.set_value(self.u_prev, u_prev)
        try:
            sol = self.opti.solve()
            return sol.value(self.U[:, 0]), sol.value(self.X)
        except:
            return np.array([0.0, -0.5]), None

class NMPCControlNode(Node):
    def __init__(self):
        super().__init__('nmpc_control_node')
        share_dir = get_package_share_directory('tracter_control')
        config_path = os.path.join(share_dir, 'config/mpc_config.yaml')
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
            
        self.controller = NMPCController(self.config)
        self.max_accel = self.config['vehicle']['max_accel']
        
        # Standard Publishers
        self.control_pub = self.create_publisher(Control, '/mpc/control_cmd', 10)
        self.pred_path_pub = self.create_publisher(Path, '/mpc/predicted_trajectory', 10)
        
        # Autoware External API Publishers
        self.autoware_control_pub = self.create_publisher(Control, '/external/selected/control_cmd', 10)
        self.heartbeat_pub = self.create_publisher(Heartbeat, '/api/external/set/command/remote/heartbeat', 10)
        self.gear_pub = self.create_publisher(GearCommand, '/external/selected/gear_cmd', 10)
        
        self.engage_client = self.create_client(Engage, '/api/autoware/set/engage')
        self.goal_reached_triggered = False
        
        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/localization/kinematic_state', self.odom_callback, 10)
        self.traj_sub = self.create_subscription(Trajectory, '/planning/scenario_planning/trajectory', self.traj_callback, 10)
        
        self.current_state = None
        self.full_ref_points = None
        self.u_prev = np.array([0.0, 0.0])
        
        # Timers
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.heartbeat_timer = self.create_timer(0.5, self.publish_heartbeat)
        
        # Register shutdown hook
        rclpy.get_default_context().on_shutdown(self.on_shutdown)
        
        self.get_logger().info("NMPC Control Node with Autoware External API Support Started")

    def on_shutdown(self):
        self.get_logger().info("Shutting down... Sending STOP command.")
        stop_cmd = Control()
        stop_cmd.stamp = self.get_clock().now().to_msg()
        stop_cmd.lateral.steering_tire_angle = 0.0
        stop_cmd.longitudinal.velocity = 0.0
        stop_cmd.longitudinal.acceleration = -3.0 # Full brake
        self.autoware_control_pub.publish(stop_cmd)
        
        # Send one last heartbeat to ensure the selector sees the stop
        hb = Heartbeat()
        hb.stamp = stop_cmd.stamp
        self.heartbeat_pub.publish(hb)

    def publish_heartbeat(self):
        hb = Heartbeat()
        hb.stamp = self.get_clock().now().to_msg()
        self.heartbeat_pub.publish(hb)
        
        # Also periodically ensure we are in DRIVE gear
        gs = GearCommand()
        gs.stamp = hb.stamp
        gs.command = GearCommand.DRIVE
        self.gear_pub.publish(gs)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
        v = msg.twist.twist.linear.x
        trailers = [yaw] * (2 * self.controller.num_trailers)
        self.current_state = np.array([x, y, yaw] + trailers + [v])

    def traj_callback(self, msg):
        self.full_ref_points = msg.points
        # Reset the goal reached trigger when a new long trajectory is received
        if self.goal_reached_triggered and len(msg.points) > 20:
            self.goal_reached_triggered = False

    def find_nearest_index(self, state, ref_points):
        dists = [(p.pose.position.x - state[0])**2 + (p.pose.position.y - state[1])**2 for p in ref_points]
        return np.argmin(dists)

    def timer_callback(self):
        if self.current_state is None or self.full_ref_points is None:
            return
        idx = self.find_nearest_index(self.current_state, self.full_ref_points)
        N = self.controller.N
        nx = self.controller.nx
        ref_horizon = np.zeros((nx, N + 1))
        
        # Calculate current errors for debugging
        p_nearest = self.full_ref_points[idx]
        q_n = p_nearest.pose.orientation
        yaw_n = math.atan2(2*(q_n.w*q_n.z + q_n.x*q_n.y), 1 - 2*(q_n.y**2 + q_n.z**2))
        
        dx = self.current_state[0] - p_nearest.pose.position.x
        dy = self.current_state[1] - p_nearest.pose.position.y
        # Lateral error (cross track error)
        e_y = -dx * math.sin(yaw_n) + dy * math.cos(yaw_n)
        # Heading error
        e_theta = self.current_state[2] - yaw_n
        while e_theta > math.pi: e_theta -= 2*math.pi
        while e_theta < -math.pi: e_theta += 2*math.pi
        
        # Goal reaching check
        if not self.goal_reached_triggered and len(self.full_ref_points) > 0:
            goal_p = self.full_ref_points[-1].pose.position
            dist_to_goal = math.hypot(self.current_state[0] - goal_p.x, self.current_state[1] - goal_p.y)
            if dist_to_goal < 1.0 and idx >= len(self.full_ref_points) - 15:
                self.get_logger().info("🎯 GOAL REACHED! Auto-disengaging vehicle...")
                self.goal_reached_triggered = True
                if self.engage_client.wait_for_service(timeout_sec=0.5):
                    req = Engage.Request()
                    req.engage = False
                    self.engage_client.call_async(req)

        for i in range(N + 1):
            p_idx = min(idx + i, len(self.full_ref_points) - 1)
            p = self.full_ref_points[p_idx]
            ref_horizon[0, i] = p.pose.position.x
            ref_horizon[1, i] = p.pose.position.y
            q = p.pose.orientation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
            ref_horizon[2, i] = yaw
            for j in range(2 * self.controller.num_trailers):
                ref_horizon[3 + j, i] = yaw
            ref_horizon[-1, i] = p.longitudinal_velocity_mps
            
        u_opt, x_pred = self.controller.solve(self.current_state, ref_horizon, self.u_prev)
        self.u_prev = u_opt
        
        # Logging every 10 steps (~1 second)
        if not hasattr(self, '_log_counter'): self._log_counter = 0
        self._log_counter += 1
        if self._log_counter % 10 == 0:
            self.get_logger().info(
                f"NMPC Status: e_y={e_y:.3f}m, e_theta={math.degrees(e_theta):.2f}deg, "
                f"Steer={math.degrees(u_opt[0]):.2f}deg, Accel={u_opt[1]:.2f}m/s2"
            )

        # 1. Publish to Standard Topic
        cmd = Control()
        cmd.stamp = self.get_clock().now().to_msg()
        cmd.lateral.stamp = cmd.stamp
        cmd.lateral.steering_tire_angle = float(u_opt[0])
        cmd.longitudinal.stamp = cmd.stamp
        cmd.longitudinal.velocity = float(ref_horizon[-1, 0])
        cmd.longitudinal.acceleration = float(u_opt[1])
        cmd.longitudinal.is_defined_acceleration = True
        self.control_pub.publish(cmd)
        
        # 2. Publish to Autoware External API
        aw_cmd = Control()
        aw_cmd.stamp = cmd.stamp
        aw_cmd.lateral.stamp = cmd.stamp
        aw_cmd.lateral.steering_tire_angle = float(u_opt[0])
        aw_cmd.longitudinal.stamp = cmd.stamp
        aw_cmd.longitudinal.velocity = float(ref_horizon[-1, 0])
        aw_cmd.longitudinal.acceleration = float(u_opt[1])
        aw_cmd.longitudinal.is_defined_acceleration = True
        self.autoware_control_pub.publish(aw_cmd)
        
        if x_pred is not None:
            self.publish_pred_path(x_pred)

    def publish_pred_path(self, x_pred):
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        for i in range(x_pred.shape[1]):
            p = PoseStamped()
            p.pose.position.x = x_pred[0, i]
            p.pose.position.y = x_pred[1, i]
            path.poses.append(p)
        self.pred_path_pub.publish(path)

def main(args=None):
    rclpy.init(args=args)
    node = NMPCControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received.")
    finally:
        # Robust stop on exit
        node.on_shutdown()
        # Give a small amount of time for the messages to be sent
        import time
        time.sleep(0.2)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
