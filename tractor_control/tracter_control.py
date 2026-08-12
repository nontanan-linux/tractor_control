#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Point, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64MultiArray
from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from autoware_control_msgs.msg import Control
from visualization_msgs.msg import Marker, MarkerArray
from rcl_interfaces.msg import SetParametersResult
import numpy as np
import math
import sys
import yaml
from ament_index_python.packages import get_package_share_directory

# Optimization config
os.environ["OMP_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["OPENBLAS_NUM_THREADS"] = "1"

# Load Config
config_path = os.path.join(get_package_share_directory('tracter_control'), 'config/mpc_config.yaml')
try:
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
except FileNotFoundError:
    print(f"Error: Config file not found at {config_path}")
    sys.exit(1)

# --- Parameters ---
DT = config['system']['dt']
L = config['vehicle']['wheelbase']
MAX_STEER = config['vehicle']['max_steer']
MAX_ACCEL = config['mpc']['constraints'].get('acceleration_limit', config['vehicle']['max_accel'])
N = config['mpc']['prediction_horizon']
DT_PRED = config['mpc']['prediction_dt']

Q_EY = config['mpc']['weights']['lat_error']
Q_ETHETA = config['mpc']['weights']['heading_error']
Q_EV = config['mpc']['weights']['velocity_error']
R_STEER = config['mpc']['weights']['steering_input']
R_ACCEL = config['mpc']['weights']['acceleration_input']
W_STEER_RATE = config['mpc']['weights']['steer_rate']
W_STEER_ACC = config['mpc']['weights']['steer_acc']
W_LAT_JERK = config['mpc']['weights']['lat_jerk']
DEBUG_LOG_INTERVAL = config['debug'].get('logging_interval', 10)

class MPCController:
    # ---------------------------------------------------------
    # Same MPC math structure from mpc_node.py
    # ---------------------------------------------------------
    def __init__(self):
        self.Q = np.diag([Q_EY, Q_ETHETA, Q_EV]) 
        self.R = np.diag([R_STEER, R_ACCEL])
        self.Q_bar = np.kron(np.eye(N), self.Q)
        self.R_bar = np.kron(np.eye(N), self.R)
        self.w_steer_rate = W_STEER_RATE
        self.w_lat_jerk = W_LAT_JERK

    def update_weights(self, q_ey, q_etheta, q_ev, w_steer_rate, w_lat_jerk):
        self.Q = np.diag([q_ey, q_etheta, q_ev])
        self.Q_bar = np.kron(np.eye(N), self.Q)
        self.w_steer_rate = w_steer_rate
        self.w_lat_jerk = w_lat_jerk
        
    def get_linear_error_model(self, v_ref, delta_ref):
        cos_sq_delta = math.cos(delta_ref)**2
        if cos_sq_delta < 0.01: cos_sq_delta = 0.01
        
        A = np.array([
            [1.0, v_ref * DT_PRED, 0.0],
            [0.0, 1.0, (math.tan(delta_ref) / L) * DT_PRED],
            [0.0, 0.0, 1.0]
        ])
        B = np.array([
            [0.0, 0.0],
            [(v_ref * DT_PRED) / (L * cos_sq_delta), 0.0],
            [0.0, DT_PRED]
        ])
        return A, B

    def solve(self, error_state, v_ref, kappa_ref):
        delta_ref = math.atan(L * kappa_ref)
        A, B = self.get_linear_error_model(v_ref, delta_ref)
        
        nx, nu = 3, 2
        S_x = np.zeros((N * nx, nx))
        S_u = np.zeros((N * nx, N * nu))
        
        S_x[0:nx, :] = A
        for i in range(1, N):
            S_x[i*nx:(i+1)*nx, :] = np.dot(A, S_x[(i-1)*nx:i*nx, :])
            
        S_u[0:nx, 0:nu] = B
        for i in range(1, N):
             S_u[i*nx:(i+1)*nx, 0:nu] = np.dot(A, S_u[(i-1)*nx:i*nx, 0:nu])
             
        for j in range(1, N):
            S_u[j*nx:, j*nu:(j+1)*nu] = S_u[0:(N-j)*nx, 0:nu]
        
        H_rate = np.zeros((N * nu, N * nu))
        for i in range(1, N):
            idx_curr, idx_prev = i * nu, (i - 1) * nu
            H_rate[idx_curr, idx_curr] += self.w_steer_rate
            H_rate[idx_prev, idx_prev] += self.w_steer_rate
            H_rate[idx_curr, idx_prev] -= self.w_steer_rate
            H_rate[idx_prev, idx_curr] -= self.w_steer_rate
            
            jerk_weight = self.w_lat_jerk * (v_ref**2 / L)**2
            H_rate[idx_curr, idx_curr] += jerk_weight
            H_rate[idx_prev, idx_prev] += jerk_weight
            H_rate[idx_curr, idx_prev] -= jerk_weight
            H_rate[idx_prev, idx_curr] -= jerk_weight

        H_acc = np.zeros((N * nu, N * nu))
        for i in range(2, N):
            idx_0, idx_1, idx_2 = i * nu, (i - 1) * nu, (i - 2) * nu
            v_vec = np.array([1, -2, 1])
            comp = np.outer(v_vec, v_vec) * W_STEER_ACC
            for ii, ri in enumerate([idx_0, idx_1, idx_2]):
                for jj, rj in enumerate([idx_0, idx_1, idx_2]):
                    H_acc[ri, rj] += comp[ii, jj]

        H = 2 * (np.dot(S_u.T, np.dot(self.Q_bar, S_u)) + self.R_bar + H_rate + H_acc)
        term1 = np.dot(S_x, error_state)
        f = 2 * np.dot(term1.T, np.dot(self.Q_bar, S_u)).flatten()
        
        try:
            H += np.eye(H.shape[0]) * 1e-6
            du_opt_full = np.linalg.solve(H, -f)
            du_opt = du_opt_full[0:nu]
        except np.linalg.LinAlgError:
            return delta_ref, 0.0, []

        delta_opt = delta_ref + du_opt[0]
        accel_opt = du_opt[1]
        X_pred = np.dot(S_x, error_state) + np.dot(S_u, du_opt_full)
        return delta_opt, accel_opt, X_pred

class TracterControlNode(Node):
    def __init__(self):
        super().__init__('tracter_control_node')
        self.controller = MPCController()
        
        # 1. Output to Autoware's External Remote command
        output_topic = '/external/selected/control_cmd'
        self.control_pub = self.create_publisher(Control, output_topic, 10)
        
        # Visualization / Debug topics
        self.predicted_path_pub = self.create_publisher(Path, '/predicted_trajectory', 10)
        self.error_pub = self.create_publisher(Float64MultiArray, '/mpc/error_status', 10)
        
        # 2. Input from Autoware's Scenario Planner Trajectory
        traj_topic = '/planning/scenario_planning/trajectory'
        self.traj_sub = self.create_subscription(Trajectory, traj_topic, self.trajectory_callback, 10)
        
        # 3. Input from Localization State (assuming Odometry for ease, commonly it's nav_msgs/Odometry or localized pose)
        # Often Autoware's localized kinematic state is published on /localization/kinematic_state
        self.odom_sub = self.create_subscription(Odometry, '/localization/kinematic_state', self.odometry_callback, 10)
        
        self.ref_traj = None 
        self.current_state = None
        self.timer = self.create_timer(DT, self.timer_callback)
        self.time_idx_debug = 0

        gwo_enabled = config['mpc'].get('enable_gwo_tuning', False) 
        self.declare_parameter('enable_gwo_tuning', gwo_enabled)
        self.declare_parameter('mpc.weights.lat_error', float(Q_EY))
        self.declare_parameter('mpc.weights.heading_error', float(Q_ETHETA))
        self.declare_parameter('mpc.weights.velocity_error', float(Q_EV))
        self.declare_parameter('mpc.weights.steer_rate', float(W_STEER_RATE))
        self.declare_parameter('mpc.weights.lat_jerk', float(W_LAT_JERK))
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        self.get_logger().info(f"Tracter Control Node (Remote External) Started. Publishing to {output_topic}")

    def parameter_callback(self, params):
        enable_tuning = self.get_parameter('enable_gwo_tuning').value
        current_weights = {
            'lat_error': self.get_parameter('mpc.weights.lat_error').value,
            'heading_error': self.get_parameter('mpc.weights.heading_error').value,
            'velocity_error': self.get_parameter('mpc.weights.velocity_error').value,
            'steer_rate': self.get_parameter('mpc.weights.steer_rate').value,
            'lat_jerk': self.get_parameter('mpc.weights.lat_jerk').value
        }
        weights_changed = False
        for p in params:
            if p.name == 'enable_gwo_tuning':
                enable_tuning = p.value
                continue
            if p.name.startswith('mpc.weights.'):
                if not enable_tuning:
                    return SetParametersResult(successful=False, reason="Tuning disabled")
                key = p.name.replace('mpc.weights.', '')
                if key in current_weights:
                    current_weights[key] = p.value
                    weights_changed = True

        if weights_changed:
            self.controller.update_weights(
                current_weights['lat_error'], current_weights['heading_error'],
                current_weights['velocity_error'], current_weights['steer_rate'],
                current_weights['lat_jerk']
            )
        return SetParametersResult(successful=True)

    def odometry_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)
        v = msg.twist.twist.linear.x
        self.current_state = np.array([x, y, theta, v])

    def trajectory_callback(self, msg):
        traj_pts = []
        for p in msg.points:
            q = p.pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            kappa = math.tan(p.front_wheel_angle_rad) / L if p.front_wheel_angle_rad else 0.0
            traj_pts.append([p.pose.position.x, p.pose.position.y, yaw, p.longitudinal_velocity_mps, kappa])
        if len(traj_pts) > 0:
            self.ref_traj = np.array(traj_pts)

    def get_closest_point(self, x, y):
        dists_sq = (self.ref_traj[:,0] - x)**2 + (self.ref_traj[:,1] - y)**2
        min_idx = np.argmin(dists_sq)
        return min_idx, self.ref_traj[min_idx]

    def calculate_errors(self, vehicle_state, ref_pt):
        x, y, theta, v = vehicle_state
        xr, yr, thetar, vr, kr = ref_pt
        dx, dy = x - xr, y - yr
        e_y = -dx * math.sin(thetar) + dy * math.cos(thetar)
        e_theta = (theta - thetar + math.pi) % (2.0 * math.pi) - math.pi
        e_v = v - vr
        return np.array([e_y, e_theta, e_v])

    def timer_callback(self):
        if self.ref_traj is None or self.current_state is None:
            return

        x, y, theta, v = self.current_state
        ref_idx, ref_pt = self.get_closest_point(x, y)
        xr, yr, thetar, vr, kr = ref_pt
        
        error_state = self.calculate_errors(self.current_state, ref_pt)
        delta_cmd, a_cmd, pred_errors = self.controller.solve(error_state, vr, kr)
        
        delta_cmd = np.clip(delta_cmd, -MAX_STEER, MAX_STEER)
        a_cmd = np.clip(a_cmd, -MAX_ACCEL, MAX_ACCEL)
        
        self.publish_control(delta_cmd, a_cmd, vr)
        
        err_msg = Float64MultiArray()
        err_msg.data = [error_state[0], error_state[1], error_state[2]]
        self.error_pub.publish(err_msg)

        if self.time_idx_debug % DEBUG_LOG_INTERVAL == 0:
            self.get_logger().info(f"e_y: {error_state[0]:.3f}, e_th: {error_state[1]:.3f}, steer: {delta_cmd:.3f}, acc: {a_cmd:.3f}")
        self.time_idx_debug += 1

    def publish_control(self, steer, accel, speed):
        msg = Control()
        msg.stamp = self.get_clock().now().to_msg()
        msg.lateral.stamp = msg.stamp
        msg.lateral.steering_tire_angle = float(steer)
        msg.longitudinal.stamp = msg.stamp
        msg.longitudinal.velocity = float(speed)
        msg.longitudinal.acceleration = float(accel)
        msg.longitudinal.is_defined_acceleration = True
        self.control_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TracterControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
