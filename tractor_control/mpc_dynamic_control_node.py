#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from autoware_planning_msgs.msg import Trajectory
from autoware_control_msgs.msg import Control
from std_srvs.srv import SetBool
try:
    from tier4_external_api_msgs.msg import Heartbeat, GearShiftStamped, GearShift
    from tier4_external_api_msgs.srv import Engage
    from autoware_vehicle_msgs.msg import GearCommand
    from autoware_adapi_v1_msgs.srv import ChangeOperationMode
    from autoware_auto_vehicle_msgs.msg import SteeringReport
except ImportError:
    pass

import numpy as np
import math
import casadi as ca
import yaml
from ament_index_python.packages import get_package_share_directory

# Import the new debug logger
from tractor_control.mpc_logger import MPCDebugLogger

class MPCDynamicController:
    def __init__(self, config):
        self.config = config
        self.N = config['mpc']['prediction_horizon']
        self.dt = config['mpc']['prediction_dt']
        
        # Load Vehicle Parameters from tractor_description
        tractor_desc_dir = get_package_share_directory('tractor_description')
        veh_info_path = os.path.join(tractor_desc_dir, 'config', 'vehicle_info.param.yaml')
        with open(veh_info_path, 'r') as f:
            veh_info = yaml.safe_load(f)['/**']['ros__parameters']
            
        self.L_hitch = veh_info.get('hitch_length', 0.70)
        self.L_bar = veh_info.get('drawbar_joint_length', 1.53)
        
        # Trailer axle is assumed to be centrally located along the joint length
        trailer_joint = veh_info.get('trailer_joint_length', 1.98)
        self.L_trail = trailer_joint / 2.0
        self.l_rt = trailer_joint / 2.0
        
        # Tractor wheelbase split (assuming CG in middle)
        wb = veh_info.get('wheel_base', 1.4)
        self.l_f = wb / 2.0
        self.l_r = wb / 2.0

        # Masses and Inertias (not in yaml, using previous 1 drawbar + 1 trailer spec)
        self.m_v = 3000.0  
        self.I_v = 2600.0  
        self.m_d = 500.0   
        self.I_d = 400.0   
        self.m_t = 2000.0  
        self.I_t = 10400.0
        self.C_f = 100000.0
        self.C_r = 150000.0
        self.C_df = 100000.0
        self.C_tr = 150000.0
        self.max_F_tractor = 0.8 * 3000 * 9.81
        self.max_F_trailer = 0.8 * 6000 * 9.81
        self.max_steer = config['vehicle']['max_steer']
        self.max_accel = config['vehicle']['max_accel']

        self.nx = 10 # [x, y, theta_v, theta_d, theta_t, v_x, v_y, r, r_d, r_t]
        self.nu = 2  # [steer, accel]
        self.setup_optimizer()

    def calc_slip(self, w, u, steer=0.0):
        # Increased u_safe to 1.0 to completely eliminate low-speed gradient explosions
        u_safe = ca.sqrt(u**2 + 1.0)
        return steer - ca.atan(w / u_safe)

    def get_M_rhs(self, x, u):
        theta_v = x[2]
        theta_d = x[3]
        theta_t = x[4]
        v_x = x[5]
        v_y = x[6]
        r = x[7]
        r_d = x[8]
        r_t = x[9]
        steer_angle = u[0]
        accel = u[1]
        
        # Thrust force
        F_xr = (self.m_v + self.m_d + self.m_t) * accel
        F_xf = 0.0
        
        # Velocities in body frame
        u_v = v_x * ca.cos(theta_v) + v_y * ca.sin(theta_v)
        w_v = -v_x * ca.sin(theta_v) + v_y * ca.cos(theta_v)
        
        # Tire slip for tractor
        alpha_f = self.calc_slip(w_v + self.l_f * r, u_v, steer_angle)
        alpha_r = self.calc_slip(w_v - self.l_r * r, u_v)
        
        # Dolly velocity mapped to Dolly frame
        v_xd = u_v
        v_yd = w_v - self.L_hitch * r
        u_d = v_xd * ca.cos(theta_v - theta_d) + v_yd * ca.sin(theta_v - theta_d)
        w_d = -v_xd * ca.sin(theta_v - theta_d) + v_yd * ca.cos(theta_v - theta_d)
        alpha_d = self.calc_slip(w_d - self.L_bar * r_d, u_d)
        
        # Trailer velocity mapped to Trailer frame
        v_xt = u_d
        v_yt = w_d - self.L_bar * r_d
        u_t = v_xt * ca.cos(theta_d - theta_t) + v_yt * ca.sin(theta_d - theta_t)
        w_t = -v_xt * ca.sin(theta_d - theta_t) + v_yt * ca.cos(theta_d - theta_t)
        alpha_tr = self.calc_slip(w_t - (self.L_trail + self.l_rt) * r_t, u_t)
        
        # Smooth saturation
        # Linear tire models guarantee non-zero Jacobians for IPOPT!
        # At low speeds (e.g. < 5 m/s), tires operate purely in the linear region anyway.
        F_yf = self.C_f * alpha_f
        F_yr = self.C_r * alpha_r
        F_yd = self.C_df * alpha_d
        F_yt = self.C_tr * alpha_tr
        
        FX_f = F_xf * ca.cos(theta_v + steer_angle) - F_yf * ca.sin(theta_v + steer_angle)
        FY_f = F_xf * ca.sin(theta_v + steer_angle) + F_yf * ca.cos(theta_v + steer_angle)
        
        FX_r = F_xr * ca.cos(theta_v) - F_yr * ca.sin(theta_v)
        FY_r = F_xr * ca.sin(theta_v) + F_yr * ca.cos(theta_v)
        
        FX_d = -F_yd * ca.sin(theta_d)
        FY_d = F_yd * ca.cos(theta_d)
        
        FX_t = -F_yt * ca.sin(theta_t)
        FY_t = F_yt * ca.cos(theta_t)
        
        F_yf_v = FY_f * ca.cos(theta_v) - FX_f * ca.sin(theta_v)
        F_yr_v = FY_r * ca.cos(theta_v) - FX_r * ca.sin(theta_v)
        F_yd_v = FY_d * ca.cos(theta_v) - FX_d * ca.sin(theta_v)
        F_yt_v = FY_t * ca.cos(theta_v) - FX_t * ca.sin(theta_v)
        
        F_yd_d = FY_d * ca.cos(theta_d) - FX_d * ca.sin(theta_d)
        F_yt_d = FY_t * ca.cos(theta_d) - FX_t * ca.sin(theta_d)
        
        F_yt_t = FY_t * ca.cos(theta_t) - FX_t * ca.sin(theta_t)
        
        m_tot = self.m_v + self.m_d + self.m_t
        mdt = self.m_d + self.m_t
        
        M = ca.MX.zeros(5, 5)
        M[0, 0] = m_tot
        M[0, 2] = mdt * self.L_hitch * ca.sin(theta_v)
        M[0, 3] = mdt * self.L_bar * ca.sin(theta_d)
        M[0, 4] = self.m_t * self.L_trail * ca.sin(theta_t)
        M[1, 1] = m_tot
        M[1, 2] = -mdt * self.L_hitch * ca.cos(theta_v)
        M[1, 3] = -mdt * self.L_bar * ca.cos(theta_d)
        M[1, 4] = -self.m_t * self.L_trail * ca.cos(theta_t)
        M[2, 0] = mdt * self.L_hitch * ca.sin(theta_v)
        M[2, 1] = -mdt * self.L_hitch * ca.cos(theta_v)
        M[2, 2] = self.I_v + mdt * self.L_hitch**2
        M[2, 3] = mdt * self.L_hitch * self.L_bar * ca.cos(theta_v - theta_d)
        M[2, 4] = self.m_t * self.L_hitch * self.L_trail * ca.cos(theta_v - theta_t)
        M[3, 0] = mdt * self.L_bar * ca.sin(theta_d)
        M[3, 1] = -mdt * self.L_bar * ca.cos(theta_d)
        M[3, 2] = mdt * self.L_hitch * self.L_bar * ca.cos(theta_v - theta_d)
        M[3, 3] = self.I_d + mdt * self.L_bar**2
        M[3, 4] = self.m_t * self.L_bar * self.L_trail * ca.cos(theta_d - theta_t)
        M[4, 0] = self.m_t * self.L_trail * ca.sin(theta_t)
        M[4, 1] = -self.m_t * self.L_trail * ca.cos(theta_t)
        M[4, 2] = self.m_t * self.L_hitch * self.L_trail * ca.cos(theta_v - theta_t)
        M[4, 3] = self.m_t * self.L_bar * self.L_trail * ca.cos(theta_d - theta_t)
        M[4, 4] = self.I_t + self.m_t * self.L_trail**2
        
        n = ca.MX.zeros(5, 1)
        n[0] = mdt * self.L_hitch * r**2 * ca.cos(theta_v) + mdt * self.L_bar * r_d**2 * ca.cos(theta_d) + self.m_t * self.L_trail * r_t**2 * ca.cos(theta_t)
        n[1] = mdt * self.L_hitch * r**2 * ca.sin(theta_v) + mdt * self.L_bar * r_d**2 * ca.sin(theta_d) + self.m_t * self.L_trail * r_t**2 * ca.sin(theta_t)
        n[2] = -mdt * self.L_hitch * self.L_bar * r_d**2 * ca.sin(theta_v - theta_d) - self.m_t * self.L_hitch * self.L_trail * r_t**2 * ca.sin(theta_v - theta_t)
        n[3] = mdt * self.L_hitch * self.L_bar * r**2 * ca.sin(theta_v - theta_d) - self.m_t * self.L_bar * self.L_trail * r_t**2 * ca.sin(theta_d - theta_t)
        n[4] = self.m_t * self.L_hitch * self.L_trail * r**2 * ca.sin(theta_v - theta_t) + self.m_t * self.L_bar * self.L_trail * r_d**2 * ca.sin(theta_d - theta_t)
        
        Q = ca.MX.zeros(5, 1)
        Q[0] = FX_f + FX_r + FX_d + FX_t
        Q[1] = FY_f + FY_r + FY_d + FY_t
        Q[2] = F_yf_v * self.l_f - F_yr_v * self.l_r - F_yd_v * self.L_hitch - F_yt_v * self.L_hitch
        Q[3] = -F_yd_d * self.L_bar - F_yt_d * self.L_bar
        Q[4] = -F_yt_t * (self.L_trail + self.l_rt)
        
        rhs = Q - n
        M_reg = M + ca.MX.eye(5) * 1e-4
        return M_reg, rhs
        
    def setup_optimizer(self):
        self.opti = ca.Opti()
        self.X = self.opti.variable(self.nx, self.N + 1)
        self.U = self.opti.variable(self.nu, self.N)
        self.A = self.opti.variable(5, self.N) # Accelerations as decision variables
        
        self.x_init = self.opti.parameter(self.nx)
        self.ref_traj = self.opti.parameter(3, self.N + 1) # [x, y, theta]
        self.v_ref = self.opti.parameter(1, self.N + 1)
        self.u_prev = self.opti.parameter(self.nu)
        
        q_ey = self.config['mpc']['weights']['lat_error']
        q_etheta = self.config['mpc']['weights']['heading_error']
        q_ev = self.config['mpc']['weights']['velocity_error']
        r_steer = self.config['mpc']['weights']['steering_input']
        r_accel = self.config['mpc']['weights']['acceleration_input']
        rd_steer = self.config['mpc']['weights']['steer_rate']
        rd_accel = self.config['mpc']['weights'].get('accel_rate', 100.0)

        obj = 0
        for k in range(self.N):
            # Error costs
            obj += q_ey * ((self.X[0, k] - self.ref_traj[0, k])**2 + (self.X[1, k] - self.ref_traj[1, k])**2)
            obj += q_etheta * (self.X[2, k] - self.ref_traj[2, k])**2
            
            # Forward velocity in body frame
            u_v = self.X[5, k] * ca.cos(self.X[2, k]) + self.X[6, k] * ca.sin(self.X[2, k])
            obj += q_ev * (u_v - self.v_ref[0, k])**2
            
            # Input costs
            obj += r_steer * self.U[0, k]**2
            obj += r_accel * self.U[1, k]**2
            
            # Rate costs
            if k == 0:
                obj += rd_steer * (self.U[0, k] - self.u_prev[0])**2
                obj += rd_accel * (self.U[1, k] - self.u_prev[1])**2
            if k > 0:
                # Normalize rate penalties to a base dt of 0.1 to match user's tuning scale!
                # If dt changes, this perfectly cancels out the (dU)^2 time-scaling effect.
                dt_scale = (0.1 / self.dt)**2
                obj += (rd_steer * dt_scale) * (self.U[0, k] - self.U[0, k-1])**2
                obj += (rd_accel * dt_scale) * (self.U[1, k] - self.U[1, k-1])**2
                
            # 1. Implicit Dynamics Formulation (Backward Euler for absolute stability!)
            # Evaluate forces at k+1 to make the integration unconditionally stable
            # Since tire models are now linear, IPOPT will solve this in 1 step without hanging!
            M_reg, rhs = self.get_M_rhs(self.X[:, k+1], self.U[:, k])
            scale = 1e-3
            self.opti.subject_to(ca.mtimes(M_reg, self.A[:, k]) * scale == rhs * scale)
            
            # 2. Backward Euler Integration (Unconditionally Stable)
            v_curr = self.X[5:10, k]
            v_next = self.X[5:10, k+1]
            p_curr = self.X[0:5, k]
            p_next = self.X[0:5, k+1]
            a_next = self.A[:, k]
            
            self.opti.subject_to(v_next == v_curr + a_next * self.dt)
            self.opti.subject_to(p_next == p_curr + v_next * self.dt)

        self.opti.subject_to(self.X[:, 0] == self.x_init)
        self.opti.subject_to(self.opti.bounded(-self.max_steer, self.U[0, :], self.max_steer))
        self.opti.subject_to(self.opti.bounded(-self.max_accel, self.U[1, :], self.max_accel))
        
        self.opti.minimize(obj)
        
        # IPOPT Options: Tuned for Real-Time Stiff Dynamics
        opts = {
            'ipopt.print_level': 0, 
            'print_time': 0, 
            'ipopt.max_iter': 300,      # Increased to allow convergence for longer horizons!
            'ipopt.tol': 1e-1, 
            'ipopt.acceptable_tol': 5e-1,
            'ipopt.acceptable_iter': 5,
            'ipopt.warm_start_init_point': 'yes',
            'ipopt.mu_strategy': 'adaptive'
        }
        self.opti.solver('ipopt', opts)

    def solve(self, x0, ref_traj, v_ref, u_prev):
        self.opti.set_value(self.x_init, x0)
        self.opti.set_value(self.ref_traj, ref_traj)
        self.opti.set_value(self.v_ref, v_ref)
        self.opti.set_value(self.u_prev, u_prev)
        
        # Robust Warm Start: ONLY use previous solution if it actually succeeded
        if getattr(self, 'solve_success', False) and hasattr(self, 'last_X_sol') and self.last_X_sol is not None:
            # Shift previous solution by 1 step
            X_guess = np.zeros((self.nx, self.N + 1))
            X_guess[:, :-1] = self.last_X_sol[:, 1:]
            X_guess[:, -1] = self.last_X_sol[:, -1]
            
            U_guess = np.zeros((self.nu, self.N))
            U_guess[:, :-1] = self.last_U_sol[:, 1:]
            U_guess[:, -1] = self.last_U_sol[:, -1]
            
            A_guess = np.zeros((5, self.N))
            if hasattr(self, 'last_A_sol') and self.last_A_sol is not None:
                A_guess[:, :-1] = self.last_A_sol[:, 1:]
                A_guess[:, -1] = self.last_A_sol[:, -1]
        else:
            # Perfect Kinematic forward projection for first step
            X_guess = np.tile(x0, (self.N + 1, 1)).T
            v_curr = math.hypot(x0[5], x0[6])
            theta = x0[2]
            for i in range(1, self.N + 1):
                X_guess[0, i] = X_guess[0, i-1] + v_curr * math.cos(theta) * self.dt
                X_guess[1, i] = X_guess[1, i-1] + v_curr * math.sin(theta) * self.dt
                X_guess[2:5, i] = x0[2:5] # Keep headings constant
                X_guess[5, i] = v_curr * math.cos(theta) # v_x
                X_guess[6, i] = v_curr * math.sin(theta) # v_y
                X_guess[7:10, i] = 0.0 # No yaw rate
                
            U_guess = np.tile(u_prev, (self.N, 1)).T
            A_guess = np.zeros((5, self.N))
            
        self.opti.set_initial(self.X, X_guess)
        self.opti.set_initial(self.U, U_guess)
        self.opti.set_initial(self.A, A_guess)
        
        try:
            sol = self.opti.solve()
            self.solve_success = True
            self.last_X_sol = sol.value(self.X)
            self.last_U_sol = sol.value(self.U)
            self.last_A_sol = sol.value(self.A)
            return self.last_U_sol[:, 0], self.last_X_sol
        except Exception as e:
            self.solve_success = False
            self.last_error_msg = str(e)
            try:
                # --- CRITICAL FIX: Prevent Control Lockups! ---
                # If IPOPT fails (e.g. max iterations reached), we MUST NOT freeze!
                # Using the suboptimal guess is infinitely better than locking the steering!
                sub_X = self.opti.debug.value(self.X)
                sub_U = self.opti.debug.value(self.U)
                return sub_U[:, 0], sub_X
            except:
                return u_prev, None

class MPCDynamicControlNode(Node):
    def __init__(self):
        super().__init__('mpc_dynamic_control_node')
        share_dir = get_package_share_directory('tractor_control')
        config_path = os.path.join(share_dir, 'config/mpc_config.yaml')
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
            
        self.controller = MPCDynamicController(self.config)
        
        self.control_pub = self.create_publisher(Control, '/mpc/control_cmd', 10)
        self.pred_path_pub = self.create_publisher(Path, '/mpc/predicted_trajectory', 10)
        self.autoware_control_pub = self.create_publisher(Control, '/external/selected/control_cmd', 10)
        
        # Autoware Integration
        self.gear_pub = self.create_publisher(GearShiftStamped, '/api/external/set/command/shift', 10)
        self.heartbeat_pub = self.create_publisher(Heartbeat, '/api/external/set/command/heartbeat', 10)
        
        self.sub_steer = self.create_subscription(SteeringReport, '/vehicle/status/steering_status', self.steer_callback, 10)
        self.actual_steer = 0.0
        
        # Action Clients
        self.engage_client = self.create_client(Engage, '/api/autoware/set/engage')
        self.remote_client = self.create_client(ChangeOperationMode, '/api/operation_mode/change_to_remote')
        self.local_client = self.create_client(ChangeOperationMode, '/api/operation_mode/change_to_local')
        
        self.enable_srv = self.create_service(SetBool, '/mpc/enable', self.enable_callback)
        self.enabled = False
        
        self.odom_sub = self.create_subscription(Odometry, '/localization/kinematic_state', self.odom_callback, 10)
        self.traj_sub = self.create_subscription(Trajectory, '/planning/scenario_planning/trajectory', self.traj_callback, 10)
        
        self.current_state = None
        self.full_ref_points = None
        self.u_prev = np.array([0.0, 0.0])
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.heartbeat_timer = self.create_timer(0.5, self.publish_heartbeat)
        
        # Initialize Debug Logger
        # This will create a new CSV file in the tractor_control/debug folder
        # and prevent using old values by keeping fresh logs.
        debug_dir = os.path.expanduser("~/AutoTract/tractor_ws/src/control/tractor_control/debug")
        self.csv_logger = MPCDebugLogger(log_dir=debug_dir)
        self.get_logger().info(f"MPC Debug Logger initialized. Saving to: {self.csv_logger.filename}")
        
        self.get_logger().info("Dynamic 5-DOF MPC Node Started")

    def enable_callback(self, request, response):
        self.enabled = request.data
        if self.enabled:
            self.get_logger().info("MPC Enabled! Taking over Autoware...")
            
            # Switch to Remote
            if self.remote_client.wait_for_service(timeout_sec=1.0):
                req = ChangeOperationMode.Request()
                self.remote_client.call_async(req)
            else:
                self.get_logger().warn("ChangeOperationMode service not available!")
                
            # Engage
            if self.engage_client.wait_for_service(timeout_sec=1.0):
                req = Engage.Request()
                req.engage = True
                self.engage_client.call_async(req)
            else:
                self.get_logger().warn("Engage service not available!")
                
            response.success = True
            response.message = "MPC Enabled and Autoware Takeover Requested"
        else:
            self.get_logger().info("MPC Disabled! Returning control...")
            if self.local_client.wait_for_service(timeout_sec=1.0):
                req = ChangeOperationMode.Request()
                self.local_client.call_async(req)
            response.success = True
            response.message = "MPC Disabled"
            
        return response

    def publish_heartbeat(self):
        if self.enabled:
            hb = Heartbeat()
            hb.stamp = self.get_clock().now().to_msg()
            self.heartbeat_pub.publish(hb)
            
            gs = GearShiftStamped()
            gs.stamp = hb.stamp
            gs.gear_shift.data = GearShift.DRIVE
            self.gear_pub.publish(gs)

    def steer_callback(self, msg):
        self.actual_steer = msg.steering_tire_angle

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
        
        v = msg.twist.twist.linear.x
        v_x = v * math.cos(yaw)
        v_y = v * math.sin(yaw)
        r = msg.twist.twist.angular.z
        
        if self.current_state is None:
            self.current_state = np.array([x, y, yaw, yaw, yaw, v_x, v_y, r, 0.0, 0.0])
        else:
            self.current_state = np.array([x, y, yaw, self.current_state[3], self.current_state[4], v_x, v_y, r, self.current_state[8], self.current_state[9]])

    def traj_callback(self, msg):
        self.full_ref_points = msg.points

    def timer_callback(self):
        if self.current_state is None or self.full_ref_points is None or len(self.full_ref_points) == 0:
            return
            
        dists = [(p.pose.position.x - self.current_state[0])**2 + (p.pose.position.y - self.current_state[1])**2 for p in self.full_ref_points]
        idx = np.argmin(dists)
        
        # --- Prevent the "Rabbit" from starting behind the vehicle! ---
        # If the closest point is physically behind the rear axle, the Euclidean penalty
        # will command the vehicle to brake/reverse! We must pick the next point ahead.
        while idx < len(self.full_ref_points) - 1:
            p = self.full_ref_points[idx]
            yaw = self.current_state[2]
            dx = p.pose.position.x - self.current_state[0]
            dy = p.pose.position.y - self.current_state[1]
            # Dot product to check if point is behind vehicle heading
            if dx * math.cos(yaw) + dy * math.sin(yaw) < 0:
                idx += 1
            else:
                break
        # --------------------------------------------------------------
        
        # --- Goal Tolerance & Auto-Disable Logic ---
        final_p = self.full_ref_points[-1]
        dist_to_goal = math.hypot(final_p.pose.position.x - self.current_state[0], final_p.pose.position.y - self.current_state[1])
        
        # Calculate heading error to final point
        q = final_p.pose.orientation
        final_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
        yaw_diff = (self.current_state[2] - final_yaw + math.pi) % (2 * math.pi) - math.pi
        yaw_error = abs(yaw_diff)
        
        goal_tolerance = 0.5     # [meters]
        heading_tolerance = 0.3  # [rad] (~17 degrees) Relaxed to prevent reversing!
        
        if dist_to_goal < goal_tolerance and yaw_error < heading_tolerance and final_p.longitudinal_velocity_mps == 0.0:
            # We reached the goal! Avoid singular solver at v=0 by stopping explicitly.
            if self.enabled:
                self.get_logger().info("Reached goal destination! Disabling MPC and stopping vehicle.")
                if self.engage_client.wait_for_service(timeout_sec=0.1):
                    req = Engage.Request()
                    req.engage = False
                    self.engage_client.call_async(req)
                self.enabled = False
            
            # Send Stop Command
            cmd = Control()
            cmd.stamp = self.get_clock().now().to_msg()
            cmd.lateral.stamp = cmd.stamp
            cmd.longitudinal.stamp = cmd.stamp
            cmd.lateral.steering_tire_angle = float(self.u_prev[0])
            cmd.longitudinal.velocity = 0.0
            cmd.longitudinal.acceleration = -1.0
            self.control_pub.publish(cmd)
            return
        # --------------------------------------------
        
        N = self.controller.N
        ref_traj = np.zeros((3, N + 1))
        v_ref = np.zeros((1, N + 1))
        
        # Build local path arc length (Expanded to 300 points to ensure the prediction horizon doesn't hit a wall)
        path_len = min(300, len(self.full_ref_points) - idx)
        local_points = self.full_ref_points[idx:idx+path_len]
        arc = [0.0]
        for i in range(1, len(local_points)):
            dx = local_points[i].pose.position.x - local_points[i-1].pose.position.x
            dy = local_points[i].pose.position.y - local_points[i-1].pose.position.y
            arc.append(arc[-1] + math.hypot(dx, dy))
            
        s_ref = 0.0
        dt = self.controller.dt
        
        for i in range(N + 1):
            idx_s = np.searchsorted(arc, s_ref)
            if idx_s == 0:
                p = local_points[0]
            elif idx_s >= len(local_points):
                p = local_points[-1]
            else:
                p = local_points[idx_s]
                
            ref_traj[0, i] = p.pose.position.x
            ref_traj[1, i] = p.pose.position.y
            q = p.pose.orientation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
            
            # Unwrap Yaw to prevent 360-degree rotation attempts by IPOPT
            if i == 0:
                prev_yaw = self.current_state[2]
            else:
                prev_yaw = ref_traj[2, i-1]
                
            while yaw - prev_yaw > math.pi: yaw -= 2*math.pi
            while yaw - prev_yaw < -math.pi: yaw += 2*math.pi
            
            ref_traj[2, i] = yaw
                
            # Read velocity directly from Autoware planner so we can STOP smoothly
            v_ref[0, i] = p.longitudinal_velocity_mps
            
            # Advance spatial reference using the ACTUAL velocity reference!
            s_ref += p.longitudinal_velocity_mps * dt
            
        # Only use actual_steer if it's actually updating (not dead at 0.0)!
        # Otherwise, trust our last command to maintain rate continuity.
        if hasattr(self, 'actual_steer') and abs(self.actual_steer) > 0.001:
            self.u_prev[0] = self.actual_steer
            
        import time
        start_solve = time.time()
        u_opt, x_pred = self.controller.solve(self.current_state, ref_traj, v_ref, self.u_prev)
        solve_time_ms = (time.time() - start_solve) * 1000.0
        
        if x_pred is not None:
            self.u_prev[0] = u_opt[0] # MUST update prev steer to maintain continuity!
            self.u_prev[1] = u_opt[1] # Keep accel
            self.current_state[3] = x_pred[3, 1]
            self.current_state[4] = x_pred[4, 1]
            self.current_state[8] = x_pred[8, 1]
            self.current_state[9] = x_pred[9, 1]
            
            # self.get_logger().info(f"MPC Solved: Steer={math.degrees(u_opt[0]):.1f}deg, Accel={u_opt[1]:.2f}")
        else:
            err_msg = getattr(self.controller, 'last_error_msg', 'Unknown Error')
            self.get_logger().warn(f"MPC Failed to solve! Using previous input. Reason: {err_msg}")
            u_opt = self.u_prev
            
        cmd = Control()
        cmd.stamp = self.get_clock().now().to_msg()
        cmd.lateral.stamp = cmd.stamp
        cmd.longitudinal.stamp = cmd.stamp
        
        if self.enabled:
            cmd.lateral.steering_tire_angle = float(u_opt[0])
            
            # Forward the MPC's optimal predicted velocity, not the raw reference!
            if x_pred is not None:
                # Convert global vx, vy to body longitudinal speed
                u_v_pred = x_pred[5, 1] * math.cos(x_pred[2, 1]) + x_pred[6, 1] * math.sin(x_pred[2, 1])
                cmd.longitudinal.velocity = float(u_v_pred)
            else:
                cmd.longitudinal.velocity = float(v_ref[0, 0])
                
            cmd.longitudinal.acceleration = float(u_opt[1])
            cmd.longitudinal.is_defined_acceleration = True
            
            # --- Log Data to CSV ---
            try:
                # Extract starting point from the trajectory (used by MPC)
                start_p = self.full_ref_points[idx]
                start_coords = (start_p.pose.position.x, start_p.pose.position.y)
                goal_coords = (final_p.pose.position.x, final_p.pose.position.y)
                
                # reference at k=0
                ref_0 = (ref_traj[0, 0], ref_traj[1, 0], ref_traj[2, 0], v_ref[0, 0])
                
                # cmd
                cmd_data = (cmd.lateral.steering_tire_angle, cmd.longitudinal.acceleration, cmd.longitudinal.velocity)
                
                # feedback
                fb_data = (
                    self.current_state[0], self.current_state[1], self.current_state[2], 
                    self.current_state[5], self.current_state[6], self.current_state[7], 
                    self.actual_steer
                )
                
                # Log step
                self.csv_logger.log_step(
                    ts=cmd.stamp.sec + cmd.stamp.nanosec * 1e-9,
                    start=start_coords,
                    goal=goal_coords,
                    ref_traj=ref_0,
                    cmd=cmd_data,
                    fb=fb_data,
                    solve_time_ms=solve_time_ms,
                    dist_to_goal=dist_to_goal
                )
            except Exception as e:
                self.get_logger().error(f"Failed to log CSV: {e}")
            # -----------------------
            
            if x_pred is not None:
                path = Path()
                path.header.frame_id = "map"
                path.header.stamp = cmd.stamp
                for i in range(x_pred.shape[1]):
                    p = PoseStamped()
                    p.pose.position.x = float(x_pred[0, i])
                    p.pose.position.y = float(x_pred[1, i])
                    path.poses.append(p)
                self.pred_path_pub.publish(path)
        else:
            cmd.lateral.steering_tire_angle = 0.0
            cmd.longitudinal.velocity = 0.0
            cmd.longitudinal.acceleration = -1.0
            cmd.longitudinal.is_defined_acceleration = True
            
        self.control_pub.publish(cmd)
        self.autoware_control_pub.publish(cmd)
        

def main(args=None):
    rclpy.init(args=args)
    node = MPCDynamicControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
