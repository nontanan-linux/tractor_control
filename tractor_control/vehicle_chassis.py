from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import TransformStamped
import numpy as np
import math

class VehicleSimulator:
    """
    Simulates the specific Kinematic Bicycle Model (Rear Axle Ref)
    State: [x, y, theta, v]
    """
    def __init__(self, x=0.0, y=0.0, theta=0.0, v=0.0, 
                 wheelbase=2.5, track_width=1.2, max_steer=0.6, max_accel=2.0, dt=0.1,
                 chassis_length=3.5, chassis_width=1.6, chassis_height=1.0, wheel_radius=0.4):
        self.state = np.array([x, y, theta, v])
        self.L = wheelbase
        self.TRACK_WIDTH = track_width
        self.MAX_STEER = max_steer
        self.MAX_ACCEL = max_accel
        self.DT = dt
        
        # Visualization params
        self.CHASSIS_LENGTH = chassis_length
        self.CHASSIS_WIDTH = chassis_width
        self.CHASSIS_HEIGHT = chassis_height
        self.WHEEL_RADIUS = wheel_radius
        self.wheel_angle = 0.0
        
    def update(self, u):
        """
        Update state based on non-linear kinematic bicycle model.
        u = [delta, a]
        """
        x, y, theta, v = self.state
        delta = u[0]
        a = u[1]
        
        # Clip inputs
        delta = np.clip(delta, -self.MAX_STEER, self.MAX_STEER)
        a = np.clip(a, -self.MAX_ACCEL, self.MAX_ACCEL)

        # Kinematic Bicycle Model equations (Rear Axle Reference)
        dx = v * math.cos(theta)
        dy = v * math.sin(theta)
        dtheta = (v * math.tan(delta)) / self.L
        dv = a
        
        # Euler Integration
        self.state[0] += dx * self.DT
        self.state[1] += dy * self.DT
        self.state[2] += dtheta * self.DT
        self.state[3] += dv * self.DT
        
        # Normalize angle
        self.state[2] = math.atan2(math.sin(self.state[2]), math.cos(self.state[2]))
        
        # Update wheel rolling angle for visualization
        self.wheel_angle += (v * self.DT) / self.WHEEL_RADIUS
        
        return self.state

    def get_marker_array(self, timestamp, steering_angle=0.0):
        marker_array = MarkerArray()
        
        # 1. Chassis
        chassis = Marker()
        chassis.header.frame_id = "base_link"
        chassis.header.stamp = timestamp
        chassis.ns = "vehicle_body"
        chassis.id = 0
        chassis.type = Marker.CUBE
        chassis.action = Marker.ADD
        chassis.scale.x = self.CHASSIS_LENGTH
        chassis.scale.y = self.CHASSIS_WIDTH
        chassis.scale.z = self.CHASSIS_HEIGHT
        chassis.color.a = 0.4
        chassis.color.r = 0.0
        chassis.color.g = 1.0
        chassis.color.b = 0.0
        chassis.pose.position.x = self.L / 2.0 
        chassis.pose.position.y = 0.0
        chassis.pose.position.z = self.CHASSIS_HEIGHT / 2.0
        chassis.pose.orientation.w = 1.0
        marker_array.markers.append(chassis)
        
        # 2. Wheels
        half_track = self.TRACK_WIDTH / 2.0
        wheel_offsets = [
            (0.0, -half_track, self.WHEEL_RADIUS), # Rear Right
            (0.0,  half_track, self.WHEEL_RADIUS), # Rear Left
            (self.L, -half_track, self.WHEEL_RADIUS), # Front Right
            (self.L,  half_track, self.WHEEL_RADIUS)  # Front Left
        ]
        
        qw_norm = self._quaternion_from_euler(1.5707, 0, 0)
        
        for i, (x, y, z) in enumerate(wheel_offsets):
            wheel = Marker()
            wheel.header.frame_id = "base_link"
            wheel.header.stamp = timestamp
            wheel.ns = "vehicle_wheels"
            wheel.id = i+1
            wheel.type = Marker.CYLINDER
            wheel.action = Marker.ADD
            wheel.scale.x = self.WHEEL_RADIUS * 2
            wheel.scale.y = self.WHEEL_RADIUS * 2
            wheel.scale.z = 0.3
            wheel.color.a = 1.0
            wheel.color.r = 0.2
            wheel.color.g = 0.2
            wheel.color.b = 0.2
            
            wheel.pose.position.x = x
            wheel.pose.position.y = y
            wheel.pose.position.z = z
            
            if i >= 2: # Front wheels
                q_steer = self._quaternion_from_euler(0, 0, steering_angle)
                q = self._quaternion_multiply(q_steer, qw_norm)
            else:
                q = qw_norm
            
            wheel.pose.orientation.x = q[0]
            wheel.pose.orientation.y = q[1]
            wheel.pose.orientation.z = q[2]
            wheel.pose.orientation.w = q[3]
            
            marker_array.markers.append(wheel)
            
        return marker_array

    def get_wheel_tfs(self, timestamp, steering_angle=0.0):
        half_track = self.TRACK_WIDTH / 2.0
        wheel_data = [
            ("rear_right_wheel", 0.0, -half_track, self.WHEEL_RADIUS, 0.0),
            ("rear_left_wheel",  0.0,  half_track, self.WHEEL_RADIUS, 0.0),
            ("front_right_wheel", self.L, -half_track, self.WHEEL_RADIUS, steering_angle),
            ("front_left_wheel",  self.L,  half_track, self.WHEEL_RADIUS, steering_angle)
        ]
        
        tfs = []
        for name, x, y, z, steer in wheel_data:
            t = TransformStamped()
            t.header.stamp = timestamp
            t.header.frame_id = "base_link"
            t.child_frame_id = name
            
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z
            
            q_steer = self._quaternion_from_euler(0, 0, steer)
            q_roll = self._quaternion_from_euler(0, self.wheel_angle, 0)
            q = self._quaternion_multiply(q_steer, q_roll)
            
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            tfs.append(t)
            
        return tfs

    def _quaternion_from_euler(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = sr * cp * cy - cr * sp * sy
        q[1] = cr * sp * cy + sr * cp * sy
        q[2] = cr * cp * sy - sr * sp * cy
        q[3] = cr * cp * cy + sr * sp * sy
        return q

    def _quaternion_multiply(self, q1, q2):
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        ]
