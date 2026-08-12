import csv
import time
import os
import math

class MPCDebugLogger:
    def __init__(self, log_dir="/tmp/mpc_logs"):
        self.log_dir = log_dir
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
            
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        self.filename = os.path.join(self.log_dir, f"mpc_log_{timestamp}.csv")
        
        self.file = open(self.filename, mode='w', newline='')
        self.writer = csv.writer(self.file)
        
        # Write header
        self.writer.writerow([
            "timestamp", 
            "start_x", "start_y", 
            "goal_x", "goal_y", 
            "ref_x", "ref_y", "ref_yaw", "ref_v",
            "cmd_steer", "cmd_accel", "cmd_v",
            "fb_x", "fb_y", "fb_yaw", "fb_v_x", "fb_v_y", "fb_r", "fb_steer",
            "solve_time_ms",
            "dist_to_goal"
        ])
        
    def log_step(self, ts, start, goal, ref_traj, cmd, fb, solve_time_ms, dist_to_goal):
        """
        start: (x, y)
        goal: (x, y)
        ref_traj: (x, y, yaw, v) for the CURRENT step (k=0)
        cmd: (steer, accel, v)
        fb: (x, y, yaw, v_x, v_y, r, steer)
        """
        row = [
            ts,
            start[0], start[1],
            goal[0], goal[1],
            ref_traj[0], ref_traj[1], ref_traj[2], ref_traj[3],
            cmd[0], cmd[1], cmd[2],
            fb[0], fb[1], fb[2], fb[3], fb[4], fb[5], fb[6],
            solve_time_ms,
            dist_to_goal
        ]
        self.writer.writerow(row)
        self.file.flush() # Ensure it's written immediately

    def close(self):
        if self.file and not self.file.closed:
            self.file.close()
