#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from autoware_control_msgs.msg import Control
from autoware_auto_vehicle_msgs.msg import VelocityReport
import sys
import select
import termios
import tty
import collections
from datetime import datetime

# Autoware Messages
try:
    from tier4_external_api_msgs.msg import Heartbeat, GearShiftStamped, GearShift
    from tier4_external_api_msgs.srv import Engage
    HAS_TIER4 = True
except ImportError:
    HAS_TIER4 = False

try:
    from autoware_adapi_v1_msgs.srv import ChangeOperationMode
    HAS_ADAPI = True
except ImportError:
    HAS_ADAPI = False

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# Rich Library
from rich.live import Live
from rich.layout import Layout
from rich.panel import Panel
from rich.align import Align
from rich.text import Text
from rich.progress_bar import ProgressBar
from rich.table import Table

settings = termios.tcgetattr(sys.stdin)

class ActuationControl:
    """
    Closed-loop PID velocity controller converting Velocity Error into Throttle and Brake outputs.
    """
    def __init__(self, kp=0.5, ki=0.05, kd=0.1, max_speed=5.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_speed = max_speed
        
        self.integral = 0.0
        self.prev_error = 0.0
        
    def compute_command(self, target_velocity, real_velocity, dt=0.1):
        if target_velocity == 0.0:
            self.integral = 0.0
            self.prev_error = 0.0
            return 0.0, 1.0  # Zero throttle, Full brake
            
        # Compare speed magnitudes to operate independently of driving direction
        sp = abs(target_velocity)
        act = abs(real_velocity)
        
        error = sp - act
        self.integral += error * dt
        # Anti-windup clamping
        self.integral = max(-2.0, min(2.0, self.integral))
        
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        
        if output > 0.0:
            # Acceleration needed
            throttle = min(1.0, output)
            brake = 0.0
        else:
            # Deceleration needed
            throttle = 0.0
            brake = min(1.0, -output)
            
        return float(throttle), float(brake)


class AckermannTeleop(Node):
    def __init__(self):
        super().__init__('ackermann_teleop')
        
        # System Logs (deque for max 10 lines)
        self.system_logs = collections.deque(maxlen=10)
        self.add_log("Initializing Velocity Teleop Node...")
        
        # Use Reliable + Transient Local QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Always use standard Control message directly to vehicle_cmd_gate
        self.control_pub = self.create_publisher(Control, '/external/selected/control_cmd', qos_profile)
        self.add_log("[green]Using standard Control -> /external/selected/control_cmd[/]")
        
        if HAS_TIER4:
            self.heartbeat_pub = self.create_publisher(Heartbeat, '/api/external/set/command/remote/heartbeat', 10)
            self.gear_pub = self.create_publisher(GearShiftStamped, '/api/external/set/command/remote/shift', 10)
            self.engage_client = self.create_client(Engage, '/api/autoware/set/engage')
            
        if HAS_ADAPI:
            self.remote_client = self.create_client(ChangeOperationMode, '/api/operation_mode/change_to_remote')
            self.local_client = self.create_client(ChangeOperationMode, '/api/operation_mode/change_to_local')
            self.has_adapi = True
        else:
            self.has_adapi = False
        
        # Velocity Status Subscriber
        self.vel_sub = self.create_subscription(
            VelocityReport, 
            '/vehicle/status/velocity_status', 
            self.velocity_callback, 
            10
        )
        
        # Real Sensor Feedback Variables
        self.real_velocity = 0.0
        self.real_heading_rate = 0.0

        # Control Variables
        self.target_velocity = 0.0
        self.max_velocity = 5.0
        self.velocity_step = 0.1
        
        self.steering = 0.0
        self.steering_step = 0.02
        self.max_steering = 0.6
        self.enabled = False
        
        # Actuation Controller & Current Output Buffers
        self.actuation = ActuationControl(max_speed=self.max_velocity)
        self.throttle_cmd = 0.0
        self.brake_cmd = 1.0
        
        # Sensor Telemetry History (width=26 columns, dot_width=52 sub-pixels)
        self.vel_act_hist = collections.deque([0.0]*52, maxlen=52)
        self.heading_rate_hist = collections.deque([0.0]*52, maxlen=52)
        
        # Rich Layout & Live Setup
        self.layout = self.generate_layout()
        self.live = Live(self.layout, auto_refresh=False, screen=True)
        self.live.start()
        
        self.add_log("[green]Ready! Press Space to Enable & Engage.[/]")
        
        self.timer = self.create_timer(0.1, self.timer_callback)

    def velocity_callback(self, msg):
        self.real_velocity = msg.longitudinal_velocity
        self.real_heading_rate = msg.heading_rate

    def add_log(self, message: str):
        now = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.system_logs.append(f"[[cyan]{now}[/]] {message}")

    def generate_layout(self) -> Layout:
        layout = Layout()
        layout.split_column(
            Layout(name="header", size=3),
            Layout(name="body", size=15),
            Layout(name="bottom")
        )
        layout["body"].split_row(
            Layout(name="body_left", ratio=1),
            Layout(name="body_right", ratio=1)
        )
        layout["bottom"].split_row(
            Layout(name="shortcuts", ratio=1),
            Layout(name="logs", ratio=2)
        )
        return layout

    def get_ratatui_chart(self, history, min_val, max_val, style="green"):
        char_width = 26
        char_height = 4
        dot_width = char_width * 2
        dot_height = char_height * 4
        
        grid = [[0 for _ in range(dot_width)] for _ in range(dot_height)]
        span = max_val - min_val
        if span == 0: span = 1.0
        
        # Populate grid with padded left history
        hist_list = list(history)
        missing = dot_width - len(hist_list)
        padded = [min_val]*missing + hist_list
        
        for x in range(dot_width):
            v = padded[x]
            norm = max(0.0, min(1.0, (v - min_val) / span))
            y = dot_height - 1 - int(norm * (dot_height - 0.001))
            grid[y][x] = 1
            
        lines = []
        for r in range(char_height):
            row_chars = []
            for c in range(char_width):
                w = 0
                if grid[r*4 + 0][c*2 + 0]: w += 1
                if grid[r*4 + 1][c*2 + 0]: w += 2
                if grid[r*4 + 2][c*2 + 0]: w += 4
                if grid[r*4 + 3][c*2 + 0]: w += 64
                if grid[r*4 + 0][c*2 + 1]: w += 8
                if grid[r*4 + 1][c*2 + 1]: w += 16
                if grid[r*4 + 2][c*2 + 1]: w += 32
                if grid[r*4 + 3][c*2 + 1]: w += 128
                row_chars.append(chr(0x2800 + w))
            lines.append(f"[{style}]" + "".join(row_chars) + "[/]")
            
        l_max = f"{max_val:5.1f}"
        l_mid = f"{(max_val+min_val)/2:5.1f}"
        l_min = f"{min_val:5.1f}"
        
        chart_text = (
            f" [cyan]{l_max}[/] │ {lines[0]}\n"
            f"       │ {lines[1]}\n"
            f" [cyan]{l_mid}[/] │ {lines[2]}\n"
            f" [cyan]{l_min}[/] │ {lines[3]}\n"
            f"       └" + "─" * char_width + "\n"
            f"       [grey37]-5.2s{' ' * (char_width - 9)}Now[/]"
        )
        return chart_text

    def update_layout(self):
        # Update Sensor History Deques
        self.vel_act_hist.append(self.real_velocity)
        self.heading_rate_hist.append(self.real_heading_rate)

        # Header
        status_color = "bold green" if self.enabled else "bold red"
        status_text = "ENABLED" if self.enabled else "DISABLED"
        gear_str = "REVERSE" if self.target_velocity < 0.0 else "DRIVE"
        
        header_text = Text(f"Autoware Ackermann Teleop Control | STATUS: [{status_text}] | GEAR: [{gear_str}]", justify="center")
        header_text.stylize(status_color, 49, 49 + len(status_text) + 2)
        self.layout["header"].update(Panel(header_text, style="white on blue" if self.enabled else "white on grey37"))

        # Body Left: Visual Bars (Command State)
        vel_norm = (self.target_velocity + self.max_velocity) / (2 * self.max_velocity)
        vel_idx = max(0, min(30, int(vel_norm * 30)))
        vel_arr = ['─'] * 31
        vel_arr[15] = '┼'
        if vel_idx == 15:
            vel_arr[vel_idx] = '┿'
        else:
            vel_arr[vel_idx] = '█'
        vel_str = "".join(vel_arr)

        throttle_bar = ProgressBar(total=1.0, completed=self.throttle_cmd, style="green", width=30)
        brake_bar = ProgressBar(total=1.0, completed=self.brake_cmd, style="red", width=30)
        
        s_norm = (self.steering + self.max_steering) / (2 * self.max_steering)
        # Invert visual mapping: positive steering (left turn) moves block to the left
        steer_idx = max(0, min(30, int((1.0 - s_norm) * 30)))
        steer_arr = ['─'] * 31
        steer_arr[15] = '┼'
        if steer_idx == 15:
            steer_arr[steer_idx] = '┿'
        else:
            steer_arr[steer_idx] = '█'
        steer_str = "".join(steer_arr)

        body_table = Table.grid(padding=(0, 2))
        body_table.add_column("Label", justify="right", style="bold yellow")
        body_table.add_column("Bar", justify="center")
        body_table.add_column("Value", justify="left")
        
        body_table.add_row("TARGET VEL", f"[magenta]{vel_str}[/]", f"[magenta]{self.target_velocity:5.2f} m/s[/]")
        body_table.add_row("THROTTLE", throttle_bar, f"[green]{self.throttle_cmd*100:3.0f}%[/]")
        body_table.add_row("BRAKE", brake_bar, f"[red]{self.brake_cmd*100:3.0f}%[/]")
        body_table.add_row("STEERING", f"[cyan]{steer_str}[/]", f"[cyan]{self.steering:5.2f} rad[/]")
        
        self.layout["body_left"].update(Panel(Align.center(body_table, vertical="middle"), title="Actuation State"))

        # Body Right: Premium Ratatui-Style Braille Charts
        v_chart = self.get_ratatui_chart(self.vel_act_hist, -self.max_velocity, self.max_velocity, style="green")
        h_chart = self.get_ratatui_chart(self.heading_rate_hist, -1.0, 1.0, style="cyan")
        
        dash_content = (
            f"[bold yellow]─ Longitudinal Velocity Feedback[/] ([green]{self.real_velocity:+.2f} m/s[/])\n"
            f"{v_chart}\n"
            f"[bold yellow]─ Heading Rate Feedback[/] ([cyan]{self.real_heading_rate:+.2f} rad/s[/])\n"
            f"{h_chart}"
        )
        
        self.layout["body_right"].update(Panel(dash_content, title="Sensor Feedback Stream (/vehicle/status/velocity_status)"))

        # Bottom Left: Shortcuts
        shortcut_table = Table(show_header=False, show_edge=False, box=None)
        shortcut_table.add_column("Key", style="bold cyan")
        shortcut_table.add_column("Action")
        shortcut_table.add_row("[W/X]", "Speed Up / Down")
        shortcut_table.add_row("[A/D]", "Steer Left / Right")
        shortcut_table.add_row("[S]", "Emergency Stop")
        shortcut_table.add_row("[Z]", "Center Steering")
        shortcut_table.add_row("[Space]", "Toggle Enable/Engage")
        shortcut_table.add_row("[Ctrl+C]", "Quit")
        self.layout["bottom"]["shortcuts"].update(Panel(shortcut_table, title="Keyboard Shortcuts"))

        # Bottom Right: Logs
        log_text = Text.from_markup("\n".join(self.system_logs))
        self.layout["bottom"]["logs"].update(Panel(log_text, title="System Logs"))

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = None
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, settings)
        return key

    def call_engage(self):
        if getattr(self, 'has_adapi', False):
            self.add_log("Changing Operation Mode to REMOTE...")
            req_remote = ChangeOperationMode.Request()
            self.remote_client.call_async(req_remote)
            
        if HAS_TIER4:
            self.add_log("Calling Engage Service...")
            req = Engage.Request()
            req.engage = True
            self.engage_client.call_async(req)

    def call_disengage(self):
        if getattr(self, 'has_adapi', False):
            self.add_log("Changing Operation Mode to LOCAL...")
            req_local = ChangeOperationMode.Request()
            self.local_client.call_async(req_local)

    def timer_callback(self):
        key = self.get_key()
        if key == ' ':
            self.enabled = not self.enabled
            self.target_velocity = 0.0
            self.steering = 0.0
            if self.enabled:
                self.add_log("[bold green]System ENABLED and ENGAGED[/]")
                self.call_engage()
            else:
                self.add_log("[bold yellow]System DISABLED (Returned to Local)[/]")
                self.call_disengage()
        
        if key == 'w':
            self.target_velocity = min(self.max_velocity, self.target_velocity + self.velocity_step)
        elif key == 'x':
            self.target_velocity = max(-self.max_velocity, self.target_velocity - self.velocity_step)
        elif key == 'a':
            self.steering += self.steering_step
        elif key == 'd':
            self.steering -= self.steering_step
        elif key == 's':
            self.target_velocity = 0.0
            self.add_log("[red]Emergency Stop (Speed = 0)[/]")
        elif key == 'z':
            self.steering = 0.0
        elif key == '\x03':  # Ctrl-C
            self.live.stop()
            rclpy.shutdown()
            sys.exit()

        self.steering = max(-self.max_steering, min(self.max_steering, self.steering))

        now = self.get_clock().now().to_msg()
        
        if self.enabled:
            self.throttle_cmd, self.brake_cmd = self.actuation.compute_command(self.target_velocity, self.real_velocity, dt=0.1)
        else:
            self.throttle_cmd, self.brake_cmd = 0.0, 1.0
            self.actuation.integral = 0.0
            self.actuation.prev_error = 0.0

        if HAS_TIER4:
            hb = Heartbeat()
            hb.stamp = now
            self.heartbeat_pub.publish(hb)
            
            gs = GearShiftStamped()
            gs.stamp = now
            if self.target_velocity < 0.0:
                gs.gear_shift.data = GearShift.REVERSE
            else:
                gs.gear_shift.data = GearShift.DRIVE
            self.gear_pub.publish(gs)

        # Publish Standard Control Command to External Selected Control
        cmd = Control()
        cmd.stamp = now
        cmd.lateral.stamp = now
        cmd.lateral.steering_tire_angle = float(self.steering)
        
        cmd.longitudinal.stamp = now
        cmd.longitudinal.velocity = float(self.target_velocity)
        cmd.longitudinal.acceleration = float(self.throttle_cmd * 2.0 - self.brake_cmd * 2.0)
        cmd.longitudinal.is_defined_acceleration = True
        self.control_pub.publish(cmd)
            
        self.update_layout()
        self.live.refresh()

def main(args=None):
    rclpy.init(args=args)
    node = AckermannTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.live.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
