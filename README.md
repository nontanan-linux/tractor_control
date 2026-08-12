# Tractor Control Package

The `tractor_control` package is a ROS 2-based autonomous control system designed for tractor-trailer systems and standard vehicles. It features a Nonlinear Model Predictive Control (NMPC) module, a dynamic Grey Wolf Optimizer (GWO) for real-time parameter tuning, and necessary bridge nodes for interfacing with the Autoware autonomous driving stack.

## 🎯 Key Features

* **MPC Controller (`mpc_node`)**: A Model Predictive Control node that handles both lateral and longitudinal vehicle dynamics. It calculates the optimal steering and acceleration by minimizing state errors against a reference trajectory.
* **Tracter Control Node (`tractor_control.py`)**: The primary Autoware integration node. It listens to Autoware's scenario planner (`/planning/scenario_planning/trajectory`) and localization states, performs NMPC calculations, and acts as an external remote controller by publishing directly to Autoware's `external_cmd_selector`.
* **GWO Tuner (`gwo_tuner_node`)**: Uses the Grey Wolf Optimizer algorithm to dynamically adjust MPC penalty weights (Q and R matrices) in real-time based on tracking performance, ensuring a smooth and accurate ride.
* **Path Publisher (`path_publisher_node`)**: Loads reference path coordinates from `.csv` files and publishes them as a `nav_msgs/Path` along with detailed trajectory states (speed, curvature) for the basic MPC to follow.
* **Vehicle Simulation (`vehicle_node`)**: Provides a software-in-the-loop kinematics simulator for testing without real hardware. It calculates and publishes the vehicle's simulated odometry based on control commands.
* **Planning Simulator**: A comprehensive simulation setup that combines path generation, NMPC control, and vehicle dynamics to validate planning algorithms for tractor-trailer systems.
* **Control Selector (`control_selector_node`)**: An older bridging node previously used to intercept and replace the autonomous control stream directly before it reaches the vehicle interface.
* **Manual Control (`ackermann_teleop_node.py`)**: A keyboard-based teleoperation node designed to seamlessly interface with Autoware's External API to allow manual overriding and vehicle driving.

## 📂 Package Structure

```text
tractor_control/
├── config/              # Configuration files (e.g., mpc_config.yaml)
├── launch/              # ROS 2 Launch files
├── paths/               # Reference trajectories (.csv files)
├── urdf/                # Unified Robot Description Format files
├── simulation/          # Simulation & testing nodes (Standalone NMPC)
│   ├── __init__.py
│   ├── mpc_node.py
│   ├── gwo_tuner_node.py
│   ├── path_publisher.py
│   └── generate_path.py
└── tractor_control/     # Main control nodes
    ├── __init__.py
    ├── ackermann_teleop_node.py # Keyboard manual control
    ├── nmpc_control_node.py # NMPC Node (Tractor-Trailer)
    ├── tractor_control.py
    ├── vehicle_node.py
    ├── vehicle_chassis.py
    └── control_selector_node.py
```

## 🚀 Installation & Usage

1. **Build the Package**:
   Use the clean build script located in your workspace root.

   ```bash
   cd ~/AutoTract
   ./clean_build.sh
   source install/setup.bash
   ```

2. **Bringing up the System via Launch File** *(Example)*:

   ```bash
   ros2 launch tractor_control tractor_control.launch.py
   ```

3. **Running Standalone Nodes**:
   * **Start the Nonlinear MPC (Tractor-Trailer)**:
     `ros2 run tractor_control nmpc_control_node`
   * **Start the standalone MPC**:
     `ros2 run tractor_control mpc_node`
   * **Start the Autoware Integration Node (Tracter Control)**:
     `ros2 run tractor_control tractor_control_node`
   * **Start the GWO Tuner**:
     `ros2 run tractor_control gwo_tuner_node`

## ⚙️ MPC Parameters

All core parameters are located in `config/mpc_config.yaml`:
* `dt`: Control time step
* `prediction_horizon`: The number of steps the MPC looks ahead into the future
* `weights`: Default penalty weights (Lat/Heading Error, Steer Rate, etc.) used if GWO is disabled
* `vehicle`: Dimensions and physical limits constraints (wheelbase, max steer angle, etc.)

## 🎮 Planning Simulator
The planning simulator allows you to test the NMPC controller within the full Autoware environment.

1. **Launch Autoware Planning Simulator (Heli Tow Tractor 12t)**:
   ```bash
   source install/setup.bash
   # Note: map_path must be an absolute path
   ros2 launch tracter_launch planning_simulator.launch.xml map_path:=/home/nontanan/autoware_data/BG_new
   ```
2. **Initialize and Set Goal**:
   In RViz, use the **2D Pose Estimate** tool to set the vehicle's starting position and the **2D Nav Goal** tool to set the destination.
3. **Run NMPC Control**:
   ```bash
   ros2 run tractor_control nmpc_control_node
   ```
4. **Switch Autoware to Remote Mode**:
   In a new terminal, call the service to activate external control:
   ```bash
   ros2 service call /api/operation_mode/change_to_remote autoware_adapi_v1_msgs/srv/ChangeOperationMode
   ```
5. **Engage the Vehicle**:
   Finally, click the **Engage** button in the RViz Autoware State Panel, or run:
   ```bash
   ros2 service call /api/autoware/set/engage tier4_external_api_msgs/srv/Engage "{engage: true}"
   ```
6. **Verify Remote Control**:
   To ensure Autoware is receiving commands from your node instead of its internal controller, check the `vehicle_cmd_gate` mode:
   ```bash
   ros2 topic echo /control/vehicle_cmd_gate/operation_mode
   ```
   **Operation Mode Values:**
   * `1 (STOP)`: Vehicle is stopped for safety.
   * `2 (AUTONOMOUS)`: Vehicle is controlled by Autoware's internal MPC/Pure Pursuit.
   * `3 (LOCAL)`: Vehicle is controlled by local hardware/joystick.
   * `4 (REMOTE)`: Vehicle is controlled by external API (Your `nmpc_control_node`!).

   *(If the output shows `mode: 4`, it means Autoware is in Remote Mode and your node is successfully driving the vehicle!)*

7. **Switch Back to Autoware Control (Optional)**:
   If you want to stop your custom NMPC and let Autoware drive itself again, change the mode back to Autonomous:
   ```bash
   ros2 service call /api/operation_mode/change_to_autonomous autoware_adapi_v1_msgs/srv/ChangeOperationMode
   ```

## 🕹️ Manual Control (Teleop)
You can manually control the vehicle using the keyboard. By default, this node integrates with Autoware's External API, publishing `ControlCommandStamped` (Throttle/Brake/Steer) and automatically handling Heartbeat and Gear signals.

**Run Teleop Node**:
```bash
ros2 run tractor_control ackermann_teleop_node
```

### Key Mapping:
| Key | Action |
| --- | --- |
| **W / X** | Increase / Decrease Throttle and Brake |
| **A / D** | Increase / Decrease Steering Angle |
| **S** | Stop (Apply Full Brake) |
| **Z** | Reset Steering to 0 |
| **Space** | **Toggle ENABLE / DISABLE** (Auto-engages and switches to Remote Mode) |
| **Ctrl+C** | Quit |

> [!TIP]
> **To start driving:** Run the node and press **Spacebar** once. This will enable the controller, switch Autoware to Remote Mode, and send the Engage command automatically. Then press **`w`** to accelerate. Press **Spacebar** again to disable and force a safe stop.

## 🔌 Autoware Integration

This package has been updated to output commands using the standard `autoware_control_msgs/msg/Control` suitable for modern Autoware Universe. You can inject these commands into the Autoware stack specifically using the **External/Remote Mode**:

Through `nmpc_control_node`, the custom NMPC will natively subscribe to modern Autoware topics (`/planning/scenario_planning/trajectory` and `/localization/kinematic_state`) and publish its control commands directly to `/external/selected/control_cmd`. To activate the custom MPC during runtime, simply use Autoware's API or RViz interface to change the active driving mode from **Auto** to **Remote**.

## 🏗️ System Architecture

The Tractor Control System acts as an **External Controller** for Autoware:
1. **Inputs:** Subscribes to `/planning/scenario_planning/trajectory` (for the reference path) and `/localization/kinematic_state` (for current position/velocity).
2. **Processing:** The NMPC Node isolates a localized prediction horizon from the trajectory, translates coordinates to the vehicle frame, and formulates an optimization problem using CasADi.
3. **Solving:** The IPOPT Interior Point optimizer iteratively solves for the optimal steering and acceleration commands within a hard 100ms real-time deadline.
4. **Outputs:** The optimal control horizon is extracted, and the first command is published to `/external/selected/control_cmd` for Autoware to execute.

## 🚜 Vehicle Model Analysis

Unlike standard kinematic bicycle models, this package uses a **5-DOF Nonlinear Tractor-Trailer Kinematic Model** to capture the complex hitch and trailer joint dynamics.

**State Variables (10):**
* `[x, y, yaw]` : Tractor absolute pose
* `[yaw_trail, yaw_bar]` : Trailer and Drawbar articulation angles
* `[v_x, v_y, r]` : Tractor velocities and yaw rate
* `[prev_steer, prev_accel]` : Memory states for physical continuity

**Dynamics (Backward Euler Integration):**
To ensure unconditional stability at high discretization steps (e.g., $dt=0.15s$), the system uses implicit **Backward Euler** integration instead of Runge-Kutta. The tire models are linearized ($F_y = C_\alpha \cdot \alpha$) to ensure the Jacobians remain non-singular even at $v=0$, preventing solver crashes during low-speed maneuvers.

## 🧠 MPC Algorithm

The controller utilizes a **Nonlinear Model Predictive Control (NMPC)** formulation.

**Objective Function:**
The cost function minimizes the sum of errors over the prediction horizon $N$:
$$ J = \sum_{k=1}^{N} (Q_y \cdot e_y^2 + Q_\theta \cdot e_\theta^2 + Q_v \cdot e_v^2) + \sum_{k=1}^{N} (R_{\Delta} \cdot \Delta u^2) $$

* **Rate Penalty Invariance:** Rate penalties are automatically scaled by $(0.1 / dt)^2$. This guarantees that tuning weights (like `steer_rate: 500.0`) behave identically across physical time regardless of what $dt$ horizon is configured.
* **Solver Tuning:** Solved via CasADi `nlpsol` using the `ipopt` backend. N is configured to $20$ with a $dt$ of $0.15s$ (yielding a $3.0s$ lookahead) to balance trajectory tracking precision with real-time computational limits ($\le 50ms$).
* **Death-Spiral Prevention:** If the optimization exceeds `max_iter`, the solver natively extracts the sub-optimal debug variables (`opti.debug.value()`). This prevents control-loop freezing and eliminates bang-bang "snaking" behavior when error matrices become highly nonlinear.
