# Tractor Control and Trajectory Verification Architecture

This package provides a high-fidelity control, trajectory tracking, and behavioral planning verification framework for autonomous tractor and multi-trailer articulated vehicles integrated within the Autoware autonomous driving stack. The software architecture incorporates Nonlinear Model Predictive Control (NMPC), linear error-space Model Predictive Control with metaheuristic parameter optimization, keyboard teleoperation safety overrides, and an automated behavioral path verification pipeline.

---

## 1. System Overview

Articulated multi-trailer systems present non-minimum phase characteristics, hitch constraints, and off-tracking phenomena that render standard single-rigid-body vehicle controllers inadequate. The `tractor_control` and verification architecture coordinates three operational tiers within the Autoware ecosystem:

1. **Behavioral Planning and Trajectory Verification**: Intercepts mission routing, validates candidate paths against road network geometry via orthogonal 2D projection envelopes, and certifies kinematic feasibility prior to trajectory execution.
2. **Trajectory Tracking Control (NMPC / Linear MPC)**: Ingests localized kinematic odometry (`/localization/kinematic_state`) and global or local reference paths (`/planning/scenario_planning/trajectory`), calculating constrained longitudinal and lateral actuation inputs.
3. **External Vehicle Actuation and Gateway Integration**: Injects high-frequency control signals directly into Autoware's `vehicle_cmd_gate` and `external_cmd_selector` via the modern Autoware External API.

```
+---------------------------------------------------------------------------------------------------+
|                                      Autoware Planning Stack                                      |
|                                                                                                   |
|  +---------------------------+       /planning/scenario_planning/      +-----------------------+  |
|  | Mission / Behavior Planner|  lane_driving/behavior_planning/path_.. | Path Verification Node|  |
|  | (Global Path Computation) |---------------------------------------->|  get_path_behavior_   |  |
|  +---------------------------+                                         |        path.py        |  |
|                |                                                       +-----------------------+  |
|                | /planning/scenario_planning/trajectory                                            |
|                v                                                                                  |
|  +---------------------------------------------------------------------------------------------+  |
|  |                                   Tractor Control Architecture                              |  |
|  |                                                                                             |  |
|  |  +-----------------------------+         +--------------------+     +--------------------+  |  |
|  |  |      NMPC Controller        |         |  Linearized MPC    |     |  Manual Teleop     |  |  |
|  |  |  (CasADi + IPOPT, 5-DOF)    |<------->|  (Kronecker / GWO) |<--->|  (Ackermann PID)   |  |  |
|  |  |   nmpc_control_node.py      |         | tractor_control.py |     | ackermann_teleop_..|  |  |
|  |  +-----------------------------+         +--------------------+     +--------------------+  |  |
|  +---------------------------------------------------------------------------------------------+  |
|                |                                                                                  |
|                | /external/selected/control_cmd (autoware_control_msgs/msg/Control)               |
|                v                                                                                  |
|  +---------------------------------------------------------------------------------------------+  |
|  |                               Autoware vehicle_cmd_gate Selector                            |  |
|  |                                                                                             |  |
|  |     Mode 1: STOP   |   Mode 2: AUTONOMOUS   |   Mode 3: LOCAL   |   Mode 4: REMOTE          |  |
|  +---------------------------------------------------------------------------------------------+  |
+---------------------------------------------------------------------------------------------------+
```

---

## 2. Theoretical Formulation and Kinematic Modeling

### 2.1 Multi-Body Articulated Kinematic Model

The system models an active front-steered tractor unit coupled to $M$ passive trailers via mechanical drawbars. For a configuration with $M$ trailers, the continuous state vector is defined in $\mathbb{R}^{3 + 2M + 1}$:

$$\mathbf{x} = \begin{bmatrix} x_0 & y_0 & \theta_0 & \theta_{d,1} & \theta_{t,1} & \dots & \theta_{d,M} & \theta_{t,M} & v \end{bmatrix}^T$$

where:
* $(x_0, y_0)$ denotes the Cartesian position of the tractor rear axle center in the global coordinate frame $\mathcal{F}_W$.
* $\theta_0$ represents the tractor heading orientation angle.
* $\theta_{d,i}$ and $\theta_{t,i}$ represent the orientation angles of the $i$-th drawbar and $i$-th trailer body, respectively.
* $v$ denotes the longitudinal forward velocity of the tractor.

The control input vector $\mathbf{u} \in \mathbb{R}^2$ comprises:

$$\mathbf{u} = \begin{bmatrix} \delta & a \end{bmatrix}^T$$

where $\delta$ is the front-wheel steering angle and $a$ is the longitudinal acceleration.

The non-holonomic kinematic differential equations governing the tractor are:

$$\dot{x}_0 = v \cos(\theta_0)$$

$$\dot{y}_0 = v \sin(\theta_0)$$

$$\dot{\theta}_0 = \frac{v}{L_0} \tan(\delta)$$

$$\dot{v} = a$$

where $L_0$ is the tractor wheelbase. For each successive articulated assembly $i \in \{1, \dots, M\}$ connected with hitch offset $d_{h, i-1}$, drawbar length $L_{\text{bar}, i}$, and trailer length $L_{\text{trl}, i}$, the angular velocities propagate according to:

$$\Delta \theta_{a,i} = \theta_{t, i-1} - \theta_{d,i}$$

$$\dot{\theta}_{d,i} = \frac{v_{i-1}}{L_{\text{bar}, i}} \sin(\Delta \theta_{a,i}) - \frac{d_{h, i-1}}{L_{\text{bar}, i}} \dot{\theta}_{t, i-1} \cos(\Delta \theta_{a,i})$$

$$\Delta \theta_{b,i} = \theta_{d,i} - \theta_{t,i}$$

$$\dot{\theta}_{t,i} = \frac{v_{d,i}}{L_{\text{trl}, i}} \sin(\Delta \theta_{b,i})$$

### 2.2 Numerical Discretization and Stability

To maintain unconditional numerical stability over extended prediction horizons without the stiffness degradation characteristic of explicit Runge-Kutta integration, the discrete kinematic state transitions are evaluated via implicit Backward Euler discretization:

$$\mathbf{x}_{k+1} = \mathbf{x}_k + \mathbf{f}(\mathbf{x}_{k+1}, \mathbf{u}_k) \Delta t$$

Tire lateral force characteristics are linearized within the low slip-angle regime ($F_y = C_\alpha \alpha$), ensuring continuous Jacobian evaluation and preventing singular matrix inversion during zero-velocity standstill conditions.

---

## 3. Nonlinear Model Predictive Control (NMPC)

The trajectory tracking controller implemented in [`nmpc_control_node.py`](file:///home/tacv/AutoTract/tractor_ws/src/control/tractor_control/tractor_control/nmpc_control_node.py) optimizes actuation profiles over a finite prediction horizon $N$ with discrete time step $\Delta t_{\text{pred}}$.

### 3.1 Cost Function Formulation

The constrained finite-horizon Optimal Control Problem (OCP) minimizes the tracking errors relative to the reference trajectory:

$$\min_{\mathbf{U}} J = \sum_{k=0}^{N-1} \left( \|\mathbf{x}_k - \mathbf{x}_{\text{ref}, k}\|_{\mathbf{Q}}^2 + \|\mathbf{u}_k\|_{\mathbf{R}}^2 + \|\Delta \mathbf{u}_k\|_{\mathbf{R}_{\Delta}}^2 \right) + \|\mathbf{x}_N - \mathbf{x}_{\text{ref}, N}\|_{\mathbf{Q}_N}^2$$

where:
* $\mathbf{Q} = \mathrm{diag}(q_{e_y}, q_{e_y}, q_{e_\theta}, q_{e_{\theta, d}}, q_{e_{\theta, t}}, \dots, q_{e_v})$ weights tracking deviations in Cartesian position, tractor yaw, articulated joint angles, and longitudinal speed.
* $\mathbf{R} = \mathrm{diag}(r_\delta, r_a)$ penalizes actuation control magnitudes.
* $\mathbf{R}_{\Delta} = \mathrm{diag}(r_{\Delta \delta}, r_{\Delta a})$ suppresses aggressive actuation rates.

### 3.2 Actuation Rate Penalty Invariance

To eliminate sensitivity to the selection of the prediction step size $\Delta t_{\text{pred}}$, rate penalties are normalized using the quadratic scaling factor:

$$\mathbf{R}_{\Delta, \text{eff}} = \mathbf{R}_{\Delta} \cdot \left(\frac{0.10}{\Delta t_{\text{pred}}}\right)^2$$

This guarantees identical physical damping characteristics across differing discretization configurations.

### 3.3 Kinematic Constraints and Numerical Solver

The optimization problem is constrained by physical actuator limits:

$$\delta_{\min} \le \delta_k \le \delta_{\max}, \quad \forall k \in [0, N-1]$$

$$a_{\min} \le a_k \le a_{\max}, \quad \forall k \in [0, N-1]$$

$$\mathbf{x}_0 = \mathbf{x}_{\text{measured}}$$

$$\mathbf{x}_{k+1} = \mathbf{x}_k + \mathbf{f}(\mathbf{x}_k, \mathbf{u}_k) \Delta t_{\text{pred}}$$

The nonlinear programming (NLP) problem is transcribed using the CasADi framework and solved using the Interior Point Optimizer (IPOPT) with MUMPS linear solver backend. In scenarios where IPOPT exceeds the real-time execution budget (`ipopt.max_iter`), the controller retrieves the sub-optimal primal variables from the solver scratchpad via `opti.debug.value()`, preventing controller timeout and mitigating trajectory divergence.

---

## 4. Node Architecture and Behavioral Planning Coordination

```
+----------------------------------------------------------------------------------------------------+
|                      Path Verification Architecture: get_path_behavior_path.py                     |
|                                                                                                    |
|  +--------------------------+                                      +----------------------------+  |
|  |       path.csv           |                                      |      GoalPoints.csv        |  |
|  |  (Topological Edges:     |                                      |   (Cartesian Anchors:      |  |
|  |   FromNode -> ToNode)    |                                      |    x, y, z, qx, qy, qz, qw)|  |
|  +--------------------------+                                      +----------------------------+  |
|               |                                                                   |                |
|               +----------------------------------+--------------------------------+                |
|                                                  v                                                 |
|                        +---------------------------------------------------+                       |
|                        |          Operational Mode Determination           |                       |
|                        +---------------------------------------------------+                       |
|                                     /                         \                                    |
|                                    /                           \                                   |
|                                   v                             v                                  |
|   +----------------------------------------------+   +------------------------------------------+  |
|   |          Mode A: Sequential Execution        |   |      Mode B: On-Demand Verification      |  |
|   |        (Autonomous Batch Traversal)          |   |       (ROS 2 Service Server Mode)        |  |
|   +----------------------------------------------+   +------------------------------------------+  |
|   | 1. Read edge index (k)                       |   | 1. Intercept /verify_specific_route      |  |
|   | 2. Publish initialpose (station[from])       |   | 2. Dispatch start pose and goal pose     |  |
|   | 3. Publish goal (station[to])                |   | 3. Intercept PathWithLaneId trajectory   |  |
|   | 4. Await Autoware behavior path generation   |   | 4. Execute 2D orthogonal projection test |  |
|   | 5. Execute 2D planar projection test         |   | 5. Return success flag, start deviation, |  |
|   | 6. Serialize CSV and PNG plot to disk        |   |    goal deviation, and waypoint metrics  |  |
|   | 7. Advance edge index (k + 1)                |   +------------------------------------------+  |
|   +----------------------------------------------+                                                 |
|                                                  |                                                 |
|                                                  v                                                 |
|                   +--------------------------------------------------------------+                 |
|                   |           Orthogonal 2D Planar Verification Engine           |                 |
|                   |                                                              |                 |
|                   |  * Start Envelope Check:       d_proj(Start, Path) <= 0.45 m |                 |
|                   |  * Goal Envelope Check:        d_proj(Goal, Path)  <= 1.00 m |                 |
|                   |  * Endpoint Fallback:          d(Start, P0)        <= 0.45 m |                 |
|                   |                                d(Goal, Plast)      <= 1.00 m |                 |
|                   +--------------------------------------------------------------+                 |
+----------------------------------------------------------------------------------------------------+
```

### 4.1 Behavioral Verification Node: `get_path_behavior_path.py`

The [`get_path_behavior_path.py`](file:///home/tacv/AutoTract/tractor_ws/src/testing/towtruck_testing/towtruck_testing/get_path_behavior_path.py) node, structured under [`PathWithLaneIDSubscriber`](file:///home/tacv/AutoTract/tractor_ws/src/testing/towtruck_testing/towtruck_testing/get_path_behavior_path.py#L37-L720), provides closed-loop verification between high-level topological routes and Autoware's low-level behavior planner. It operates in two primary operational paradigms:

#### Mode A: Sequential Route Execution Mode
1. **Topological Ingestion**: Loads route edge sequences from [`path.csv`](file:///home/tacv/AutoTract/tractor_ws/src/testing/towtruck_testing/resource/paths.csv) (`from_node` and `to_node`) and resolves station coordinate vectors via [`GoalPoints.csv`](file:///home/tacv/AutoTract/tractor_ws/src/testing/towtruck_testing/resource/GoalPoints.csv).
2. **State Machine Execution**: Governed by the internal `RouteState` enumerator:
   * `INITIALIZING`: Resolves coordinates and dispatches the initial covariance pose to `initialpose`.
   * `WAITING_FOR_ROUTE`: Dispatches navigation goal to `/planning/mission_planning/goal` and activates receipt timers.
   * `ROUTE_PLANNING`: Intercepts the generated `PathWithLaneId` message from Autoware and executes geometric envelope verification.
   * `READY_TO_RECORD`: Renders Cartesian coordinate plots with boundary lines and writes path data to CSV files.
   * `FINALIZING`: Validates non-zero file writes and initiates a 2.0-second simulator settling period before advancing to the subsequent edge.
   * `FAILURE`: Flags unreachable paths, logs terminal diagnostics, and triggers skip-logic if validation retries exceed 50 iterations.

#### Mode B: On-Demand Service Verification Mode
Provides an interactive verification service interface (`/verify_specific_route`) allowing higher-level dispatchers, fleet managers, or simulation regression scripts to validate isolated route feasibility on demand:
* The client passes an origin and destination node identifier (or explicit Cartesian poses).
* The node commands Autoware's behavior planner, intercepts the resulting trajectory, executes the 2D projection validation, and returns deviation telemetry synchronously.

### 4.2 Trajectory Tracking Nodes

* [`nmpc_control_node.py`](file:///home/tacv/AutoTract/tractor_ws/src/control/tractor_control/tractor_control/nmpc_control_node.py) ([`NMPCControlNode`](file:///home/tacv/AutoTract/tractor_ws/src/control/tractor_control/tractor_control/nmpc_control_node.py#L115-L292)): Multi-trailer nonlinear model predictive controller using CasADi/IPOPT. Generates control commands and predicted paths.
* [`tracter_control.py`](file:///home/tacv/AutoTract/tractor_ws/src/control/tractor_control/tractor_control/tracter_control.py) ([`TracterControlNode`](file:///home/tacv/AutoTract/tractor_ws/src/control/tractor_control/tractor_control/tracter_control.py#L142-L280)): Linear error-space MPC controller utilizing Kronecker product block matrices for fast quadratic programming (QP).
* [`gwo_tuner_node.py`](file:///home/tacv/AutoTract/tractor_ws/src/control/tractor_control/simulation/gwo_tuner_node.py): Real-time parameter optimization node executing the Grey Wolf Optimizer metaheuristic to dynamically tune the diagonal weights ($Q_y, Q_\theta, Q_v, R_\delta$) based on integral tracking error metrics.
* [`ackermann_teleop_node.py`](file:///home/tacv/AutoTract/tractor_ws/src/control/tractor_control/tractor_control/ackermann_teleop_node.py): Low-latency keyboard teleoperation node with closed-loop PID longitudinal velocity control and heartbeat management for emergency operator takeover.

---

## 5. Verification Geometry and Projection Algorithms

### 5.1 2D Planar Point-to-Line-Segment Orthogonal Projection

Trajectory verification mandates establishing whether the path generated by Autoware connects the commanded origin and destination poses within strict spatial bounds. Rather than evaluating distance strictly against discrete path endpoints (which are subject to variable waypoint spacing), the algorithm calculates the orthogonal projection distance onto each constituent linear segment of the trajectory in $\mathbb{R}^2$.

```
                 P (Target Pose: Start or Goal)
                 |
                 |  Orthogonal Distance: d_proj
                 v
   A o-----------x---------------------------o B
   (p_i)         P_proj                      (p_{i+1})
   
   Segment Vector: v = B - A
   Vector AP:      w = P - A
   Scalar factor:  t = (w . v) / ||v||^2
   Constrained:    t* = clamp(t, 0.0, 1.0)
   Projection:     P_proj = A + t* * v
```

Let the discrete path $\Gamma$ comprise $M$ sequential waypoints in the horizontal plane:

$$\Gamma = \left\{ \mathbf{p}_1, \mathbf{p}_2, \dots, \mathbf{p}_M \right\}, \quad \mathbf{p}_i = \begin{bmatrix} x_i & y_i \end{bmatrix}^T \in \mathbb{R}^2$$

For each line segment $\mathcal{S}_i = [\mathbf{p}_i, \mathbf{p}_{i+1}]$, define the segment displacement vector $\mathbf{v}_i$:

$$\mathbf{v}_i = \mathbf{p}_{i+1} - \mathbf{p}_i = \begin{bmatrix} x_{i+1} - x_i \\ y_{i+1} - y_i \end{bmatrix}$$

Given a query boundary point $\mathbf{p}_q \in \{ \mathbf{p}_{\text{start}}, \mathbf{p}_{\text{goal}} \}$, define the relative displacement vector:

$$\mathbf{w}_i = \mathbf{p}_q - \mathbf{p}_i = \begin{bmatrix} x_q - x_i \\ y_q - y_i \end{bmatrix}$$

The unconstrained orthogonal projection scalar $t_i$ is obtained by projecting $\mathbf{w}_i$ onto $\mathbf{v}_i$:

$$t_i = \frac{\mathbf{w}_i \cdot \mathbf{v}_i}{\|\mathbf{v}_i\|_2^2} = \frac{(x_q - x_i)(x_{i+1} - x_i) + (y_q - y_i)(y_{i+1} - y_i)}{(x_{i+1} - x_i)^2 + (y_{i+1} - y_i)^2}$$

To constrain the projection to the finite segment bounds, $t_i$ is clamped:

$$t_i^* = \max\left(0.0, \min(1.0, t_i)\right)$$

The coordinates of the closest point $\mathbf{p}_{\text{proj}, i}$ on segment $\mathcal{S}_i$ are:

$$\mathbf{p}_{\text{proj}, i} = \mathbf{p}_i + t_i^* \mathbf{v}_i = \begin{bmatrix} x_i + t_i^*(x_{i+1} - x_i) \\ y_i + t_i^*(y_{i+1} - y_i) \end{bmatrix}$$

The Euclidean projection deviation $d(\mathbf{p}_q, \mathcal{S}_i)$ is computed as:

$$d(\mathbf{p}_q, \mathcal{S}_i) = \|\mathbf{p}_q - \mathbf{p}_{\text{proj}, i}\|_2 = \sqrt{(x_q - x_{\text{proj}, i})^2 + (y_q - y_{\text{proj}, i})^2}$$

The minimum distance across the entire trajectory path $\Gamma$ is:

$$d_{\min}(\mathbf{p}_q, \Gamma) = \min_{i \in \{1, \dots, M-1\}} d(\mathbf{p}_q, \mathcal{S}_i)$$

### 5.2 Planar 2D Projection Rationale

All geometric verification is evaluated strictly in the 2D Cartesian plane ($X, Y$), ignoring vertical elevation ($Z$). This decoupling prevents false-negative route rejections arising from:
1. Variations in sensor kit baseline elevations across differing tractor chassis.
2. Pitch and roll perturbations during elevation transitions on industrial ramps.
3. Discretization discrepancies between vector map road surface meshes and localized odometry.

### 5.3 Asymmetric Acceptance Envelopes

The verification engine enforces asymmetric tolerance gates:

$$\text{Verification Criterion} = \begin{cases} \text{Passed}, & \text{if } d_{\min}(\mathbf{p}_{\text{start}}, \Gamma) \le \epsilon_{\text{start}} \quad \text{AND} \quad d_{\min}(\mathbf{p}_{\text{goal}}, \Gamma) \le \epsilon_{\text{goal}} \\ \text{Failed}, & \text{otherwise} \end{cases}$$

* **Start Acceptance Envelope ($\epsilon_{\text{start}} = 0.45\,\text{m}$)**: Imposes tight tolerances on trajectory inception to ensure exact alignment with the localized vehicle position and prevent lateral path jumping upon autonomous engagement.
* **Goal Acceptance Envelope ($\epsilon_{\text{goal}} = 1.00\,\text{m}$)**: A broader longitudinal envelope accommodates Autoware's behavior velocity planner braking logic, deceleration buffer regions, and terminal waypoint pruning before physical collision surfaces or stop lines.

### 5.4 Boundary Endpoint Fallback Criterion

In cases where segment interpolation does not encompass terminal points due to sparse point density at path extremes, direct endpoint distance is evaluated:

$$d(\mathbf{p}_{\text{start}}, \mathbf{p}_1) \le \epsilon_{\text{start}}, \quad d(\mathbf{p}_{\text{goal}}, \mathbf{p}_M) \le \epsilon_{\text{goal}}$$

---

## 6. System Interfaces

### 6.1 Subscribed Topics

| Topic | Message Type | Consuming Node | Description |
| :--- | :--- | :--- | :--- |
| `/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id` | `autoware_internal_planning_msgs/msg/PathWithLaneId` | `get_path_behavior_path` | Trajectory containing centerline poses, left/right lane boundaries, and lanelet associations. |
| `/planning/scenario_planning/lane_driving/behavior_planning/path` | `autoware_planning_msgs/msg/Path` | `get_path_behavior_path` | Raw behavioral planner path representation. |
| `/planning/scenario_planning/trajectory` | `autoware_planning_msgs/msg/Trajectory` | `nmpc_control_node`, `tracter_control` | Optimized reference trajectory including velocity profiles and target headings. |
| `/localization/kinematic_state` | `nav_msgs/msg/Odometry` | `nmpc_control_node`, `tracter_control` | High-frequency filtered odometry state ($x, y, z$, orientation quaternion, and linear/angular velocity). |
| `/control/vehicle_cmd_gate/operation_mode` | `autoware_adapi_v1_msgs/msg/OperationModeState` | System Monitor | Autoware operational control mode status. |
| `/vehicle/status/velocity_status` | `autoware_auto_vehicle_msgs/msg/VelocityReport` | `ackermann_teleop_node` | Vehicle longitudinal velocity feedback for teleoperation PID loop. |

### 6.2 Published Topics

| Topic | Message Type | Sourced Node | Description |
| :--- | :--- | :--- | :--- |
| `initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | `get_path_behavior_path` | Seeds localized vehicle origin in the global map frame. |
| `/planning/mission_planning/goal` | `geometry_msgs/msg/PoseStamped` | `get_path_behavior_path` | Sets destination goal poses to trigger Autoware route planning. |
| `/external/selected/control_cmd` | `autoware_control_msgs/msg/Control` | `nmpc_control_node`, `tracter_control` | External control commands forwarded to Autoware vehicle gateway (steering angle, velocity, acceleration). |
| `/external/selected/gear_cmd` | `autoware_vehicle_msgs/msg/GearCommand` | `nmpc_control_node`, `ackermann_teleop_node` | Commanded transmission state (e.g., `GearCommand::DRIVE`). |
| `/api/external/set/command/remote/heartbeat` | `tier4_external_api_msgs/msg/Heartbeat` | `nmpc_control_node`, `ackermann_teleop_node` | Safety watchdog signal required to retain remote control authorization. |
| `/mpc/control_cmd` | `autoware_control_msgs/msg/Control` | `nmpc_control_node`, `tracter_control` | Local control output stream for standalone debugging. |
| `/mpc/predicted_trajectory` | `nav_msgs/msg/Path` | `nmpc_control_node` | CasADi/IPOPT optimal state prediction horizon for RViz visualization. |
| `/mpc/error_status` | `std_msgs/msg/Float64MultiArray` | `tracter_control` | Tracking error metrics: $[e_y, e_\theta, e_v]$. |

### 6.3 Service Interfaces

#### Route Verification Service: `/verify_specific_route`

The on-demand service allows explicit verification of arbitrary route segments between topological nodes:

```text
# Service Definition: towtruck_interfaces/srv/VerifyRoute (or internal verification service)
# Request
string start_node                  # Origin station identifier matching GoalPoints.csv (e.g., "H01N")
string goal_node                   # Destination station identifier matching GoalPoints.csv (e.g., "H02N")
bool record_path                   # Serialization flag: writes CSV and PNG artifacts when true
---
# Response
bool success                       # True if trajectory meets both start and goal tolerance envelopes
float64 start_lateral_deviation    # Minimum orthogonal distance from start point to path (m)
float64 goal_distance_deviation    # Minimum distance from goal point to path terminus (m)
uint32 waypoint_count              # Total number of discrete waypoints in validated path
string status_message              # Detailed telemetry status or error diagnostic string
```

#### Autoware Control Mode Transition Services

* `/api/operation_mode/change_to_remote` (`autoware_adapi_v1_msgs/srv/ChangeOperationMode`): Transfers control authority from internal planners to the external interface.
* `/api/operation_mode/change_to_autonomous` (`autoware_adapi_v1_msgs/srv/ChangeOperationMode`): Restores internal Autoware control authority.
* `/api/autoware/set/engage` (`tier4_external_api_msgs/srv/Engage`): Controls vehicle motion engagement (`engage: true` / `engage: false`).

---

## 7. Package and File Structure

```text
tractor_control/
├── config/
│   ├── gwo_config.yaml              # Hyperparameters for Grey Wolf Optimizer
│   ├── mpc_best_params.yaml         # Optimal offline weights converged via GWO
│   └── mpc_config.yaml              # Core vehicle kinematics and NMPC weights
├── launch/
│   └── tracter_control.launch.py    # Primary launch orchestration script
├── paths/
│   └── figure8.csv                  # Standard benchmarking trajectory
├── resource/
│   └── tractor_control              # Package index resource identifier
├── simulation/
│   ├── __init__.py
│   ├── generate_path.py             # Synthetic path generation utilities
│   ├── gwo_tuner_node.py            # Dynamic metaheuristic parameter optimizer
│   ├── mpc_node.py                  # Standalone uncoupled vehicle MPC
│   └── path_publisher.py            # Static CSV waypoint publisher
├── tractor_control/
│   ├── __init__.py
│   ├── ackermann_teleop_node.py     # Keyboard teleoperation with Autoware External API
│   ├── control_selector_node.py     # Legacy multiplexer for command arbitration
│   ├── mpc_dynamic_control_node.py  # Adaptive parameter-scheduling MPC
│   ├── mpc_logger.py                # Telemetry logging and performance metrics
│   ├── nmpc_control_node.py         # 5-DOF multi-trailer CasADi/IPOPT controller
│   ├── tracter_control.py           # Autoware Universe trajectory tracking node
│   ├── vehicle_chassis.py           # Physical dimensions and constraint models
│   └── vehicle_node.py              # Software-in-the-loop kinematic vehicle simulator
├── urdf/
│   └── tractor.urdf                 # Kinematic link and joint chain definitions
├── package.xml                      # ROS 2 package dependencies and metadata
├── setup.cfg                        # Script installation directory layout
└── setup.py                         # Entry point definitions and build configuration
```

Related path verification source files reside within the testing package:
* [`get_path_behavior_path.py`](file:///home/tacv/AutoTract/tractor_ws/src/testing/towtruck_testing/towtruck_testing/get_path_behavior_path.py): Trajectory interception and geometric envelope verification.
* [`GoalPoints.csv`](file:///home/tacv/AutoTract/tractor_ws/src/testing/towtruck_testing/resource/GoalPoints.csv): Station coordinate lookup table.
* [`paths.csv`](file:///home/tacv/AutoTract/tractor_ws/src/testing/towtruck_testing/resource/paths.csv): Topological edge adjacency definitions.

---

## 8. Execution, Verification, and Debugging Guide

### 8.1 Workspace Build Instructions

Execute a targeted clean build within the workspace root:

```bash
cd ~/AutoTract
./clean_build.sh
source install/setup.bash
```

Or build specific packages via `colcon`:

```bash
cd ~/AutoTract/tractor_ws
colcon build --symlink-install --packages-select tractor_control towtruck_testing
source install/setup.bash
```

### 8.2 Launching the Autoware Simulation Environment

Initialize the Autoware Planning Simulator with the tow tractor vehicle model and vector map:

```bash
source ~/AutoTract/install/setup.bash
ros2 launch tractor_launch planning_simulator.launch.xml \
  map_path:=/home/tacv/autoware_map/minebae \
  vehicle_model:=tractor \
  sensor_model:=tractor_sensor_kit
```

### 8.3 Executing Path Verification

#### Automated Sequential Batch Mode
To process all topological edges defined in `paths.csv`:

```bash
source ~/AutoTract/install/setup.bash
ros2 run towtruck_testing get_path_behavior_path
```

#### On-Demand Service Call Mode (Isolated Debugging)
To verify a single candidate route edge between defined stations (e.g., origin `H01N` to destination `H02N`) without batch processing:

```bash
ros2 service call /verify_specific_route towtruck_interfaces/srv/VerifyRoute \
  "{start_node: 'H01N', goal_node: 'H02N', record_path: true}"
```

Expected terminal response on verification success:

```text
requester: making request: towtruck_interfaces.srv.VerifyRoute_Request(start_node='H01N', goal_node='H02N', record_path=True)

response:
towtruck_interfaces.srv.VerifyRoute_Response(
  success=True,
  start_lateral_deviation=0.084,
  goal_distance_deviation=0.312,
  waypoint_count=142,
  status_message='Path successfully verified within tolerance envelopes.'
)
```

### 8.4 Operating Trajectory Tracking Controllers

#### Nonlinear MPC (Articulated Multi-Trailer Mode)
Start the 5-DOF CasADi/IPOPT tracking controller:

```bash
ros2 run tractor_control nmpc_control_node
```

#### Activating Remote Control Authority in Autoware
1. Transition `vehicle_cmd_gate` to Remote Mode:
   ```bash
   ros2 service call /api/operation_mode/change_to_remote autoware_adapi_v1_msgs/srv/ChangeOperationMode
   ```
2. Engage the autonomous drive system:
   ```bash
   ros2 service call /api/autoware/set/engage tier4_external_api_msgs/srv/Engage "{engage: true}"
   ```
3. Confirm active operation mode:
   ```bash
   ros2 topic echo /control/vehicle_cmd_gate/operation_mode
   ```
   A reported value of `mode: 4` confirms external command authority (`REMOTE`).

4. Revert to internal Autoware planning control:
   ```bash
   ros2 service call /api/operation_mode/change_to_autonomous autoware_adapi_v1_msgs/srv/ChangeOperationMode
   ```

### 8.5 Keyboard Teleoperation Safety Override

To execute manual override maneuvers or test low-level actuation interfaces:

```bash
ros2 run tractor_control ackermann_teleop_node
```

* `Spacebar`: Toggles system enable state (automatically commands `change_to_remote` and engages drive mode).
* `W` / `X`: Adjusts target velocity (increments forward throttle / applies braking).
* `A` / `D`: Adjusts front tire steering angle left / right.
* `S`: Commands full emergency stop (zero velocity, $-3.0\,\text{m/s}^2$ deceleration).
* `Z`: Centers front wheel steering angle to $0.0\,\text{rad}$.
* `Ctrl+C`: Safe shutdown sequence (transmits zero-velocity full-brake command and disengages remote authority).

### 8.6 Telemetry and Diagnostics Verification

Monitor command frequencies and tracking performance during testing:

```bash
# Verify external command publication rate (~10 Hz)
ros2 topic hz /external/selected/control_cmd

# Inspect real-time NMPC tracking error output
ros2 topic echo /mpc/error_status

# Verify behavior planner path reception
ros2 topic hz /planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id
```
