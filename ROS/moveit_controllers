# Notes on Controllers in MoveIt / ros2_control

Reference notes on how MoveIt connects to robot hardware, what the controller layer actually does, and the practical implications for choosing between control modes.

## The stack, top to bottom

When you "send a goal to MoveIt," the chain is:

1. **MoveIt2** plans a path (OMPL, STOMP, Pilz, etc.) and produces a `JointTrajectory` — a list of waypoints with positions, velocities, accelerations, and timestamps. Time parameterization (typically `TimeOptimalTrajectoryGeneration` / TOTG) is what adds the velocity/acceleration/time information to a path of positions.
2. **MoveIt's controller manager** (`MoveItSimpleControllerManager` or `MoveItRosControlInterface`) is the bridge from MoveIt to whatever executes the trajectory. It forwards the trajectory via a `FollowJointTrajectory` action.
3. **`ros2_control` controller** (e.g. `joint_trajectory_controller`) receives the trajectory, interpolates between waypoints in its `update()` loop at 100–1000 Hz, and writes setpoints to the hardware interface.
4. **Hardware interface plugin** is the vendor-implemented piece. Exposes *command interfaces* (what you can write — position, velocity, effort) and *state interfaces* (what you can read — position, velocity, effort, F/T, etc.).
5. **Vendor driver** speaks the robot's native protocol (RTDE for UR, FCI for Franka) and runs the actual real-time loop.

## MoveItSimpleControllerManager vs ros2_control

These sit at different layers. The real comparison is "MoveIt talking to a bare custom `FollowJointTrajectory` action server" vs "MoveIt talking to a `joint_trajectory_controller` running inside `ros2_control`."

What `ros2_control` adds:

- **The whole controller ecosystem.** Once you have a hardware interface, every controller compatible with what it exposes works without integration work. `joint_trajectory_controller`, `forward_position_controller`, `forward_velocity_controller`, `joint_state_broadcaster`, `force_torque_sensor_broadcaster`, `admittance_controller`, etc.
- **Hot-swapping controllers at runtime.** `ros2 control switch_controllers --deactivate X --activate Y` flips control modes in milliseconds. Critical for tasks that need different modes in different phases (plan-and-execute for approach, servoing for visual alignment, compliance for contact).
- **Resource arbitration.** Framework enforces that two controllers can't write the same command interface simultaneously.
- **Real-time loop guarantees.** `controller_manager` runs `update()` deterministically at a fixed rate in a real-time-safe thread.
- **Standard state broadcasters.** `/joint_states` and other conventions just work — the whole ROS2 ecosystem (RViz, MoveIt, tf2) expects them.
- **Simulation parity.** `use_mock_hardware:=true` swaps the real plugin for `mock_components/GenericSystem`, a stub that stores commands and reports them back as state. Same controller configs work in mock, Gazebo, and on hardware. Mock doesn't simulate dynamics — it's a pipe test. For dynamics, use Gazebo/Ignition with `gz_ros2_control` (also a hardware interface plugin).

`MoveItSimpleControllerManager` is fine if the only thing you ever want is "MoveIt plans, robot executes." The moment you want anything reactive (servoing, teleop, mode switching), you want `ros2_control` underneath.

## "Controllers for free" — what's actually free

A controller can only command what the hardware interface exposes. If the vendor only implements a position command interface, you get position-based controllers and nothing else. Velocity/effort controllers will fail to claim resources at startup.

What's free regardless of what the hardware exposes:

- State broadcasters (they read, don't command, so always work).
- The controller manager infrastructure itself.
- The simulation pipeline.

What's free if the corresponding command interface exists:

- Every controller in the `ros2_controllers` package compatible with that interface.
- Community controllers built on the framework.

## The command modes

| Mode | What you send | What it's good for |
|------|---------------|---------------------|
| Position | target angles | Trajectory execution, servoing |
| Velocity | target rad/s | Smooth servoing, jogging, teleop |
| Effort | joint torques (Nm) | Compliance, impedance, learned control |
| Force mode | desired wrench | Contact tasks (vendor-specific) |

### `forward_position_controller`

Writes to position command interface. Takes a single setpoint (`Float64MultiArray` on `/forward_position_controller/commands`), forwards it directly to hardware every cycle. No interpolation, no buffering, no trajectory concept. If you stop publishing, the last commanded position holds.

Contrast with `joint_trajectory_controller`: trajectory controller takes a *trajectory* (sequence of waypoints with timestamps) and interpolates. Forward controller takes a *setpoint*.

Cost: no safety net. The user is responsible for sending achievable commands — no velocity/acceleration smoothing, no limit enforcement at the controller level. Publish a setpoint a meter away from current and the robot tries to get there immediately.

### `forward_velocity_controller`

Writes to velocity command interface. Send target joint velocities, hardware tries to hold them. Stop publishing → driver watchdog zeros velocities for safety.

### `forward_effort_controller`

Writes to effort command interface — direct motor torque (Nm), bypassing internal position and velocity loops. Much harder to use safely: gravity pulls the arm down at zero torque, so you need at least gravity compensation, usually a full dynamic model. The only way to do true compliance, impedance, or model-based control. Franka exposes this cleanly at 1 kHz with the full robot model as state, which is why most compliance/impedance research uses Panda or FR3.

### `force_mode_controller` (UR-specific)

Structurally different from the three above. Not a low-level command interface — a wrapper around UR's *firmware-level* force mode. You specify "apply this wrench in this direction in this frame, stay compliant in these other directions" and UR's firmware handles implementation (hybrid position/force control with their own dynamic model). Much easier to use for polishing/insertion/wiping than building your own torque controller.

The general distinction: `force_mode_controller` is high-level (you specify behavior, firmware figures out torques). `forward_effort_controller` is low-level (you specify torques yourself).

## Why moveit_servo uses forward_position_controller

Servoing is online, reactive control — compute next-cycle joint command from sensor feedback at high rate (100–500 Hz), no trajectory plan to execute.

Pipeline:

```
Cartesian twist (joystick/camera/policy)
  → moveit_servo computes IK for next cycle
  → publishes joint position to forward_position_controller
  → controller writes to hardware
```

Trying to do this through `joint_trajectory_controller` means sending a new one-point "trajectory" every 10ms and preempting the previous one. Result: jerky motion, action server thrashing, latency from action-server roundtripping.

`forward_position_controller` is the shortest path from "computed desired position" to "wire writes to motor." That's why it's used for servoing.

## Position vs velocity on a UR5 — important subtlety

UR's firmware-level controller is fundamentally a position servo at ~500 Hz. When you "command velocity" through the UR ROS2 driver, you're NOT bypassing the position loop. The driver integrates your velocity commands into position setpoints, which go to the robot's position controller.

So on UR, position vs velocity mode affects:

- **Interaction model**: velocity is naturally suited to "continuous stream of commands at fixed rate." Position trajectory is suited to "here's the plan, execute it."
- **Failure modes**: velocity command loss → watchdog zeros velocities → robot decelerates. Trajectory completion → robot holds position.
- **Smoothness**: not because velocity is inherently smoother, but because well-conditioned velocity commands (from Ruckig or similar) feed directly without the discretization that happens when a position controller chops a trajectory into per-cycle setpoints.

On Franka, by contrast, the firmware actually runs in different modes (position, velocity, torque) at 1 kHz with hard real-time guarantees. The mode you pick changes what the firmware loop actually does.

## Does velocity-mode trajectory execution follow the correct path?

Yes, on a properly configured driver. Errors don't compound because position is tracked, not integrated open-loop.

Two ways drivers handle this:

1. **Closed-loop position correction in the controller.** `joint_trajectory_controller` in velocity mode writes `velocity_command = velocity_feedforward + Kp * (position_desired - position_actual)`. Position is the reference; velocity is feedforward.
2. **Firmware-level position tracking.** UR's firmware does position control regardless. Velocity commands become a way to specify *how fast* the position setpoint advances.

When it goes wrong: configuring `joint_trajectory_controller` in velocity mode *without* PID gains on position error → pure feedforward → errors compound. Common misconfiguration that leads people to falsely conclude "velocity control is unstable."

Where velocity mode actually shines: velocity continuity at trajectory boundaries and transitions. Position-mode controllers can produce small velocity discontinuities. The "smoother motion" benefit is real but usually modest on robots with stiff position loops, more noticeable on robots with softer loops or significant inertia.

Related concern: the trajectory's velocity field must have correct values. TOTG produces consistent (position, velocity, acceleration) tuples respecting joint limits. If velocities are junk (e.g. only positions specified, controller left to differentiate), velocity-mode execution is *worse* than position mode.

Velocity mode unambiguously wins when the trajectory source is itself velocity-based (velocity-output planner, reactive controller computing velocities directly). Then you avoid integrating to positions just to have the controller differentiate back.

## Who computes velocity/effort when people say "we use it for smoother motions"

Common confusion. Four cases:

1. **Position-based trajectory execution.** MoveIt plans positions, TOTG adds velocities/accelerations. `joint_trajectory_controller` interpolates with splines, writes position setpoints. You compute nothing — smoothness from time parameterization + spline interpolation.
2. **Velocity-based trajectory execution.** Same MoveIt plan. Controller writes velocities (from trajectory's velocity field + position-error correction). You still compute nothing at the plan level.
3. **Effort-based trajectory execution.** Same MoveIt plan. Controller converts (position_desired, velocity_desired) to torque via PID + gravity compensation. You're not computing efforts at the plan level, but the controller is doing real work at execution. Where Franka starts to differentiate.
4. **True torque/impedance control.** Custom controller computes torques from a dynamic model — gravity compensation, inertial decoupling, impedance terms. Trajectory is just a setpoint stream. Here you ARE computing efforts directly. Contact-rich manipulation regime.

"Trajectory from OMPL, MoveIt handles the translation" is right for cases 1–3. Case 4 is the exception — research labs and advanced manipulation work.

## FollowJointTrajectory does NOT give you servoing/teleop

`FollowJointTrajectory` is goal-based: client sends complete trajectory, server executes, reports success/failure. Wrong shape for "right now's desired joint position from this cycle's joystick/camera/policy output at 200 Hz."

Why abusing it for streaming fails:

- Action server overhead (goal acceptance, status updates, result publication) heavyweight at 200 Hz.
- Preemption gap: server has to cancel in-flight goal before accepting new one. Cumulative effect: jerky motion.
- Interpolator confusion: controller built to interpolate *toward* a waypoint over its duration, fighting itself when preempted every 10ms.
- Latency: action-server roundtripping on top of everything else.

Right architecture for servoing: `forward_position_controller` (or velocity), topic-based, no action semantics, latency = one control period.

**Practical implication for vendor asks:** a `FollowJointTrajectory` action server gets you MoveIt plan-and-execute, and that's it. For teleop or AI policy inference you need something else — usually `ros2_control` so you can switch between `joint_trajectory_controller` (plan-and-execute) and `forward_position_controller` (streaming) on the same hardware interface. Intermediate option some vendors land on: action server + a dedicated streaming topic that accepts position setpoints at high rate. Works for streaming but is ecosystem-incompatible (`moveit_servo` would need a custom output plugin).

## How major vendors structure their integration

### Universal Robots — gold standard

- Proper `ros2_control` hardware interface, position command + state interfaces plus `scaled_joint_command_interface` (includes speed scaling).
- Multiple controllers ship out of the box: `scaled_joint_trajectory_controller` (default), `forward_position_controller`, `forward_velocity_controller`, `forward_effort_controller`, `force_mode_controller`, `freedrive_mode_controller`, `io_and_status_controller`.
- `scaled_joint_trajectory_controller` is aware of teach pendant speed slider and safety slowdowns — trajectory monitoring doesn't false-trigger.
- `external_control` URCap pulls commands from ROS driver over RTDE at robot's native rate (~500 Hz on e-Series).
- One driver, whole product line (3 kg to 30 kg, CB3 and e-Series).
- Note: trajectory control is position-only. Velocity control on UR ROS2 driver was on the roadmap.

### Franka — torque story

- DOES have `ros2_control` integration (`franka_hardware::FrankaHardwareInterface`). Common misconception that they don't.
- Exposes position, velocity, AND effort command interfaces, plus Cartesian pose/velocity command interfaces.
- Full robot model exposed as state interface (mass matrix, Jacobian, coriolis) — your custom controllers can do model-based stuff in real time.
- 1 kHz cycle, hard real-time.
- MoveIt integration more bare-bones than UR's. Philosophy is "write custom controllers using FCI," with `ros2_control` as one option. UR philosophy is "use MoveIt out of the box."
- Network/state machine is finicky (FCI mode, brake unlocking, error recovery actions).

## Vendor ask sequencing (when the integration isn't there)

If a vendor has a low-level realtime API but no MoveIt integration:

**Short-term:** `FollowJointTrajectory` action server wrapping their trajectory interface. Unblocks MoveIt plan-and-execute via `MoveItSimpleControllerManager`. Weeks of work.

**Medium-term:** Proper `ros2_control` hardware interface plugin. Unlocks streaming, mode switching, simulation parity, the whole controller ecosystem. 1–3 months depending on what their realtime layer can do.

Frame as "step one now, step two on the roadmap." Sharp vendors may want to skip step one entirely and go straight to the hardware interface.

Additional asks for robots that have joint torque sensing but don't expose it:
- Effort state interface from existing sensing.
- Estimated tool wrench (no hardware change required).
- Streaming command interface with documented latency/jitter specs.
- Effort command interface (longer horizon — usually requires firmware work).