# Balancio GZ

A simulation of the [Balancio] self-balancing robot in Gazebo with ROS 2.

A nonlinear model predictive controller (NMPC) keeps it balanced and drives it along paths.
The controller uses the robot dynamics from [twsbr].
The Isaac Sim version of this robot is [balancio-sim].

## Pixi

You'll need a package manager called Pixi.
It installs ROS 2 and Gazebo for you, on Linux or macOS.
Follow this [link][pixi-installation] to install it.

Then, from the repo folder, install everything.
This also builds the ROS packages.

```text
pixi install
```

## Run

Start Gazebo with Balancio balancing in place:

```text
pixi run ros2 launch balancio_bringup balancio.launch.py
```

To follow a path, add `trajectory:=` with `line`, `circle`, `sine` or `figure8`.
Add `rviz:=true` to see the path in RViz.

```text
pixi run ros2 launch balancio_bringup balancio.launch.py trajectory:=figure8 rviz:=true
```

The controller is the Python one (CasADi) by default. Add `controller:=cpp` for the C++ one
(`balancio_nmpc`, acados), the version meant for the ESP32. Both read
`balancio_control/config/mpc.yaml` and use the same topics.

Each part also has its own launch file in `balancio_bringup`, to run in separate shells:
`sim.launch.py` (Gazebo and the robot), `control.launch.py` (the controller and path follower)
and `viz.launch.py` (RViz).

## Drive it yourself

Launch without a `trajectory`, then in another shell run:

```text
pixi run ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## The robot

The robot's size, mass and motors are in `src/balancio_description/config/balancio.yaml`.
Both the simulation and the controller read this file.

The controller settings are in `src/balancio_control/config/mpc.yaml`.

### Motors

The wheels are driven by TT gear motors through an L298N, as on the real robot.
The controller sends each motor a PWM duty between -1 and 1, not a torque.
The torque the motor gives drops as the wheel spins faster (back-EMF).

A Gazebo plugin simulates the motors at every physics step.
It also simulates the inertia of the spinning motor, and the free play in the gearbox (backlash).
Backlash is off by default.
To turn it on, set `backlash` in `balancio.yaml` to half of the total free play, in radians.
TT gearboxes usually have 2 to 4 degrees in total.
With backlash, the robot wobbles a little when standing still, like a real one.

### Sensors

The controller only uses the IMU, since the real robot has no wheel encoders.
It works out the robot's speed from the IMU and the duty it sent to the motors.
The position is the speed added up over time, so it drifts.

This depends on the motor numbers in `balancio.yaml` being right.
Measure them on the real robot before trusting it there.

The 3D model comes from Fusion 360.
To update it, export the whole assembly as USDZ, then run:

```text
pixi run cad-convert path/to/assembly.usdz
pixi install
```

This needs [Blender] installed.

## Tests

```text
pixi run test
pixi run test-cpp
```

`test` runs the Python tests, `test-cpp` builds and tests the C++ controller (`balancio_nmpc`).

## Contributing

Commits follow [Conventional Commits].
Install the git hooks once; they check the commit message and the code style.

```text
pixi run -e style hooks
```

[Balancio]: https://github.com/udesa-ai/balancio-kit/tree/main
[twsbr]: https://github.com/alvgaona/twsbr
[balancio-sim]: https://github.com/alvgaona/balancio-sim
[Blender]: https://www.blender.org/download/
[Conventional Commits]: https://www.conventionalcommits.org
[pixi-installation]: https://pixi.sh/latest/#installation
