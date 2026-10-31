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
pixi run ros2 launch balancio_control balancio_mpc.launch.py
```

To follow a path, add `trajectory:=` with `line`, `circle`, `sine` or `figure8`.

```text
pixi run ros2 launch balancio_control balancio_mpc.launch.py trajectory:=figure8
```

## Visualize

In another shell, stream the robot, its path and its sensors to [Rerun] with [Rewire]:

```text
pixi run viz
```

## Drive it yourself

Launch without a `trajectory`, then in another shell run:

```text
pixi run ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## The robot

The robot's size, mass and motors are in `src/balancio_description/config/balancio.yaml`.
Both the simulation and the controller read this file.

The controller settings are in `src/balancio_control/config/mpc.yaml`.

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
```

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
[Rerun]: https://rerun.io
[Rewire]: https://rewire.run
[Conventional Commits]: https://www.conventionalcommits.org
[pixi-installation]: https://pixi.sh/latest/#installation
