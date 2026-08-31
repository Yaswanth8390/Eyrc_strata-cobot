# e-Yantra Robotics Competition 2026-27 — StrataCobot (SC)

Ubuntu 24.04 · ROS 2 Jazzy · Gazebo Harmonic

## 1. Install the task dependencies

From this directory:

```bash
./requirements.sh
```

## 2. Build the workspace

```bash
cd ..
colcon build
source install/setup.bash
```

Add the `source` line to your `~/.bashrc`.

## 3. Start the simulation

```bash
ros2 launch eyantra_kepler_colony task0.launch.py
```

Leave it running in its own terminal.

## 4. Check your setup

In a second terminal, source the workspace again and run this from this directory:

```bash
./eyrc-sc-evaluator
```

Enter your team ID when asked. It writes `result-SC-<your-id>-task-0-<date>.json`.
Upload that file to the portal exactly as it is: do not rename it, do not edit it and do
not compress it. A file under any other name is not graded.
