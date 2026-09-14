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

## 5. Docker Setup
Only if you don't have Ubuntu 24.04.
Install Docker first and clone this repo into your Desktop.

Run the following to build and run the Dockerfile present here.
```bash
docker build -t strata-cobot:jazzy .
```
Now you can run it. Most Docker containers don't get GUI access, which is why we explicitly allow it and also allow volumes so that the changes you make in the container are saved.
```bash
docker run -it \
  --name strata-cobot \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --volume ~/Desktop/Eyrc_strata-cobot:/root/ros2_ws/src \
  strata-cobot:jazzy
```
Make sure you provide the local repo path in the volume.

```bash
docker start -i strata-cobot
```
After starting the container you will be looking at the ros2_ws and then you can build and source.
```bash
colcon build
. install/setup.bash
```
Once you have sourced the workspace you can easily launch the GUI.

```bash
ros2 launch eyantra_kepler_colony task0.launch.py
```
You should be getting a gazebo window with the environment and you can cross check by ros2 topic list from another terminal by using the below command
```bash
docker exec -it strata-cobot bash
root@4f96b2dacb55:~/ros2_ws# ros2 topic list
```
