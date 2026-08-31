# ur_description

UR7e description, meshes and simulation bringup for the StrataCobot task.

Modified by e-Yantra from the upstream Universal Robots ROS 2 description.

## Contents

    config/ur7e/        kinematics, joint limits, physical and visual parameters
    config/             controller_manager, Cartesian servo and gz bridge configuration
    meshes/ur7e/        visual (.dae) and collision (.stl) meshes
    meshes/realsense/   camera mesh
    urdf/               xacro description; ur_gz.urdf.xacro is the simulation model
    launch/             spawn_ur7e.launch.py
    rviz/               RViz configuration
    bin/                cartesian_servo, prebuilt
    lib/                libmagnet_system.so, the electromagnet tool plugin, prebuilt
    env-hooks/          adds the plugin directory to GZ_SIM_SYSTEM_PLUGIN_PATH

## Running

task0.launch.py brings up the world with the arm already in it:

    ros2 launch eyantra_kepler_colony task0.launch.py

To spawn the arm into a world that is already running:

    ros2 launch ur_description spawn_ur7e.launch.py

## Interfaces

The arm's command and status topics are introduced in Task 1, with example code, at the
point where you need them. For Task 0 nothing has to be published: `ros2 topic list` with
the simulation running shows what is available.
