# Audibot Simulator

This repository contains a Gazebo simulation model of an Audi R8. It is meant to be a very flexible simulation platform that supports single and multiple vehicle simulations.

To customize the model, include `audibot.urdf.xacro` in the `audibot_description` package in another URDF file and add sensors, plugins, etc.

`single_vehicle_example.launch` in the `audibot_gazebo` package shows how a single vehicle can be simulated in the root workspace with no TF prefix.

`two_vehicle_example.launch` shows how multiple vehicles can be simulated at the same time, with each in its own namespace and with a unique TF prefix.

To control the vehicle, publish the following topics:

- **steering_cmd** - `std_msgs/Float64` topic containing the desired steering wheel angle in radians
- **brake_cmd** - `std_msgs/Float64` topic containing the desired brake torque in Newton-meters (Nm)
- **throttle_cmd** - `std_msgs/Float64` topic containing the desired throttle percentage (range 0 to 1)
- **gear_cmd** - `std_msgs/UInt8` topic containing the desired gear (`DRIVE` = 0, `REVERSE` = 1)

Ground truth speed and yaw rate feedback are provided on the **twist** topic, which is of type `geometry_msgs/TwistStamped`

Current gear state is provided on the **gear_state** topic, which is of type `std_msgs/UInt8`. The gear state starts in `DRIVE` by default.

Some useful kinematics parameters:

- Gear ratio between steering wheel and equivalent bicycle steer angle = 17.3 : 1
- Wheelbase = 2.65 meters
- Track width = 1.638 meters
- Wheel radius = 0.36 meters

## Sensors

### Fixations

- **Laser:** origin: xyz="0 0 1.0" rpy="0 0 0", axis: xyz="0 1 0" rpy="0 0 0"
- **IMU:** origin: xyz="0 0 0.5" rpy="0 0 0" axis: xyz="0 0.5 0" rpy="0 0 0"

### Topics

- **Laser:** /$(arg robot_name)/audi_laser_scan
- **IMU:** /$(arg robot_name)/imu
- **Ground Truth Odom:** /audi_odom, /orange/odom and /blue/odom

## Command

- **CMD_VEL:** /audi_twist_cmd, /orange/cmd_vel and /blue/cmd_vel

## Launch Files

| **Package**       | **Launch File**   | **Includes**  | **Functionality** |
|:-------------     |:----------------- |:------------- | ----------------- |
| *audibot_gazebo*  | `audibot_robot.launch`  | - | Spawns a single URDF of the audibot   |
| *audibot_gazebo*  | `audibot_robot_sens.launch`  | - | Spawns a single URDF of the audibot with sensors   |
| *audibot_gazebo*  | `audibot_named_robot.launch`  | - | Spawns the multiple URDFs (Orange & Blue) of the audibot    |
| *audibot_gazebo*  | `audibot_named_robot_sens.launch` | - | Spawns the multiple URDFs (Orange & Blue) of the audibot with sensors    |
| *audibot_gazebo*  | `single_vehicle_example.launch`  | `audibot_robot.launch`  | Launches a single URDF of the audibot along with Gazebo world   |
| *audibot_gazebo*  | `single_vehicle_example_sens.launch`  | `audibot_robot_sens.launch`  | Launches a single URDF of the audibot with sensors along with Gazebo world   |
| *audibot_gazebo*  | `two_vehicle_example.launch`  | `audibot_named_robot.launch` | Launches the multiple URDFs (Orange & Blue) of the audibot along with Gazebo world   |
| *audibot_gazebo*  | `two_vehicle_example_sens.launch`  | `audibot_named_robot_sens.launch` | Launches the multiple URDFs (Orange & Blue) of the audibot with sensors along with Gazebo world   |
| *audibot_control*  | `control_single_audi.launch`  | - | Launches the nodes responsible for converting the CMD to lat and lot control for a single vehicle   |
| *audibot_control*  | `control_orange_audi.launch`  | - | Launches the nodes responsible for converting the CMD to lat and lot control for the orange vehicle   |
| *audibot_control*  | `control_blue_audi.launch`  | - | Launches the nodes responsible for converting the CMD to lat and lot control for the blue vehicle   |
| *audibot_control*  | `manual_control_single_audi.launch`  | `control_single_audi.launch` | Launches the node of keyboard control and remaps the required topics of a single vehicle   |
| *audibot_control*  | `manual_control_orange_audi.launch`  | `control_orange_audi.launch` | Launches the node of keyboard control and remaps the required topics of the orange vehicle   |
| *audibot_control*  | `manual_control_blue_audi.launch`  | `control_blue_audi.launch` | Launches the node of keyboard control and remaps the required topics of the blue vehicle   |
| *audibot_odometry*  | `odom_single_audi.launch`  | - | Launches the node that publishes the groundtruth odometry of a single vehicle   |
| *audibot_odometry*  | `odom_orange_audi.launch`  | - | Launches the node that publishes the groundtruth odometry of the orange vehicle   |
| *audibot_odometry*  | `odom_blue_audi.launch`  | - | Launches the node that publishes the groundtruth odometry of the blue vehicle   |
| *audibot_odometry*  | `odom_two_audi.launch`  | `odom_orange_audi.launch`, `odom_blue_audi.launch` | Launches the nodes that publish the groundtruth odometry of the two vehicles   |