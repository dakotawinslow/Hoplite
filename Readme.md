# Summary

Hoplite is a ROS‑based leader–follower formation control system for a small swarm of three AgileX LIMO robots equipped with Mecanum wheels. A human operator uses a wireless gamepad to command a virtual “center‑of‑mass” leader; the follower robots compute and navigate to their assigned positions using real‑time 120 Hz OptiTrack motion‑capture feedback bridged from ROS 2 into ROS 1. The system implements PD control for smooth omnidirectional movement and a repulsion‑force collision‑avoidance scheme. A simulated environment in RViz allows testing without hardware.
Table of Contents

    Hardware Requirements

    Software Requirements

    Hardware Setup

    Software Installation & Build

    Running the System

    Simulation

    Project Structure

    References

Hardware Requirements

    Three AgileX LIMO robots with Mecanum wheels

    OptiTrack motion‑capture system (minimum 120 Hz) with at least 6 infrared cameras

    Retroreflective marker sets for each robot (rigid‑body configuration)

    USB wireless gamepad (e.g. DualShock 4 or Xbox controller)

    ROS 1 (Melodic) on each robot computer (Ubuntu 18.04)

    ROS 2 (Foxy or later) for the OptiTrack VRPN bridge (Ubuntu 20.04)

    ROS 1–ROS 2 bridge package

Software Requirements

    ROS 1 Melodic

    ROS 2 Foxy (for VRPN‑ROS2 node and bridge)

    [agilex_limo driver package][limo-doc]

    Standard ROS packages: joy, tf, rviz, geometry_msgs, nav_msgs, ros1_bridge

    Python 3 with rospy, rclpy

Hardware Setup

    Assemble Mecanum wheels on each LIMO following the [AgileX user manual][limo-doc]

    Attach retroreflective marker sets in known rigid‑body configurations. Record the rigid‑body name in OptiTrack Motive.

    Arrange OptiTrack cameras around the arena for full‑volume coverage. Calibrate the system to achieve < 1 mm precision.

    Network configuration:

        All robots, the OptiTrack host PC, and the ROS 2 bridge Pi must be on the same LAN.

        Ensure multicast/UDP traffic for VRPN is allowed.

Software Installation & Build

# On each LIMO robot (Ubuntu 18.04 + ROS Melodic):
sudo apt update && sudo apt install ros-melodic-desktop-full python-rosdep
sudo rosdep init && rosdep update
cd ~/catkin_ws/src
git clone https://github.com/dakotawinalow/hoplite.git
git clone https://github.com/agilexrobotics/limo-driver.git
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make

# On OptiTrack host (Ubuntu 20.04 + ROS 2 Foxy):
sudo apt update && sudo apt install ros-foxy-desktop
sudo apt install ros-foxy-ros1-bridge ros-foxy-vrpn-client-ros2

Running the System

    Launch OptiTrack → ROS 2 VRPN node

ros2 launch vrpn_client_ros2 client.launch.py --ros-args -p server_address:=<OptiTrack_IP>

Start ROS 1–ROS 2 bridge

ros2 run ros1_bridge dynamic_bridge

On the leader robot

roslaunch hoplite leader.launch

    Connect gamepad; verify /joy and /joystick_controller/marker topics.

On each follower robot

roslaunch hoplite soldier.launch robot_id:=<1|2|3>

Visualization (optional)

    rviz -d $(rospack find hoplite)/rviz/formation.rviz

Simulation

To run in simulation without hardware:

roslaunch hoplite sim.launch

    Spawns three turtlebot3‑styled agents in Gazebo

    Uses the same leader and soldier nodes, but subscribes to simulated /gazebo/model_states instead of OptiTrack

Project Structure

hoplite/
├── launch/
│   ├── leader.launch
│   ├── soldier.launch
│   ├── sim.launch
├── src/
│   ├── joystick_control/
│   ├── hoplite_leader/
│   ├── hoplite_soldier/
│   ├── mocap_cleaner/
├── rviz/
│   └── formation.rviz
├── CMakeLists.txt
└── package.xml

    joystick_control: Reads /joy, publishes center‑of‑mass Marker & debug Twist

    hoplite_leader: Computes formation target poses (regular polygon)

    hoplite_soldier: PD control + repulsion collision avoidance, publishes /cmd_vel

    mocap_cleaner: Filters OptiTrack XZY → 2D XY poses

References

    AgileX LIMO user manual and driver: [limo-doc][limo-doc]

    ROS 1–ROS 2 bridge documentation

    PD control and potential‑field collision avoidance literature

[limo-doc]: https://github.com/agilexrobotics/limo-doc/blob/master/Limo%20user%20manual(EN).md
