# MoveIt interface for a simulated COMAU e.DO™ manipulator

<p align="center">
  <img src="https://github.com/lbusellato/rpc_project/blob/master/media/plancia.png" />
</p>

This repository contains the code for the final project of the Robot Programming
and Control course @UniVR.

The project is a ROS package that implements a MoveIt interface for the control 
of a simulated COMAU e.DO™ robotic manipulator. The package also covers the 
bringup of the robot in a Gazebo simulation, with the possibility of using Rviz
for the path planning with MoveIt or a rqt plugin for the manual control of the 
robot's joints.

## Setup

The package was developed and tested on Ubuntu 20.04. Compatibility with other 
distros or other operating systems is not guaranteed.

The package requires a [ROS Noetic](http://wiki.ros.org/noetic) installation, as
well as the MoveIt ROS packages, which can be installed with:

    sudo apt-get install ros-noetic-moveit

The package can then be installed as any other ROS package:

    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    git clone https://github.com/lbusellato/rpc_project
    cd .. && catkin_make

## Usage

The entry point for the package is the `edo.launch` file. The launch file sets 
up the Gazebo simulation, initializes MoveIt's Move Group Interface and launches
the main ROS node that offers user interaction. The launch file can be launched
as-is, alongside a MoveIt plugin for Rviz or alongside a rqt GUI for the manual 
control of the robot.

### Standalone execution

[GAZEBO W/ TERMINAL PICTURE]

The package can be launched as-is by executing:

    roslaunch edo edo.launch

The launch file sets up the Gazebo simulation, initializes MoveIt and launches 
the ROS node that handles user interaction. In this case user interaction is 
achieved with a console commands approach, in which the user inputs commands via
the terminal, which are then excecuted by the ROS node.

### Execution with Rviz

[GAZEBO W/ RVIZ PICTURE]

The package can be launched alongside the Rviz plugin by executing:

    roslaunch edo edo.launch rviz:=true

The launch file does everything the standalone case does, with the addition of
launching an Rviz interface for the interactive plannig of motions. In this case 
user interaction is achieved with a more direct approach, in which the user 
manually jogs the robot by moving its end effector, then planning the trajectory
between the starting and end points using MoveIt's planner.

### Execution with rqt GUI

[GAZEBO W/ RQT GUI PICTURE]

The package can be launched alongside the rqt GUI by executing:

    roslaunch edo edo.launch edo_gui:=true

The launch file does everything the standalone case does, with the addition of
launching an rqt-gui interface. In this case user interaction is achieved by 
direct manipulation of the joints of the robot, without the use of MoveIt's 
planning interface.

## Example tasks

### Pick and place

The ```edo_move_group_interface``` node implements a pick-and-place task 
parameterized on the markers on the work surface. Through the command line, one
can set up the pick-and-place targets by executing the command:

    pnp_target S E 

this spawns two cylinders in the given markers, and a sphere in the first one.
Alternatively, one can set up the pick-and-place targets by manually spawning the
models, for instance:

    spawn S cylinder 
    spawn E cylinder
    spawn S sphere

would produce the same setup as the previous command. The node doesn't keep track 
of what-is-spawned-where, so pay attention if doing the setup manually.

Once the task has been set up, it can be executed by executing the command:

    pnp S E

The robot should then move to the pick approach location, lower itself on the pick location and close the gripper, grasping the sphere. Then the robot will move back 
to the approach location. Finally it will move to the place approach location, lower itself on the place location, open the gripper and release the sphere. It should 
then return to the place approach location and then end by returning to the home
position.

The resulting execution should be similar to this:

<p align="center">
  <img src="https://github.com/lbusellato/rpc_project/blob/master/media/pnp.gif" />
</p>

Note that different behaviors can arise during different executions, depending on
the optimization steps performed by the planner.

### Cartesian path planning

The ```edo_move_group_interface``` node implements a Cartesian trajectory planning
task between four given markers on the work surface. In the current implementation
it is not possible to have two consecutive markers in the I through L and O 
through R regions, because the end-effector would have to cross

To plan and execute the path one can execute the command:

    cartesian S E A W

The robot should then move in a straight line from S to E, then from E to A, then
from A to W, then finally from W to S.

The resulting execution should be similar to this:

<p align="center">
  <img src="https://github.com/lbusellato/rpc_project/blob/master/media/cartesian.gif" />
</p>

## Implementation details

### Workspace modeling and URDF integration

### MoveGroupInterface implementation

The '''edo_move_group_interface''' node is the first feedback we get by running the command ('''rosrun edo main.py _ns: = edo''') it allows us to send commands by terminal to the robot to move it or to obtain information from it, such as the location of the joints, or EE.

It contains information about the positions in which the EE must stand in order to be in the correct position for each target.
In addition, it implements the following functions:

*__kill__ : Kill the execution 
*__set__ : Sets the given joint to the given angle in degrees
*__move__ : Moves the given joint by the given angle in degrees
*__set_gripper__ : Sets the gripper's span to the given width in millimeters
*__goto__ : Moves the robot to the given marker
*__home__ : Moves the robot to the home position
*__set_home__ : Sets the current joint state as the home position
*__print_joint__ : Prints the current joint state
*__print_cartesian__ : Prints the current pose
*__print_rpy__ : Prints the current rpy orientation of the EE
*__pnp__ : Executes pick and place between markerA and markerB
*__pnp_target__ : Spawns a sphere-cylinder setup in the given markers
*__cartesian__ : Plans and executes a cartesian path between the given markers
*__spawn__ : Spawns a shape at the given marker
*__delete__: Deletes the shape with the given name

This node is supported by edo_console, which menage every argoment passed by line of command.


### IKFast inverse kinematics plugin

### EdoGripper

### Gazebo Grasp Fix Plugin

### EdoConsole

### rqt_joint_trajectory_controller

## Acknowledgements

The URDF models for the robot and the gripper have been adapted from [eDO_description](https://github.com/Pro/eDO_description) and [edo_gripper](https://github.com/Pro/edo_gripper).

The configuration for the Gazebo simulation and Rviz visualization has been adapted from [edo_gazebo](https://github.com/Pro/edo_gazebo) and [edo_gripper](https://github.com/Pro/edo_gripper).

The package uses the [Gazebo grasp fix plugin](https://github.com/JenniferBuehler/gazebo-pkgs).

The rqt plugin for the slider control of the joints is based on the [joint_trajectory_controller](https://github.com/ros-controls/ros_controllers) plugin.
