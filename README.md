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

## Table Of Contents

- [MoveIt interface for a simulated COMAU e.DO™ manipulator](#moveit-interface-for-a-simulated-comau-edo--manipulator)
  * [Setup](#setup)
  * [Usage](#usage)
    + [Standalone execution](#standalone-execution)
    + [Execution with Rviz](#execution-with-rviz)
    + [Execution with rqt GUI](#execution-with-rqt-gui)
    + [Terminating the execution](#terminating-the-execution)
  * [Example tasks](#example-tasks)
    + [Pick and place](#pick-and-place)
    + [Cartesian path planning](#cartesian-path-planning)
  * [Implementation details](#implementation-details)
    + [Workspace modeling and URDF integration](#workspace-modeling-and-urdf-integration)
    + [MoveGroupInterface implementation](#movegroupinterface-implementation)
    + [IKFast inverse kinematics plugin](#ikfast-inverse-kinematics-plugin)
    + [EdoGripper](#edogripper)
    + [Gazebo Grasp Fix Plugin](#gazebo-grasp-fix-plugin)
    + [EdoConsole](#edoconsole)
    + [rqt_joint_trajectory_controller](#rqt-joint-trajectory-controller)
    + [Pick and place sample task](#pick-and-place-sample-task)
    + [Cartesian path planning sample task](#cartesian-path-planning-sample-task)
  * [Acknowledgements](#acknowledgements)

## Setup

The package was developed and tested on Ubuntu 20.04. Compatibility with other 
distros or other operating systems is not guaranteed.

The package requires a [ROS Noetic](http://wiki.ros.org/noetic) installation, as
well as the MoveIt ROS packages, which can be installed with:

    sudo apt-get update
    sudo apt-get install --reinstall ros-noetic-moveit ros-noetic-moveit-kinematics

The package can then be installed as any other ROS package:

    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    git clone https://github.com/lbusellato/rpc_project
    cd .. && catkin_make
    
If catkin_make gives errors related to Gazebo, it is suggested to reinstall the related
ROS packages. For Gazebo version 11 this is done as follows:

    sudo apt-get purge libgazebo11-dev
    sudo apt-get update
    sudo apt-get install libgazebo11-dev ros-noetic-gazebo-*

## Usage

The entry point for the package is the `edo.launch` file. The launch file sets 
up the Gazebo simulation, initializes MoveIt's Move Group Interface and launches
the main ROS node that offers user interaction. The launch file can be launched
as-is, alongside a MoveIt plugin for Rviz or alongside a rqt GUI for the manual 
control of the robot. As is always the case, before launching the launch file 
both ROS and the workspace must be sourced by executing:

    source {PATH_TO_ROS}/setup.bash && source ~/catkin_ws/devel/setup.bash

### Standalone execution

<p align="center">
  <img src="https://github.com/lbusellato/rpc_project/blob/master/media/bash.gif"  width="800" height="480" />
</p>


The package can be launched as-is by executing:

    roslaunch edo edo.launch

The launch file sets up the Gazebo simulation, initializes MoveIt and launches 
the ROS node that handles user interaction. In this case user interaction is 
achieved with a console commands approach, in which the user inputs commands via
the terminal, which are then excecuted by the ROS node.

### Execution with Rviz

<p align="center">
  <img src="https://github.com/lbusellato/rpc_project/blob/master/media/rviz.gif"  width="800" height="480" />
</p>

The package can be launched alongside the Rviz plugin by executing:

    roslaunch edo edo.launch rviz:=true

The launch file does everything the standalone case does, with the addition of
launching an Rviz interface for the interactive plannig of motions. In this case 
user interaction is achieved with a more direct approach, in which the user 
manually jogs the robot by moving its end effector, then planning the trajectory
between the starting and end points using MoveIt's planner.

### Execution with rqt GUI

<p align="center">
  <img src="https://github.com/lbusellato/rpc_project/blob/master/media/edo_gui.gif"  width="800" height="480" />
</p>

The package can be launched alongside the rqt GUI by executing:

    roslaunch edo edo.launch edo_gui:=true

The launch file does everything the standalone case does, with the addition of
launching an rqt-gui interface. In this case user interaction is achieved by 
direct manipulation of the joints of the robot, with the use of a set of sliders.

### Terminating the execution

To terminate the execution, in the terminal where __edo.launch__ was launched, first kill the instance of EdoConsole by issuing the command:

    kill

Then, once the process exits, terminate the EdoMoveGroupInterface by pressing CTRL+C.

## Sample tasks

### Pick and place

The ```edo_move_group_interface``` node implements a pick-and-place task 
parameterized on the markers on the work surface. Through the command line, one
can set up the task with the command:

    pnp_target S

that spawns a sphere in the given marker.

Once the task has been set up, it can be executed by executing the command:

    pnp S E

The robot should then move above the S marker, lower itself on the marker location
and close the gripper, grasping the sphere. Then the robot should move back to the approach location. Finally it should move above the E marker, lower itself on 
the marker location, open the gripper and release the sphere. It should 
then first return to the approach location and then end the task by returning 
to the home position.

The resulting execution should be similar to this:

<p align="center">
  <img src="https://github.com/lbusellato/rpc_project/blob/master/media/pnp.gif"  width="450" height="450" />
</p>

Note that different behaviors can arise during different executions, depending on
the optimization steps performed by the planner.

### Cartesian path planning

The ```edo_move_group_interface``` node implements a cartesian trajectory planning
task between four given markers on the work surface. Note that not all four-marker combinations result in a feasible cartesian path. For instance having two consecutive markers from the inner region (i.e. I through L and O through R) results in a failed plan, because the manipulator is too close to the joint limits.

To plan and execute the path one can execute the command:

    cartesian S E A W

The robot should then move in a straight line from S to E, then from E to A, then
from A to W, then finally from W to S.

The resulting execution should be similar to this:

<p align="center">
  <img src="https://github.com/lbusellato/rpc_project/blob/master/media/cartesian.gif"  width="450" height="450"/>
</p>

## Implementation details

### Workspace modeling and URDF integration
The work area was modeled to be as similar as possible to the real-life workspace of the COMAU e.DO™ robotic manipulator in the [UniVR IceLab](https://www.icelab.di.univr.it/). 
The manipulator is placed on a table of dimension (1.30x1.30x0.83), on which is also
placed a replica of e.DO™'s working board, which is a mat that shows the x-y cartesian
axes as well as some known positions marked with letters.

<p align="center">
  <img src="https://github.com/lbusellato/rpc_project/blob/master/media/edo_workspace_v3.jpg" width="450" height="450" />
</p>
<p align="center">
  <em>e.DO™ working board</em>
</p>


The creation of the model of the table was divided into two steps. In the first 
step, digital models of the table and working board were created. The 3D model 
of the table was created using Blender, starting from the measured dimensions of
the real one.
The working board was replicated in CAD, and then used as a texture to be placed
on top of the 3D model. The resulting 3D model was then exported in __.dae__ 
format, which is the preferred file format for importing 3D models into Gazebo.

In the second step the model was actually imported into the Gazebo simulation. 
This was achieved by adding an additional link between the world and the base 
link of the robot, in the [__edo.xacro__](https://github.com/lbusellato/rpc_project/blob/master/edo/urdf/edo.xacro) file.

<p align="center">
  <img src="https://github.com/lbusellato/rpc_project/blob/master/media/edo.png" width="450" height="450" />
</p>
<p align="center">
  <em>Resulting model</em>
</p>

The mesh obtained in Blender was used only for the visual appearance of the workspace. 
For the collision properties, a box with the same dimensions as the table was 
used. This helps reduce the loss of computation time while processing collision checks.

### MoveGroupInterface implementation

The simplest MoveIt user interface is the MoveGroupinterface, which is a set of
wrappers that provide functions to cover most of the basic operations, such as 
setting joint or cartesian space goals, creating motion plans, moving the robot, 
adding objects in the environment and attaching/detaching them from the robot.

The interface is implemented by the [__edo_move_group_interface.py__](https://github.com/lbusellato/rpc_project/blob/master/edo/src/edo_move_group_interface.py) node.

The node has three main attributes:

* *robot*, a RobotCommander instance that handles the physical structure of the 
robot itself, providing link and group names as well as information on the current
state.
* *scene*, a PlanningSceneInterface instance that handles the robot's knowledge
of the surrounding environment, providing methods to add/remove objects from the 
world and attaching/detaching them from the robot.
* *edo_move_group*, a MoveGroupCommander instance that handles the planning and 
execution of robot motions, providing methods for path planning, setting joint or cartesian space goals, information on the joint states and the spatial configuration
of the robot's end-effector.

The node implements all basic capabilities of the MoveGroupInterface and PlanningSceneInterface classes, namely:

* Moving the robot to a desired joint goal, with the [*go_to_joint_state*](https://github.com/lbusellato/rpc_project/blob/master/edo/src/edo_move_group_interface.py#L177) function.

  This is implented by directly passing the joint state goal to the MoveGroup's 
  *go* function.
* Moving the robot to a desired pose goal, with the [*go_to_pose_goal*](https://github.com/lbusellato/rpc_project/blob/master/edo/src/edo_move_group_interface.py#L205) function.

  This is implemented by fist setting the MoveGroup's pose target, and then by
  calling the *go* function.
* Moving the robot to a desired xyz coordinate with a given rpy orientation of the end-effector, with the [*go_to_xyz_rpy*](https://github.com/lbusellato/rpc_project/blob/master/edo/src/edo_move_group_interface.py#L200) function.

  This is implemented with the above descripted *go_to_pose_goal* function. The target pose is computed with the [*pose_from_xyz_rpy*](https://github.com/lbusellato/rpc_project/blob/master/edo/src/edo_move_group_interface.py#L186) function, which takes as input the xyz coordinates and rpy orientations and constructs a Pose message with them. The quaternion representing the orientation is computed from rpy with the *quaternion_from_euler* function of the *tf.transformations* module.
* The planning of paths in cartesian space, with the [*plan_cartesian_path*](https://github.com/lbusellato/rpc_project/blob/master/edo/src/edo_move_group_interface.py#L241) function.

  This is implemented simply by passing a list of waypoints to the MoveGroup's *compute_cartesian_path* function, which also takes as arguments an end-effector step value, for the generation of intermediate points between the provided waypoints, and a jump threshold, which controls the check for infeasible jumps in the joint space. By default the end-effector step is set to 0.0001 and the jump threshold is set to 0, disabling the jump check.
* The execution of the computed plan, with the [*execute_plan*](https://github.com/lbusellato/rpc_project/blob/master/edo/src/edo_move_group_interface.py#L249) function.

  This is just a wrapper around the MoveGroup's own *execute* function.
* The spawning of objects in the world, with the [*spawn_model*](https://github.com/lbusellato/rpc_project/blob/master/edo/src/edo_move_group_interface.py#L414) function.

  This is implemented by calling Gazebo's *spawn_urdf_model* service to make the model visually appear in the scene, and then by calling the PlanningSceneInterface's appropriate method (*add_box*, *add_sphere* or *add_mesh*) in order to add the object to the planning scene, so that the planner takes it into account while computing motion plans.
* The removal of objects from the world, with the [*delete_model*](https://github.com/lbusellato/rpc_project/blob/master/edo/src/edo_move_group_interface.py#L468) function.

  This is implemented by calling Gazebo's *delete_model* service to remove the object from the scene, and then by calling the PlanningSceneInterface's *remove_world_object* method, in order to remove the object from the planning scene.

The node also implements the control of the gripper, with the [*set_gripper_span*](https://github.com/lbusellato/rpc_project/blob/master/edo/src/edo_move_group_interface.py#L253) function. The function interacts with the [__edo_gripper_node.py__](https://github.com/lbusellato/rpc_project/blob/master/edo/src/edo_gripper_node.py) node, 
which actually handles the control of the gripper.

Finally, the node implements utility functions, such as the printing of the pose, joint state or rpy orientation of the end-effector (respectively with the [*print_cartesian*](https://github.com/lbusellato/rpc_project/blob/master/edo/src/edo_move_group_interface.py#L276), [*print_joint*](https://github.com/lbusellato/rpc_project/blob/master/edo/src/edo_move_group_interface.py#L290) and [*print_rpy*](https://github.com/lbusellato/rpc_project/blob/master/edo/src/edo_move_group_interface.py#L280) functions), the setting and reaching of the home position (with the [*set_home*](https://github.com/lbusellato/rpc_project/blob/master/edo/src/edo_move_group_interface.py#L166) and [*go_home*](https://github.com/lbusellato/rpc_project/blob/master/edo/src/edo_move_group_interface.py#L171) functions) and the motion of the robot to one
of the known markers (with the [*go_to_marker*](https://github.com/lbusellato/rpc_project/blob/master/edo/src/edo_move_group_interface.py#L217) function). 

### IKFast inverse kinematics plugin

IKFast is an analytical inverse kinematics solver plugin part of the [OpenRAVE](http://openrave.org/) motion planning environment.

It has been choosen instead of MoveIt's default KDL plugin because the latter did
not perform acceptably with e.DO™. Specifically, an high failure rate in the 
planning stage and a long computation time were observed. It is [reported](https://canyonkang.wordpress.com/portfolio/robotics-inverse-kinematics-implementation-kdl-ik-vs-ik-fast/) that
IKFast has an higher success rate in trajectory planning compared to KDL, as well
as a much faster computation time.

To use IKFast with e.DO™, the plugin package was generated by following the [tutorial](https://ros-planning.github.io/moveit_tutorials/doc/ikfast/ikfast_tutorial.html), and then merged into the edo package.
Then in the [__kinematics.yaml__](https://github.com/lbusellato/rpc_project/blob/master/edo/config/kinematics.yaml) file the correct plugin was indicated in the 
*kinematics_solver* property.

### EdoGripper
The end effector mounted on the manipulator is a linear gripper with two fingers that is installed on the wrist of the robot. The prismatic joint allows a linear movement, to open and close the two fingers like so:

<p align="center">
  <img src="https://github.com/lbusellato/rpc_project/blob/master/media/edo_gripper.png" width="450" height="200" />
</p>
<p align="center">
  <em>e.DO™ gripper</em>
</p>

The gripper is managed by the [__edo_gripper_node.py__](https://github.com/lbusellato/rpc_project/blob/master/edo/src/edo_gripper_node.py) node. The node subscribes 
to the following topics:

* set_gripper_span, which accepts a Float32 message containing the desired width
of the gripper.
* joint_states, which accepts a JointState message containing the current state 
of the joints, which is used to update the current gripper's width.

Once a message is received on the set_gripper_span topic, the node computes the
correct angle values for the fingers of the gripper, publishing the results on the following topics:

* edo_gripper_left_base_controller, Float64 message.
* edo_gripper_left_finger_controller, Float64 message.
* edo_gripper_right_base_controller, Float64 message.
* edo_gripper_right_finger_controller, Float64 message.

The values for the fingers are computed with a "magic formula", that comes from the linear regression of manual measures of the span of the gripper and the angle of the base fingers.

The node also publishes the current status and width of the gripper on the 
following topics:

* edo_gripper_state, Int8 message containing 1 or 0 if the gripper is, 
respectively, moving or not.
* edo_gripper_span, Float32 message containing the current width of the gripper.

The gripper's width is updated with the update function, which is called within 
a 10Hz control loop. The width is computed by inverting the previously described "magic formula".

### Gazebo Grasp Fix Plugin

The [Gazebo grasp fix plugin](https://github.com/JenniferBuehler/gazebo-pkgs) is
a package used in order to achieve a realistic grasp behavior. This is needed 
because Gazebo is not yet optimized to handle grasping interactions between objects.

The package was cloned from source and merged into the edo package. The plugin 
is then loaded in the [__edo_gripper_dummy.xacro__](https://github.com/lbusellato/rpc_project/blob/master/edo/urdf/edo_gripper_dummy.xacro#L230) file. The adopted
settings for the plugin are:

* gripper_link: edo_gripper_left_finger_link and edo_gripper_left_finger_link
* forces_angle_tolerance: 90
* update_rate: 32
* grip_count_threshold: 1
* max_grip_count: 2
* release_tolerance: 0.005
* disable_collisions_on_attach: true

The meaning of each parameter is explained on the [github page](https://github.com/JenniferBuehler/gazebo-pkgs/wiki/The-Gazebo-grasp-fix-plugin) of the plugin. 
These parameters were tuned during development, until the resulting grasp was
reliable and realistic enough.

The plugin basically works by checking if two opposing forces are applied by the gripper links on an object. If they are, the object is fixed to the end-effector link. When this check fails, the object is released.

### EdoConsole

<p align="center">
  <img src="https://github.com/lbusellato/rpc_project/blob/master/media/edo_console.png" width="450" height="250" />
</p>
<p align="center">
  <em>EdoConsole</em>
</p>

The main user interaction with the robot is through a terminal-like interface 
implemented by the [__edo_console.py__](https://github.com/lbusellato/rpc_project/blob/master/edo/src/edo_console.py) node.

The node abstracts the handling of console commands, which are supplied as a 
dictionary like:

    "command" : { "desc" : "Command description",
                  "args" : [arg1, arg2, ...],
                  "types" : [type(arg1), type(arg2), ...],
                  "callback" : function() }

The node automatically handles the casting of the strings coming from the console
to the correct types, and the call to the specified callback function.

The implemented commands are:

* __kill__ : Kills the execution 
* __set__ joint angle: Sets the given joint to the given angle in degrees
* __move__ joint angle: Moves the given joint by the given angle in degrees
* __set_gripper__ span: Sets the gripper's span to the given width in millimeters
* __goto__ marker: Moves the robot to the given marker
* __home__ : Moves the robot to the home position
* __set_home__ : Sets the current joint state as the home position
* __print_joint__ : Prints the current joint state
* __print_cartesian__ : Prints the current pose
* __print_rpy__ : Prints the current rpy orientation of the EE
* __pnp__ markerA marker B : Executes pick and place between markerA and markerB
* __pnp_target__ marker: Spawns a sphere for pick and place in the given marker
* __cartesian__ m1 m2 m3 m4: Plans and executes a cartesian path between the given markers
* __spawn__ marker shape: Spawns a shape at the given marker. Available shapes are 'box', 
'cylinder' and 'sphere'.
* __delete__ name: Deletes the shape with the given name

### rqt_joint_trajectory_controller

<p align="center">
  <img src="https://github.com/lbusellato/rpc_project/blob/master/media/edo_gui.png" width="450" height="450" />
</p>
<p align="center">
  <em>Edo GUI</em>
</p>

To offer a more intuitive way of manipulating the robot, a GUI was implemented, 
based on the [joint_trajectory_controller](https://github.com/ros-controls/ros_controllers) plugin for rqt.

The plugin was modified by adding a slider for the control of the gripper and by removing the interface for the choosing of the arm controller and namespace, 
since both values are already known. The main edits to the plugin are in the 
[__joint_trajectory_controller.py](https://github.com/lbusellato/rpc_project/blob/master/edo/src/rqt_joint_trajectory_controller/joint_trajectory_controller.py) file.

### Pick and place sample task

First of all, the robot was jogged manually using the rqt plugin in order to 
register the xyz coordinates of all markers on the working board, as well as the
corresponding rpy orientation of the end effector.

The [*pick_and_place*](https://github.com/lbusellato/rpc_project/blob/master/edo/src/edo_move_group_interface.py#L306) function takes as arguments the marker where to
pick the sphere and the marker where to place it. The task is divided in several
steps:

* Pick approach: the end-effector goes 5 millimeters above the pick marker, then the gripper fully opens.
* Pick: the end effector lowers itself on the pick marker, then the gripper closes and
grasps the sphere.
* Pick approach: the end-effector goes back to the pick approach location.
* Place approach: the end-effector goes 5 millimeters above the place marker
* Place: the end-effector lowers itself on the place marker, then the gripper opens
and the sphere is released.
* Place approach: the end-effector goes back to the place approach location.
* Homing: the robot returns to the home position.

In the pick step the sphere, if it exists, is attached to the end-effector in the
planning scene. This ensures that in the following steps the planner will take into
account the bounding box of the sphere for collision checking, as well as its mass and
inertia. In the place step the sphere, if a sphere was picked, is detached from
the end-effector in the planning scene.

The gripper span set for the pick step is slightly smaller than the actual diameter
of the sphere (0.025 mm). This helps in consistently achieving the grasp. In a 
real robot this value should probably be equal to the actual width of the object
to be picked.

### Cartesian path planning sample task

The [*cartesian*](https://github.com/lbusellato/rpc_project/blob/master/edo/src/edo_move_group_interface.py#L372) function takes as arguments four markers between
which to plan a cartesian trajectory. It does so by first computing a corresponding
list of four poses, which is then passed to the MoveGroupInterface's *plan_cartesian_path* function. The planner returns a motion plan and a numeric 
value representing the fraction of the path that it was able to generate successfully.
If the fraction is equal to 1, i.e. if all of the path was generated, the plan is
then executed.

## Acknowledgements

The URDF models for the robot and the gripper have been adapted from [eDO_description](https://github.com/Pro/eDO_description) and [edo_gripper](https://github.com/Pro/edo_gripper).

The configuration for the Gazebo simulation and Rviz visualization has been adapted from [edo_gazebo](https://github.com/Pro/edo_gazebo) and [edo_gripper](https://github.com/Pro/edo_gripper).

The package uses the [Gazebo grasp fix plugin](https://github.com/JenniferBuehler/gazebo-pkgs).

The rqt plugin for the slider control of the joints is based on the [joint_trajectory_controller](https://github.com/ros-controls/ros_controllers) plugin.
