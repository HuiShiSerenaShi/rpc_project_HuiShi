#!/usr/bin/env python3

import copy
import moveit_commander
import os
import roslib; roslib.load_manifest('urdfdom_py')
import rospkg
import rospy
import sys
import edo_utils
import edo_console

from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, PoseStamped
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from moveit_msgs.msg import DisplayTrajectory
from std_msgs.msg import Float32
from tf.transformations import quaternion_from_euler, euler_from_quaternion

PI = 3.14159265
DEG2RAD = PI / 180.0
RAD2DEG = 180.0 / PI

class EdoMoveGroupInterface(object):

    def __init__(self, home=[0, 0, 0, 0, 0, 1.57 / 2]) -> None:
        super(EdoMoveGroupInterface, self).__init__()
        
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("edo_move_group_interface", anonymous=True)

        # Handling of the robot itself
        self.robot = RobotCommander()
        # Handling of robot - scene interaction
        self.scene = PlanningSceneInterface()
        # Handling of trajectory generation and following
        group_name = "edo"
        self.edo_move_group = MoveGroupCommander(group_name)

        # Publishes the ghost trajectory for Rviz
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            DisplayTrajectory,
            queue_size=20,
        )

        # Gripper controller publisher
        self.gripper_span_pub = rospy.Publisher("set_gripper_span", Float32, queue_size=10)

        # Misc class attributes
        self.home = home
        self.planning_frame = self.edo_move_group.get_planning_frame()
        self.eef_link = self.edo_move_group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()
        self.state = self.robot.get_current_state()
        # Keep track of spawned models
        self.spawned_models = []
        self.model_counter = 0

        # xyz-rpy coords of the marked circles in the workspace
        self.marker_dict = {
            "A" : { 'xyz' : [ 0.405, 0.295, 0.9], 'rpy' : [173, 6, 168]},
            "B" : { 'xyz' : [ 0.295, 0.405, 0.9], 'rpy' : [173, 6, -174]},
            "C" : { 'xyz' : [ 0.155, 0.475, 0.9], 'rpy' : [173, 6, -155]},
            "D" : { 'xyz' : [ 0.000, 0.500, 0.9], 'rpy' : [173, 6, -138]},
            "E" : { 'xyz' : [ 0.325, 0.235, 0.9], 'rpy' : [180, 0, 180]},
            "F" : { 'xyz' : [ 0.235, 0.325, 0.9], 'rpy' : [180, 0, -170]},
            "G" : { 'xyz' : [ 0.125, 0.380, 0.9], 'rpy' : [180, 0, -152]},
            "H" : { 'xyz' : [ 0.000, 0.400, 0.9], 'rpy' : [180, 0, -134]},
            "I" : { 'xyz' : [ 0.240, 0.175, 0.9], 'rpy' : [-177.32, -4.21, -176.89]},
            "J" : { 'xyz' : [ 0.175, 0.240, 0.9], 'rpy' : [-177.32, -4.21, -176.89]},
            "K" : { 'xyz' : [ 0.090, 0.285, 0.9], 'rpy' : [-177.32, -4.206, -141.293]},
            "L" : { 'xyz' : [ 0.000, 0.300, 0.9], 'rpy' : [-177.325, -4.206, -123.493]},
            "O" : { 'xyz' : [ 0.245, -0.175, 0.9], 'rpy' : [-177.32, -4.21, 111.25]},
            "P" : { 'xyz' : [ 0.180, -0.240, 0.9], 'rpy' : [-177.32, -4.20, 92.84]},
            "Q" : { 'xyz' : [ 0.091, -0.288, 0.9], 'rpy' : [-177, -4.21, 75]},
            "R" : { 'xyz' : [ 0.000, -0.300, 0.9], 'rpy' : [-177, -4.32, 57]},
            "S" : { 'xyz' : [ 0.320, -0.235, 0.9], 'rpy' : [-180, 0, 100]},
            "T" : { 'xyz' : [ 0.235, -0.325, 0.9], 'rpy' : [-180, 0, 82]},
            "U" : { 'xyz' : [ 0.120, -0.380, 0.9], 'rpy' : [-180, 0, 64]},
            "V" : { 'xyz' : [ 0.000, -0.400, 0.9], 'rpy' : [-180, 0, 46]},
            "W" : { 'xyz' : [ 0.405, -0.295, 0.9], 'rpy' : [173, 6, 96]},
            "X" : { 'xyz' : [ 0.295, -0.405, 0.9], 'rpy' : [173, 6, 78]},
            "Y" : { 'xyz' : [ 0.155, -0.475, 0.9], 'rpy' : [173, 6, 60]},
            "Z" : { 'xyz' : [ 0.000, -0.500, 0.9], 'rpy' : [173, 6, 42]},
        }

        # Pick and place targets
        self.pick_target = "A"
        self.place_target = "Z"
        # Cartesian traj targets
        self.cartesian_targets = ["A", "B", "F", "E"]

        # Bring the robot home
        self.go_home()

        rospy.loginfo("Initialized edo MoveGroupInterface.")

        # Dictionary of console commands
        self.commands = {
            "kill" :  { "desc" : "Kill the execution.",
                        "args" : [],
                        "types" : [],
                        "callback" : self.kill },
            "set" :  { "desc" : "Sets the given joint to the given angle in degrees.",
                                "args" : ['joint', 'angle'],
                                "types" : [int, float],
                                "callback" : self.set_joint },
            "move" :  { "desc" : "Moves the given joint by the given angle in degrees.",
                            "args" : ['joint', 'angle'],
                            "types" : [int, float],
                            "callback" : self.move_joint },
            "set_gripper" :  { "desc" : "Sets the gripper's span to the given width in millimeters.",
                        "args" : ['width'],
                        "types" : [float],
                        "callback" : self.set_gripper_span },
            "goto" : { "desc" : "Moves the robot to the given marker.",
                        "args" : ['marker'],
                        "types" : [str],
                        "callback" : self.go_to_marker },
            "home" :  { "desc" : "Moves the robot to the home position.",
                        "args" : [],
                        "types" : [],
                        "callback" : self.go_home },
            "set_home" :  { "desc" : "Sets the current joint state as the home position.",
                            "args" : [],
                            "types" : [],
                            "callback" : self.set_home },
            "print_joint" :  { "desc" : "Prints the current joint state.",
                               "args" : [],
                               "types" : [],
                               "callback" : self.print_joint },
            "print_cartesian" :  { "desc" : "Prints the current pose.",
                                   "args" : [],
                                   "types" : [],
                                   "callback" : self.print_cartesian },
            "print_rpy" :  { "desc" : "Prints the current rpy orientation of the EE.",
                             "args" : [],
                             "types" : [],
                             "callback" : self.print_rpy },
            "pnp" :  { "desc" : "Executes pick and place between markerA and markerB.",
                        "args" : ['markerA', 'markerB'],
                        "types" : [str, str],
                        "callback" : self.pick_and_place },
            "pnp_target" :  { "desc" : "Spawns a sphere-cylinder setup in the given markers.",
                            "args" : ['markerA', 'markerB'],
                            "types" : [str, str],
                            "callback" : self.set_pnp_target },
            "cartesian" :  { "desc" : "Plans and executes a cartesian path between the given markers.",
                            "args" : ['m1', 'm2', 'm3', 'm4'],
                            "types" : [str, str, str, str],
                            "callback" : self.cartesian },
            "spawn" : { "desc" : "Spawns a shape at the given marker.",
                        "args" : ['marker', 'shape'],
                        "types" : [str, str],
                        "callback" : self.spawn_model },
            "delete" :  { "desc" : "Deletes the shape with the given name.",
                        "args" : ['name'],
                        "types" : [str],
                        "callback" : self.delete_model },
        }

        # Start keyboard handling thread
        console = edo_console.EdoConsole(self.commands)

    def set_home(self):
        home = self.edo_move_group.get_current_joint_values()  
        self.home = home

    def go_home(self):
        # Go to home position and close the gripper
        self.go_to_joint_state(self.home)
        self.set_gripper_span(0.0)
        rospy.loginfo("Executed homing.")

    def go_to_joint_state(self, joint_goal):
        # Pass a joint state for the move group to go to
        self.edo_move_group.go(joint_goal, wait=True)
        # Force the robot to stop moving in case there's residual motion
        self.edo_move_group.stop()
        # Check accuracy
        current_joints = self.edo_move_group.get_current_joint_values()
        return edo_utils.all_close(joint_goal, current_joints, 0.01)

    def go_to_xyz_rpy(self, xyz, rpy):
        pose = self.pose_from_xyz_rpy(xyz, rpy)
        self.go_to_pose_goal(pose)

    def go_to_pose_goal(self, pose_goal):
        self.edo_move_group.set_num_planning_attempts(10)
        self.edo_move_group.allow_replanning(True)
        # Pass a pose for the move group to go to
        self.edo_move_group.set_pose_target(pose_goal)
        self.edo_move_group.go(wait=True)
        # Force the robot to stop moving in case there's residual motion
        self.edo_move_group.stop()
        # Clear the pose target just in case
        self.edo_move_group.clear_pose_targets()
        # Check accuracy
        current_pose = self.edo_move_group.get_current_pose().pose
        return edo_utils.all_close(pose_goal, current_pose, 0.01)

    def go_to_marker(self, marker):
        if marker.upper() not in self.marker_dict:
            rospy.loginfo(f"Unknown marker '{marker}'")
        else:
            xyz = self.marker_dict[marker.upper()]['xyz'].copy()
            rpy = self.marker_dict[marker.upper()]['rpy'].copy()
            pose = self.pose_from_xyz_rpy(xyz, rpy)
            self.go_to_pose_goal(pose)

    def set_joint(self, joint, angle):
        # Set the given joint to the given angle
        joint_goal = self.edo_move_group.get_current_joint_values() 
        joint_goal[joint] = float(angle) * DEG2RAD
        rospy.loginfo(f"New target joint state:\n{edo_utils.round_list(joint_goal)}")
        self.go_to_joint_state(joint_goal)

    def move_joint(self, joint, angle):
        # Move the given joint by the given angle
        joint_goal = self.edo_move_group.get_current_joint_values()   
        joint_goal[joint] += float(angle) * DEG2RAD
        rospy.loginfo(f"New target joint state:\n{edo_utils.round_list(joint_goal)}")
        self.go_to_joint_state(joint_goal)

    def plan_cartesian_path(self, poses):
        waypoints = []
        for pose in poses:
            waypoints.append(copy.deepcopy(pose))
        # Generate a motion plan from the list of poses
        (plan, fraction) = self.edo_move_group.compute_cartesian_path(waypoints, 0.0001, 0.0)
        return plan, fraction

    def execute_plan(self, plan):
        # Execute a precomputed motion plan
        self.edo_move_group.execute(plan, wait=True)

    def set_gripper_span(self, span):
        # Publish the desired gripper width
        msg = Float32()
        msg.data = span
        self.gripper_span_pub.publish(msg)
        rospy.loginfo(f"Setting gripper span to: {span}")

    def wait_for_state_update(self, name, is_known=False, is_attached=False, timeout=4):
        # Wait for the reflection of changes to the known object and attached object lists
        # useful if for some reason the python node dies before publishing the updates
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            attached_objects = self.scene.get_attached_objects([name])
            is_attached = len(attached_objects.keys()) > 0
            is_known = name in self.scene.get_known_object_names()
            if (is_attached == is_attached) and (is_known == is_known):
                return True
            rospy.sleep(0.1)
            seconds = rospy.get_time()
        self.scene.remove_world_object()
        return False
    
    def print_cartesian(self):
        # Print the current pose
        rospy.loginfo(f"Current pose:\n{self.edo_move_group.get_current_pose().pose}")

    def print_rpy(self):
        # Compute xyz
        coords = self.edo_move_group.get_current_pose().pose.position
        # Compute rpy
        orientation = self.edo_move_group.get_current_pose().pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        rpy = [x * RAD2DEG for x in euler_from_quaternion(quaternion)]
        # Print current xyz-rpy of the EE
        rospy.loginfo(f"Current xyz: {[coords.x, coords.y, coords.z]}, current rpy: {rpy}")

    def print_joint(self):
        # Print the current joint values
        rospy.loginfo(f"Current joint state:\n{self.edo_move_group.get_current_joint_values()}")

    def display_trajectory(self, plan):
        # Publish the trajectory for rviz to display
        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)

    def set_pnp_target(self, pick, place):
        pick_cylinder = self.marker_dict[pick.upper()]['xyz'].copy()
        pick_cylinder[2] -= 0.07
        place_cylinder = self.marker_dict[place.upper()]['xyz'].copy()
        place_cylinder[2] -= 0.07
        sphere = self.marker_dict[pick.upper()]['xyz'].copy()
        self.spawn_model(pick_cylinder, 'cylinder')
        self.spawn_model(place_cylinder, 'cylinder')
        self.spawn_model(sphere, 'sphere')

    def pick_and_place(self, pick, place):
        self.pick_target = pick.upper()
        self.place_target = place.upper()
        if self.pick_target not in self.marker_dict:
            rospy.loginfo(f"Argument '{self.pick_target}' is not a known marker.")
        elif self.place_target not in self.marker_dict:
            rospy.loginfo(f"Argument '{self.place_target}' is not a known marker.")
        else:
            rospy.loginfo("Executing pick and place.")
            # Set up waypoints
            pick_approach = self.marker_dict[self.pick_target]['xyz'].copy()
            pick_approach[2] = 0.95
            pick = self.marker_dict[self.pick_target]['xyz'].copy()
            place_approach = self.marker_dict[self.place_target]['xyz'].copy()
            place_approach[2] = 0.95
            place = self.marker_dict[self.place_target]['xyz'].copy()
            # Execute pick and place
            self.go_to_xyz_rpy(pick_approach, self.marker_dict[self.pick_target]['rpy'])
            self.set_gripper_span(0.09)
            rospy.sleep(2)
            self.go_to_xyz_rpy(pick, self.marker_dict[self.pick_target]['rpy'])
            # Close the gripper sligthly less than the real width, helps achieve grasp
            self.set_gripper_span(0.024)
            rospy.sleep(2)
            self.go_to_xyz_rpy(pick_approach, self.marker_dict[self.pick_target]['rpy'])
            rospy.sleep(2)
            self.go_to_xyz_rpy(place_approach, self.marker_dict[self.place_target]['rpy'])
            rospy.sleep(2)
            self.go_to_xyz_rpy(place, self.marker_dict[self.place_target]['rpy'])
            self.set_gripper_span(0.09)
            rospy.sleep(2)
            self.go_to_xyz_rpy(place_approach, self.marker_dict[self.place_target]['rpy'])
            self.go_home()

    def cartesian(self, m1, m2, m3, m4):
        if m1.upper() not in self.marker_dict:
            rospy.loginfo(f"Argument '{m1}' is not a known marker.")
        elif m2.upper() not in self.marker_dict:
            rospy.loginfo(f"Argument '{m2}' is not a known marker.")
        elif m3.upper() not in self.marker_dict:
            rospy.loginfo(f"Argument '{m3}' is not a known marker.")
        elif m4.upper() not in self.marker_dict:
            rospy.loginfo(f"Argument '{m4}' is not a known marker.")
        else:
            poses = [
                self.pose_from_xyz_rpy(self.marker_dict[m2.upper()]['xyz'], self.marker_dict[m2.upper()]['rpy']),
                self.pose_from_xyz_rpy(self.marker_dict[m3.upper()]['xyz'], self.marker_dict[m3.upper()]['rpy']),
                self.pose_from_xyz_rpy(self.marker_dict[m4.upper()]['xyz'], self.marker_dict[m4.upper()]['rpy']),
                self.pose_from_xyz_rpy(self.marker_dict[m1.upper()]['xyz'], self.marker_dict[m1.upper()]['rpy']),
            ]
            self.go_to_pose_goal(poses[3])
            rospy.sleep(1)
            (plan, fraction) = self.plan_cartesian_path(poses)
            if fraction != 1.0:
                rospy.loginfo("No cartesian path exists that passes through the given markers.")
            else:
                self.execute_plan(plan)

    def pose_from_xyz_rpy(self, xyz, rpy):
        # Computes a pose message from the given xyz coords and rpy orientation
        pose = Pose()
        pose.position.x = xyz[0]
        pose.position.y = xyz[1]
        pose.position.z = xyz[2]
        # Compute the quaternion orientation from rpy
        new_orientation = quaternion_from_euler(rpy[0]*DEG2RAD, rpy[1]*DEG2RAD, rpy[2]*DEG2RAD)
        pose.orientation.x = new_orientation[0]
        pose.orientation.y = new_orientation[1]
        pose.orientation.z = new_orientation[2]
        pose.orientation.w = new_orientation[3]
        return pose


    def spawn_model(self, xyz, model, timeout=4):
        if model not in ['box', 'sphere', 'cylinder']:
            rospy.loginfo(f"Invalid shape '{model}' for spawn command. Valid options are: 'box', 'sphere', 'cylinder'.")
        else:
            # Get the package's path
            rospack = rospkg.RosPack()
            urdf_path = os.path.join(rospack.get_path('edo'), 'urdf/' + model + '.urdf')
            # Contact Gazebo's spawn_urdf_model service
            try:
                rospy.wait_for_service("/gazebo/spawn_urdf_model", timeout=timeout)
                spawn_model = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
                with open(urdf_path, "r") as f:
                    urdf = f.read()
                # Check if a marker letter was given
                if type(xyz) == str:
                    xyz = self.marker_dict[xyz.upper()]['xyz']
                self.model_counter += 1
                # Set up the pose for the pickable box
                pose = Pose()
                pose.position.x = xyz[0]
                pose.position.y = xyz[1]
                pose.position.z = xyz[2]
                pose.orientation.x = 0.0
                pose.orientation.y = 0.0
                pose.orientation.z = 0.0
                pose.orientation.w = 1.0
                # Generate a model name
                model_name = model + str(self.model_counter)
                # Spawn the box in the world
                spawn_model(model_name, urdf, "edo", pose, "world")
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = "world"
                pose_stamped.pose = pose
                # Add the model name to the list of spawned models
                self.spawned_models.append(model_name)
                # Add the model to the planning scene
                if model == 'box':
                    self.scene.add_box(model_name, pose_stamped, size=(0.025, 0.025, 0.025))
                elif model == 'sphere':
                    self.scene.add_sphere(model_name, pose_stamped, radius=0.0125)
                elif model == 'cylinder':
                    mesh_path = os.path.join(rospack.get_path('edo'), 'meshes/visual/cylinder.dae')
                    self.scene.add_mesh(name=model_name, pose=pose_stamped, filename=mesh_path, size=(0.4, 0.4, 0.1))
                return self.wait_for_state_update(name=model_name, is_known=True, timeout=timeout)
            except rospy.ROSException:
                rospy.logwarn("Timeout reached while waiting for spawn_urdf_model service.")
                return False

    def delete_model(self, model_name):
        if model_name not in self.spawned_models:
            rospy.loginfo("No model with that name exists in the scene. Models in the scene: " + ' '.join(self.spawned_models))
        else:
            # Contact Gazebo's delete_model service
            try:
                rospy.wait_for_service("/gazebo/delete_model", timeout=4)
                delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
                # Delete the model
                delete_model(model_name)
                # Remove the object from the planning scene
                self.scene.remove_world_object(model_name)
            except rospy.ROSException:
                rospy.logwarn("Timeout reached while waiting for delete_model service.")
            # Remove the object from the list of spawned models
            try:
                self.spawned_models.remove(model_name)
            except ValueError: # This shouldn't really ever happen
                rospy.logwarn("delete_model failed")

    def kill(self):
        # Stop the robot
        self.edo_move_group.stop()
        # Delete spawned models
        rospy.wait_for_service("/gazebo/delete_model", timeout=4)
        delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        for s in self.spawned_models:
            delete_model(s)
            self.scene.remove_world_object(s)
        rospy.loginfo("Stop key received, stopping...")
        # Shutdown rospy and kill the process
        rospy.signal_shutdown("Stop key received.")
        os._exit(os.EX_OK)