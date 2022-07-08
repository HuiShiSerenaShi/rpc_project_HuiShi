#!/usr/bin/env python3

import copy
import moveit_commander
import os
import roslib; roslib.load_manifest('urdfdom_py')
import rospkg
import rospy
import sys
import threading
import edo_utils

from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, PoseStamped
from math import asin, atan2, sqrt
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from moveit_msgs.msg import DisplayTrajectory
from std_msgs.msg import Float32
from tf.transformations import quaternion_from_euler, euler_from_quaternion

PI = 3.14159265
DEG2RAD = PI / 180.0
RAD2DEG = 180.0 / PI

class EdoMoveGroupInterface(object):

    def __init__(self) -> None:
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
        self.planning_frame = self.edo_move_group.get_planning_frame()
        self.eef_link = self.edo_move_group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()
        self.state = self.robot.get_current_state()
        self.box_name = "box"
        # Keep track of spawned models
        self.spawned_models = []
        self.model_counter = 0

        # X-Y coords of the marked circles in the workspace
        self.marker_dict = {
            "A" : [ 0.405, 0.295],
            "B" : [ 0.295, 0.405],
            "C" : [ 0.155, 0.475],
            "D" : [ 0.000, 0.500],
            "E" : [ 0.325, 0.235],
            "F" : [ 0.235, 0.325],
            "G" : [ 0.125, 0.380],
            "H" : [ 0.000, 0.400],
            "I" : [ 0.240, 0.175],
            "J" : [ 0.175, 0.240],
            "K" : [ 0.090, 0.285],
            "L" : [ 0.000, 0.300],
            "O" : [ 0.245, -0.175],
            "P" : [ 0.180, -0.240],
            "Q" : [ 0.095, -0.285],
            "R" : [ 0.000, -0.300],
            "S" : [ 0.320, -0.235],
            "T" : [ 0.235, -0.325],
            "U" : [ 0.120, -0.380],
            "V" : [ 0.000, -0.400],
            "W" : [ 0.405, -0.295],
            "X" : [ 0.295, -0.405],
            "Y" : [ 0.155, -0.475],
            "Z" : [ 0.000, -0.500],
        }

        # Pick and place targets
        self.pick_target = "A"
        self.place_target = "Z"
        # Cartesian traj targets
        self.cartesian_targets = ["A", "B", "F", "E"]
        
        # Bring the robot home
        self.go_home()

        self.go_to_xyz_rpy(self.marker_dict["E"], 1.0, [-179, -0.02, 171.0])
        rospy.sleep(3)
        self.go_to_xyz_rpy(self.marker_dict["F"], 1.0, [-179, -0.02, 189.0])
        rospy.sleep(3)
        self.go_to_xyz_rpy(self.marker_dict["G"], 1.0, [-179, -0.02, 207.0])
        rospy.sleep(3)
        self.go_to_xyz_rpy(self.marker_dict["H"], 1.0, [-179, -0.02, 225.0])
        rospy.sleep(3)
        self.go_to_xyz_rpy(self.marker_dict["S"], 1.0, [-179, -0.02, 99.0])
        rospy.sleep(3)
        self.go_to_xyz_rpy(self.marker_dict["T"], 1.0, [-179, -0.02, 81.0])
        rospy.sleep(3)
        self.go_to_xyz_rpy(self.marker_dict["U"], 1.0, [-179, -0.02, 63.0])
        rospy.sleep(3)
        self.go_to_xyz_rpy(self.marker_dict["V"], 1.0, [-179, -0.02, 45.0])
        rospy.loginfo("Initialized edo MoveGroupInterface.")

        # Start keyboard handling thread
        th = threading.Thread(target=self.handle_keyboard)
        th.start()


    def go_home(self):
        # Go to home position and close the gripper
        self.go_to_joint_state([0, 0, 0, 0, 0, 1.57 / 2])
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

    def go_to_xyz_rpy(self, xy, z, rpy):
        pose = Pose()
        pose.position.x = xy[0]
        pose.position.y = xy[1]
        pose.position.z = z
        new_orientation = quaternion_from_euler(rpy[0]*DEG2RAD, rpy[1]*DEG2RAD, rpy[2]*DEG2RAD)
        pose.orientation.x = new_orientation[0]
        pose.orientation.y = new_orientation[1]
        pose.orientation.z = new_orientation[2]
        pose.orientation.w = new_orientation[3]
        # Pass a pose for the move group to go to
        #self.edo_move_group.set_pose_target(pose)
        self.edo_move_group.set_joint_value_target(pose, True)
        self.edo_move_group.go(wait=True)
        # Force the robot to stop moving in case there's residual motion
        self.edo_move_group.stop()
        # Clear the pose target just in case
        self.edo_move_group.clear_pose_targets()
        # Check accuracy
        current_pose = self.edo_move_group.get_current_pose().pose
        return edo_utils.all_close(pose, current_pose, 0.01)

    def go_to_pose_goal(self, pose_goal):
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

    def plan_cartesian_path(self, poses):
        waypoints = []
        for pose in poses:
            waypoints.append(copy.deepcopy(pose))
        # Generate a motion plan from the list of poses
        (plan, fraction) = self.edo_move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
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

    def display_trajectory(self, plan):
        # Publish the trajectory for rviz to display
        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)

    def pick_and_place(self):
        rospy.loginfo("Executing pick and place.")
        self.go_to_xyz_rpy(self.pick_target, 0.9, [0.0, 0.0 , -180 * DEG2RAD])
        self.set_gripper_span(0.09)
        rospy.sleep(2)
        self.go_to_xyz_rpy(self.pick_target, 0.85, [0.0, 0.0 , -180 * DEG2RAD])
        self.set_gripper_span(0.025)
        rospy.sleep(2)
        self.go_to_xyz_rpy(self.place_target, 0.9, [0.0, 0.0 , -180 * DEG2RAD])
        rospy.sleep(2)
        self.go_to_xyz_rpy(self.place_target, 0.85, [0.0, 0.0 , -180 * DEG2RAD])
        self.set_gripper_span(0.09)
        rospy.sleep(2)
        self.go_home()
    
    def test_markers(self):
        for key in self.marker_dict:
            self.go_to_xyz_rpy(self.marker_dict[key], 1.0, [-180.0, 0.0, 0.0])
            rospy.sleep(1)

    def spawn_model(self, xy, z, rpy, model="box", model_name = "", timeout=4):
        # Get the package's path
        rospack = rospkg.RosPack()
        urdf_path = os.path.join(rospack.get_path('edo'), 'urdf/' + model + '.urdf')
        # Contact Gazebo's spawn_urdf_model service
        rospy.wait_for_service("/gazebo/spawn_urdf_model", timeout=timeout)
        spawn_model = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
        with open(urdf_path, "r") as f:
            urdf = f.read()
        # Check if a marker letter was given
        if type(xy) == str:
            xy = self.marker_dict[xy]
        self.model_counter += 1
        # Set up the pose for the pickable box
        pose = Pose()
        pose.position.x = xy[0]
        pose.position.y = xy[1]
        pose.position.z = z
        quaternion = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        if model_name == "":
            model_name = model + str(self.model_counter)
        # Spawn the box in the world
        spawn_model(model_name, urdf, "edo", pose, "world")
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "world"
        pose_stamped.pose = pose
        # Add the model name to the list of spawned models
        self.spawned_models.append(model_name)
        # Add the box to the planning scene
        self.scene.add_box(model_name, pose_stamped, size=(0.025, 0.025, 0.025))
        return self.wait_for_state_update(name=model_name, is_known=True, timeout=timeout)

    def delete_model(self, model_name):
        rospy.wait_for_service("/gazebo/delete_model", timeout=4)
        delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        delete_model(model_name)
        self.scene.remove_world_object(model_name)

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
    
    def print_state(self, state):
        if state == 'cartesian':
            rospy.loginfo(f"Current pose:\n{self.edo_move_group.get_current_pose().pose}")
        elif state == 'rpy':
            orientation = self.edo_move_group.get_current_pose().pose.orientation
            quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
            rpy = [x * RAD2DEG for x in euler_from_quaternion(quaternion)]
            rospy.loginfo(f"Current rpy: {rpy}")
        elif state == 'joint':
            rospy.loginfo(f"Current joint state:\n{self.edo_move_group.get_current_joint_values()}")
        else:
            rospy.logwarn("Invalid state for print_state function.")

    def set_joint(self, joint, angle, absolute):
        joint_goal = self.edo_move_group.get_current_joint_values()      
        if absolute:
            joint_goal[joint] = float(angle) * DEG2RAD
        else:
            joint_goal[joint] += float(angle) * DEG2RAD
        rospy.loginfo(f"New target joint state:\n{edo_utils.round_list(joint_goal)}")
        self.go_to_joint_state(joint_goal)


    def handle_keyboard(self):
        print("----------------------------------------------------------------------------------")
        print(" List of commands: ")
        print("----------------------------------------------------------------------------------")
        print("kill - Kill the execution")
        print("set_joint n a - Sets joint n at a degrees")
        print("move_joint n a - Moves joint n by a degrees")
        print("set_gripper s - Sets the gripper's span to s millimeters")
        print("home - Moves the robot to the home position")
        print("print_joint - Prints the current joint state")
        print("print_cartesian - Prints the current pose")
        print("print_rpy - Prints the current rpy orientation of the EE")
        print("set_pick_target 'marker' - Sets the target marker for the pick")
        print("set_place_target 'marker' - Sets the target marker for the place")
        print("pnp 'markerA' 'markerB'- Executes pick and place between markerA and markerB")
        print("spawn shape marker name - Spawns the shape at the given marker with the given name")
        print("delete name - Deletes the shape with the given name")
        print("----------------------------------------------------------------------------------")
        while True:
            m = input("> ").lower()
            m = m.split(" ")
            if m[0] == 'set_joint' or m[0] == 's':
                # Set the given joint to the given angle
                if len(m) == 3:
                    self.set_joint(int(m[1]), float(m[2]), True)
                else:
                    rospy.loginfo("Wrong argument number for set_joint command.")
            elif m[0] == 'move_joint' or m[0] == 'm':
                # Move the given joint by the given angle
                if len(m) == 3:
                    self.set_joint(int(m[1]), float(m[2]), False)
                else:
                    rospy.loginfo("Wrong argument number for move_joint command.")
            elif m[0] == 'home':
                # Go to home position
                self.go_home()
            elif m[0] == 'print_joint' or m[0] == 'j':
                # Print joint state
                self.print_state('joint')
            elif m[0] == 'print_cartesian' or m[0] == 'c':
                # Print pose
                self.print_state('cartesian')
            elif m[0] == 'print_rpy':
                # Print rpy
                self.print_state('rpy')
            elif m[0] == 'set_gripper' or m[0] == 'g':
                # Set the span of the gripper
                if len(m) == 2:
                    self.set_gripper_span(float(m[1]))
                else:
                    rospy.loginfo("Wrong argument number for set_gripper command.")
            elif m[0] == 'kill' or m[0] == 'k':
                # Stop key received, kill everything
                self.kill()
            elif m[0] == 'set_pick_target':
                if len(m) == 2:
                    self.pick_target = m[1].upper()
                else:
                    rospy.loginfo("Wrong argument number for set_pick_target command.")
            elif m[0] == 'set_place_target':
                if len(m) == 2:
                    self.place_target = m[1].upper()
                else:
                    rospy.loginfo("Wrong argument number for set_place_target command.")
            elif m[0] == 'pnp':
                if len(m) == 3:
                    self.pick_target = m[1].upper()
                    self.place_target = m[2].upper()
                    self.pick_and_place()
                else:
                    rospy.loginfo("Wrong argument number for pnp command.")
            elif m[0] == 'spawn':
                if len(m) != 4:
                    rospy.loginfo("Wrong argument number for spawn command.")
                elif m[1] not in ['box', 'sphere', 'cilinder']:
                    rospy.loginfo(f"Invalid shape '{m[1]}' for spawn command. Valid options are: 'box', 'sphere', 'cilinder'.")
                elif m[2].upper() not in self.marker_dict:
                    rospy.loginfo(f"Unknown marker '{m[1]}' for spawn command.")
                else:
                    self.spawn_model(m[2].upper(), 0.8425, [0, 0, 0], m[1], model_name=m[3])
            else:
                rospy.loginfo(f"Unknwown command '{m[0]}'")