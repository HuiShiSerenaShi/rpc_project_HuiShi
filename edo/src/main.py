#!/usr/bin/env python

import copy
import moveit_commander
import os
import roslib; roslib.load_manifest('urdfdom_py')
import rospkg
import rospy
import sys
import threading

from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, PoseStamped
from math import pi, dist, fabs, cos
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import DisplayTrajectory
from std_msgs.msg import Float32
from tf.transformations import quaternion_from_euler, euler_from_quaternion

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
        # Keeps track of where to pick the demonstration object
        self.pnp_executed = False
        # Index of spawned boxes
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
            "M" : [ 0.400, 0.000]
        }

        # For pick and place demonstration
        self.pick_target = "M"
        self.place_target = "F"

        rospy.loginfo("Initialized edo MoveGroupInterface:")
        rospy.loginfo(f"Planning frame: {self.planning_frame}")
        rospy.loginfo(f"End-effector link: {self.eef_link}")
        rospy.loginfo(f"Group names: {self.group_names}")
        rospy.loginfo(f"Current state:\n{self.state}")

    def go_to_joint_state(self, joint_goal):
        # Pass a joint state for the move group to go to
        self.edo_move_group.go(joint_goal, wait=True)
        # Force the robot to stop moving in case there's residual motion
        self.edo_move_group.stop()
        # Check accuracy
        current_joints = self.edo_move_group.get_current_joint_values()
        return self.all_close(joint_goal, current_joints, 0.01)

    def go_home(self):
        # Go to home position and close the gripper
        self.go_to_joint_state([0, 0, 0, 0, 0, 1.57 / 2])
        self.set_gripper_span(0.0)

    def go_to_xyz_rpy(self, xy, z, rpy):
        if type(xy) == str:
            xy = self.marker_dict[xy]
        pose = Pose()
        pose.position.x = xy[0]
        pose.position.y = xy[1]
        pose.position.z = z
        quaternion = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        # Pass a pose for the move group to go to
        self.edo_move_group.set_pose_target(pose)
        self.edo_move_group.go(wait=True)
        # Force the robot to stop moving in case there's residual motion
        self.edo_move_group.stop()
        # Clear the pose target just in case
        self.edo_move_group.clear_pose_targets()
        # Check accuracy
        current_pose = self.edo_move_group.get_current_pose().pose
        return self.all_close(pose, current_pose, 0.01)

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
        return self.all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self, poses):
        # TODO: test if this works
        # TODO: test if we can directly pass poses to compute_cartesian_path
        waypoints = []
        for pose in poses:
            waypoints.append(copy.deepcopy(pose))
        # Generate a motion plan from the list of poses
        (plan, fraction) = self.edo_move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        return plan, fraction

    def display_trajectory(self, plan):
        # Publish the trajectory for rviz to display
        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):
        # Execute a precomputed motion plan
        self.edo_move_group.execute(plan, wait=True)

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

    def set_gripper_span(self, span):
        # Publish the desired gripper width
        msg = Float32()
        msg.data = span
        self.gripper_span_pub.publish(msg)
        # Ensures that the gripper actually finishes moving
        # TODO: this would best be implemented as an action rather than a sleep
        rospy.sleep(1)

    def all_close(self, goal, actual, tolerance):
        """
        Convenience method for testing if the values in two lists are within a tolerance of each other.
        For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
        between the identical orientations q and -q is calculated correctly).
        @param: goal       A list of floats, a Pose or a PoseStamped
        @param: actual     A list of floats, a Pose or a PoseStamped
        @param: tolerance  A float
        @returns: bool
        """
        if type(goal) is list:
            for index in range(len(goal)):
                if abs(actual[index] - goal[index]) > tolerance:
                    return False

        elif type(goal) is PoseStamped:
            return self.all_close(goal.pose, actual.pose, tolerance)

        elif type(goal) is Pose:
            x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
            x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
            # Euclidean distance
            d = dist((x1, y1, z1), (x0, y0, z0))
            # phi = angle between orientations
            cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
            return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

        return True
    
    def round_list(self, list, digits=3):
        # Utility method to round all the elements of a list
        return [round(l, digits) for l in list]

    def pick_and_place(self):
        deg_to_rad = pi / 180.0
        self.go_to_xyz_rpy(self.pick_target, 1.653, [180 * deg_to_rad, 0.0 , -45 * deg_to_rad])
        self.set_gripper_span(0.09)
        rospy.sleep(2)
        self.go_to_xyz_rpy(self.pick_target, 1.625, [180 * deg_to_rad, 0.0 , -45 * deg_to_rad])
        self.set_gripper_span(0.0245)
        rospy.sleep(2)
        self.go_to_xyz_rpy(self.place_target, 1.65, [180 * deg_to_rad, 0.0 , -45 * deg_to_rad])
        rospy.sleep(2)
        self.go_to_xyz_rpy(self.place_target, 1.625, [180 * deg_to_rad, 0.0 , 0.0])
        self.set_gripper_span(0.09)
        rospy.sleep(2)
        self.go_home()

    def spawn_model(self, xy, z, rpy, model="box", timeout=4):
        # Get the package's path
        rospack = rospkg.RosPack()
        urdf_path = os.path.join(rospack.get_path('edo'), 'urdf/' + model + '.urdf')
        # Contact Gazebo's spawn_urdf_model service
        rospy.wait_for_service("/gazebo/spawn_urdf_model", timeout=timeout)
        spawn_model = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
        with open(urdf_path, "r") as f:
            urdf = f.read()
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
        # Spawn the box in the world
        spawn_model(model + str(self.model_counter), urdf, "edo", pose, "world")
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "world"
        pose_stamped.pose = pose
        # Add the box to the planning scene
        self.scene.add_box(model + str(self.model_counter), pose_stamped, size=(0.025, 0.025, 0.025))
        return self.wait_for_state_update(name=model + str(self.model_counter), is_known=True, timeout=timeout)

    def handle_keyboard(self):
        print("---------------------------------------------")
        print("Initialized manual control. List of commands:")
        print("---------------------------------------------")
        print("kill - Kill the execution")
        print("set_joint n a - Sets joint n at a degrees")
        print("move_joint n a - Moves joint n by a degrees")
        print("home - Moves the robot to the home position")
        print("print_joint - Prints the current joint state")
        print("set_gripper s - Sets the gripper's span to s millimeters")
        print("print_cartesian - Prints the current pose")
        print("set_pick_target 'marker' - Sets the target marker for the pick")
        print("set_place_target 'marker' - Sets the target marker for the place")
        print("pnp - Executes pick and place")
        print("---------------------------------------------")
        while True:
            m = input("> ").lower()
            m = m.split(" ")
            if m[0] == 'set_joint' or m[0] == 's':
                if len(m) == 3:
                    joint_goal = self.edo_move_group.get_current_joint_values()      
                    joint_goal[int(m[1])] = float(m[2]) * pi/180.0
                    rospy.loginfo(f"New target joint state:\n{self.round_list(joint_goal)}")
                    self.go_to_joint_state(joint_goal)
                else:
                    rospy.loginfo("Wrong argument number for set_joint command.")
            elif m[0] == 'move_joint' or m[0] == 'm':
                if len(m) == 3:
                    joint_goal = self.edo_move_group.get_current_joint_values()      
                    joint_goal[int(m[1])] += float(m[2]) * pi/180.0
                    rospy.loginfo(f"New target joint state:\n{self.round_list(joint_goal)}")
                    self.go_to_joint_state(joint_goal)
                else:
                    rospy.loginfo("Wrong argument number for move_joint command.")
            elif m[0] == 'home':
                # Go to home position
                self.go_home()
                rospy.loginfo("Executed homing.")
            elif m[0] == 'print_joint' or m[0] == 'j':
                # Print joint state
                rospy.loginfo(f"Current joint state:\n{self.edo_move_group.get_current_joint_values()}")
            elif m[0] == 'print_cartesian' or m[0] == 'c':
                # Print pose
                rospy.loginfo(f"Current pose:\n{self.edo_move_group.get_current_pose().pose}")
            elif m[0] == 'print_rpy':
                # Print rpy
                orientation = self.edo_move_group.get_current_pose().pose.orientation
                quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
                rpy = euler_from_quaternion(quaternion)
                rpyy = [x*180.0/pi for x in rpy]
                rospy.loginfo(f"Current rpy: {rpyy}")
            elif m[0] == 'set_gripper' or m[0] == 'g':
                if len(m) == 2:
                    # Close the gripper
                    rospy.loginfo(f"Setting gripper span {m[1]}")
                    self.set_gripper_span(float(m[1]))
                else:
                    rospy.loginfo("Wrong argument number for set_gripper command.")
            elif m[0] == 'kill' or m[0] == 'k':
                # Stop key received, kill everything
                self.edo_move_group.stop()
                rospy.wait_for_service("/gazebo/delete_model", timeout=4)
                delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
                for i in range(1, self.model_counter + 1):
                    delete_model("box" + str(i))
                    self.scene.remove_world_object("box" + str(i))
                    delete_model("sphere" + str(i))
                    self.scene.remove_world_object("sphere" + str(i))
                rospy.loginfo("Stop key received, stopping...")
                rospy.signal_shutdown("Stop key received.")
                os._exit(os.EX_OK)
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
                rospy.loginfo("Executing pick and place.")
                self.pick_and_place()

def main():
    try:
        # Initialize move group
        mg = EdoMoveGroupInterface()
        # Execute homing
        mg.go_home()
        mg.spawn_model(mg.pick_target, 1.63, [0, 0, 0])
        # Start keyboard handling thread
        th = threading.Thread(target=mg.handle_keyboard)
        th.start()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()