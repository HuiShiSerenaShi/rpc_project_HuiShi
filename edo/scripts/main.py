#!/usr/bin/env python

import copy
import moveit_commander
import os
import rospy
import sys
import threading


from geometry_msgs.msg import Pose, PoseStamped
from math import pi, dist, fabs, cos
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import DisplayTrajectory
from std_msgs.msg import String

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
        self.move_group = MoveGroupCommander(group_name)

        # Publishes the ghost trajectory for Rviz
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            DisplayTrajectory,
            queue_size=20,
        )

        # Misc class attributes
        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link = self.move_group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()
        self.state = self.robot.get_current_state()
        self.box_name = "box"
        # Keeps track of where to pick the demonstration object
        self.pnp_executed = False
        # Current joint (0-6) for manual control
        self.current_joint = 0
        # Step for manual control, approx 1 deg
        self.joint_step = 0.018
        rospy.loginfo("Initialized edo MoveGroupInterface:")
        rospy.loginfo(f"Planning frame: {self.planning_frame}")
        rospy.loginfo(f"End-effector link: {self.eef_link}")
        rospy.loginfo(f"Group names: {self.group_names}")
        rospy.loginfo(f"Current state:\n{self.state}")

    def go_to_joint_state(self, joint_goal):
        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()
        current_joints = self.move_group.get_current_joint_values()
        return self.all_close(joint_goal, current_joints, 0.01)

    def go_home(self):
        self.go_to_joint_state([0, 0, 0, 0, 0, 0])

    def go_to_pose_goal(self, pose_goal):
        self.move_group.set_pose_target(pose_goal)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        current_pose = self.move_group.get_current_pose().pose
        return self.all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self, poses):
        # TODO: test if we can directly pass poses to compute_cartesian_path
        waypoints = []
        for pose in poses:
            waypoints.append(copy.deepcopy(pose))
        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        return plan, fraction

    def display_trajectory(self, plan):
        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):
        self.move_group.execute(plan, wait=True)

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        # Wait for the reflection of changes to the known object and attached object lists
        # useful if for some reason the python node dies before publishing the updates
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            attached_objects = self.scene.get_attached_objects([self.box_name])
            is_attached = len(attached_objects.keys()) > 0
            is_known = self.box_name in self.scene.get_known_object_names()
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True
            rospy.sleep(0.1)
            seconds = rospy.get_time()
        return False

    def add_box(self, timeout=4):
        box_pose = PoseStamped()
        box_pose.header.frame_id = self.planning_frame
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = 0.0
        box_pose.pose.position.y = 0.4
        box_pose.pose.position.z = 0.087
        self.scene.add_box(self.box_name, box_pose, size=(0.075, 0.075, 0.075))
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=4):
        grasping_group = "edo_gripper"
        touch_links = self.robot.get_link_names(group=grasping_group)
        self.scene.attach_box(self.eef_link, self.box_name, touch_links=touch_links)
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def detach_box(self, timeout=4):
        self.scene.remove_attached_object(self.eef_link, name=self.box_name)
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

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
        # Demonstration pick and place trajectory
        pick_approach = [1.528, 0.73, 1.267, 0.091, 1.078, 0.719]
        pick_joint_state = [1.528, 0.907, 1.265, 0.091, 0.899, 0.719]
        waypoint_joint_state = [0.54, 1.083, 0.102, -0.0, 0.989, 0.63]
        place_approach = [-0.001, 0.905, 1.083, -0.0, 1.17, -0.0]
        place_joint_state = [-0.001, 1.176, 0.814, 0.0, 1.169, 0.0]
        self.go_home()
        rospy.sleep(3)
        self.go_to_joint_state(pick_approach if not self.pnp_executed else place_approach)
        rospy.sleep(3)
        self.go_to_joint_state(pick_joint_state if not self.pnp_executed else place_joint_state)
        self.attach_box()
        rospy.sleep(3)
        self.go_to_joint_state(waypoint_joint_state)
        rospy.sleep(3)
        self.go_to_joint_state(place_approach if not self.pnp_executed else pick_approach)
        rospy.sleep(3)
        self.go_to_joint_state(place_joint_state if not self.pnp_executed else pick_joint_state)
        self.detach_box()
        rospy.sleep(3)
        self.go_home()
        self.pnp_executed = not self.pnp_executed

    def handle_keyboard(self):
        print("---------------------------------------------")
        print("Initialized manual control. List of commands:")
        print("---------------------------------------------")
        print("k - Kill the execution")
        print("w - Select previous joint")
        print("s - Select next joint")
        print(f"a - Decrement the selected joint by {self.joint_step} rad")
        print(f"d - Increment the selected joint by {self.joint_step} rad")
        print("q - Close the gripper (not yet implemented)")
        print("e - Open the gripper (not yet implemented)")
        print("h - Go to home position")
        print("p - Print current joint state")
        print("r - Execute demonstration pick and place trajectory")
        print("t - Execute demonstration cartesian trajectory")
        print("---------------------------------------------")
        while True:
            menu = input().lower()
            for m in menu:
                if m == 'w':
                    # Select previous joint
                    self.current_joint = 5 if self.current_joint == 0 else self.current_joint - 1
                    rospy.loginfo(f"Joint {self.current_joint} selected.")
                elif m == 's':
                    # Select next joint
                    self.current_joint = 0 if self.current_joint == 5 else self.current_joint + 1     
                    rospy.loginfo(f"Joint {self.current_joint} selected.")
                elif m == 'a':
                    # Decrement current joint value
                    joint_goal = self.move_group.get_current_joint_values()      
                    joint_goal[self.current_joint] -= self.joint_step
                    rospy.loginfo(f"New target joint state:\n{self.round_list(joint_goal)}")
                    self.go_to_joint_state(joint_goal)
                elif m == 'd':
                    # Increment current joint value
                    joint_goal = self.move_group.get_current_joint_values()      
                    joint_goal[self.current_joint] += self.joint_step
                    rospy.loginfo(f"New target joint state:\n{self.round_list(joint_goal)}")
                    self.go_to_joint_state(joint_goal)
                elif m == 'h':
                    # Go to home position
                    self.go_home()
                    rospy.loginfo("Executed homing.")
                elif m == 'p':
                    # Print joint state
                    rospy.loginfo(f"Current joint state:\n{self.move_group.get_current_joint_values()}")
                elif m == 'r':
                    # Execute pick and place
                    rospy.loginfo(f"Executing demonstration pick and place trajectory.")
                    self.pick_and_place() 
                elif m == 't':
                    # Execute cartesian trajectory
                    rospy.loginfo(f"Executing demonstration cartesian trajectory.")
                elif m == 'q':
                    # Close the gripper
                    rospy.loginfo(f"Closing the gripper.")
                elif m == 'e':
                    # Open the gripper
                    rospy.loginfo(f"Opening the gripper.")
                elif m == 'k':
                    # Stop key received, kill everything
                    self.move_group.stop()
                    rospy.loginfo("Stop key received, stopping...")
                    rospy.signal_shutdown("Stop key received.")
                    os._exit(os.EX_OK)
                else:
                    rospy.loginfo("Unknown key received.")

def main():
    try:
        # Initialize move group
        mg = EdoMoveGroupInterface()
        mg.add_box()
        # Execute homing
        mg.go_home()
        # Start keyboard handling thread
        th = threading.Thread(target=mg.handle_keyboard)
        th.start()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()