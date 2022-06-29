#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander

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

        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link = self.move_group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()
        self.state = self.robot.get_current_state()
        self.box_name = ""
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
        box_pose.pose.position.z = 1.0        
        box_name = "box"
        self.scene.add_box(box_name, box_pose, size=(0.5, 0.5, 0.5))
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=4):
        grasping_group = "edo_gripper"
        touch_links = self.robot.get_link_names(group=grasping_group)
        self.scene.attach_box(self.eef_link, self.box_name, touch_links=touch_links)
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def detach_box(self, timeout=4):
        self.scene.remove_attached_object(self.eef_link, name=self.box_name)
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

    def remove_box(self, timeout=4):
        self.scene.remove_world_object(self.box_name)
        return self.wait_for_state_update(
            box_is_attached=False, box_is_known=False, timeout=timeout
        )

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


def main():
    try:
        mg = EdoMoveGroupInterface()
        mg.go_to_joint_state([-pi/2, pi/4, 0, pi/4, 0, 0])
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()