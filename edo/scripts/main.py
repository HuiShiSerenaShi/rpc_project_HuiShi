#!/usr/bin/env python

import copy
import moveit_commander
import os
import roslib; roslib.load_manifest('urdfdom_py')
import rospkg
import rospy
import sys
import threading

from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, PoseStamped
from math import pi, dist, fabs, cos
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import DisplayTrajectory
from std_msgs.msg import Float32, String
from urdf_parser_py.urdf import URDF

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
        group_name = "edo_gripper"
        self.gripper_move_group = MoveGroupCommander(group_name)
        
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
        self.box_pose_stamped = PoseStamped()
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
        # Pass a joint state for the move group to go to
        self.edo_move_group.go(joint_goal, wait=True)
        # Force the robot to stop moving in case there's residual motion
        self.edo_move_group.stop()
        # Check accuracy
        current_joints = self.edo_move_group.get_current_joint_values()
        return self.all_close(joint_goal, current_joints, 0.01)

    def go_home(self):
        # Go to home position and close the gripper
        self.go_to_joint_state([0, 0, 0, 0, 0, 0])
        self.set_gripper_span(0.0)

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
        # Get the package's path
        rospack = rospkg.RosPack()
        box_urdf_path = os.path.join(rospack.get_path('edo'), 'urdf/box.urdf')
        # COntact Gazebo's spawn_urdf_model service
        rospy.wait_for_service("/gazebo/spawn_urdf_model", timeout=timeout)
        spawn_model = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
        with open(box_urdf_path, "r") as f:
            box_urdf = f.read()
        # Set up the pose for the pickable box
        box_pose = Pose()
        box_pose.position.x = 0.4
        box_pose.position.y = 0.0
        box_pose.position.z = 0.087
        box_pose.orientation.x = 0.0
        box_pose.orientation.y = 0.0
        box_pose.orientation.z = 0.0
        box_pose.orientation.w = 1.0
        # Spawn the box in the world
        spawn_model("box", box_urdf, "edo",   box_pose, "world")
        self.box_pose_stamped = PoseStamped()
        self.box_pose_stamped.header.frame_id = "world"
        self.box_pose_stamped.pose = box_pose
        # Add the box to the planning scene (TODO: needed?)
        self.scene.add_box(self.box_name, self.box_pose_stamped, size=(0.025, 0.025, 0.025))
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=4):
        # Tell the planning scene that the box is attached to the ee (TODO: needed?)
        grasping_group = "edo_gripper"
        touch_links = self.robot.get_link_names(group=grasping_group)
        self.scene.attach_box(self.eef_link, self.box_name, touch_links=touch_links)
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def detach_box(self, timeout=4):
        # Tell the planning scene that the box is detached from the ee (TODO: needed?)
        self.scene.remove_attached_object(self.eef_link, name=self.box_name)
        print(self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout))

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

    def cartesian(self):
        # Plan and execute thhe demonstration cartesian traj
        p1 = Pose()
        p1.position.x = 0.1781872233813927
        p1.position.y = 0.48339276316996005
        p1.position.z = 0.13659942218845122 
        p1.orientation.x = -0.6703908630240593
        p1.orientation.y = 0.7375574153598073
        p1.orientation.z = 0.06883179458404497
        p1.orientation.w = 0.04298062209830564
        p2 = Pose()
        p2.position.x = -0.11934454844131838
        p2.position.y = 0.510805482122449
        p2.position.z = 0.14971139249189303
        p2.orientation.x = -0.6702984352405627
        p2.orientation.y = 0.7376399124800481
        p2.orientation.z = 0.06882664670682606
        p2.orientation.w = 0.04301464789549873
        p = [p1, p2]
        self.go_to_pose_goal(p1)
        (plan, fraction) = self.plan_cartesian_path(p)
        self.execute_plan(plan)

    def pick_and_place(self):
        # Demonstration pick and place trajectory
        pick_approach = [-0.010911190911310875, 0.9517952526784494, 1.0944452943478726, 0.019684740760639308, 1.1300822318508716, 0.6406438530178793]
        pick_joint_state = [-0.01089727417424946, 1.0242467411297334, 1.094712739209256, 0.01984789651120966, 1.0578107180909413, 0.6405495454717895]
        waypoint_joint_state = [0.4206274075846954, 0.3832774887285968, 1.1000825894037467, 0.019617415711204345, 1.057231879761889, 0.6405168420719409]
        place_approach = [1.5869587990376077, 1.257392829347598, 0.3341822557692531, 0.021794601966520055, 1.4828269594223187, 0.7675031198387039]
        place_joint_state = [1.5858523728679126, 1.4603561921661043, 0.10485557931239509, 0.02074594471888247, 1.5325062824100204, 0.7665689171974392]
        wait_between_waypoints = 1
        self.go_home()
        rospy.sleep(wait_between_waypoints)
        self.go_to_joint_state(pick_approach if not self.pnp_executed else place_approach)
        self.set_gripper_span(0.9)
        rospy.sleep(wait_between_waypoints)
        self.go_to_joint_state(pick_joint_state if not self.pnp_executed else place_joint_state)
        self.set_gripper_span(0.02)
        rospy.sleep(wait_between_waypoints)
        self.go_to_joint_state(waypoint_joint_state)
        rospy.sleep(wait_between_waypoints)
        self.go_to_joint_state(place_approach if not self.pnp_executed else pick_approach)
        rospy.sleep(wait_between_waypoints)
        self.go_to_joint_state(place_joint_state if not self.pnp_executed else pick_joint_state)
        self.set_gripper_span(0.9)
        rospy.sleep(wait_between_waypoints)
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
        print("q - Close the gripper")
        print("e - Open the gripper")
        print("h - Go to home position")
        print("pj - Print current joint state")
        print("pc - Print current pose")
        print("r - Execute demonstration pick and place trajectory")
        print("t - Execute demonstration cartesian trajectory")
        print("---------------------------------------------")
        while True:
            m = input("> ").lower()
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
                joint_goal = self.edo_move_group.get_current_joint_values()      
                joint_goal[self.current_joint] -= self.joint_step
                rospy.loginfo(f"New target joint state:\n{self.round_list(joint_goal)}")
                self.go_to_joint_state(joint_goal)
            elif m == 'd':
                # Increment current joint value
                joint_goal = self.edo_move_group.get_current_joint_values()      
                joint_goal[self.current_joint] += self.joint_step
                rospy.loginfo(f"New target joint state:\n{self.round_list(joint_goal)}")
                self.go_to_joint_state(joint_goal)
            elif m == 'h':
                # Go to home position
                self.go_home()
                rospy.loginfo("Executed homing.")
            elif m == 'pj':
                # Print joint state
                rospy.loginfo(f"Current joint state:\n{self.edo_move_group.get_current_joint_values()}")
            elif m == 'pc':
                # Print pose
                rospy.loginfo(f"Current pose:\n{self.edo_move_group.get_current_pose()}")
            elif m == 'r':
                # Execute pick and place
                rospy.loginfo(f"Executing demonstration pick and place trajectory.")
                self.pick_and_place() 
            elif m == 't':
                # Execute cartesian trajectory
                rospy.loginfo(f"Executing demonstration cartesian trajectory.")
                self.cartesian()
            elif m == 'q':
                # Close the gripper
                rospy.loginfo(f"Closing the gripper.")
                self.set_gripper_span(0.025)
            elif m == 'e':
                # Open the gripper
                rospy.loginfo(f"Opening the gripper.")
                self.set_gripper_span(0.9)
            elif m == 'k':
                # Stop key received, kill everything
                self.edo_move_group.stop()
                rospy.loginfo("Stop key received, stopping...")
                rospy.signal_shutdown("Stop key received.")
                os._exit(os.EX_OK)
            else:
                rospy.loginfo("Unknown key received.")

def main():
    try:
        # Initialize move group
        mg = EdoMoveGroupInterface()
        # Execute homing
        mg.go_home()
        mg.add_box()
        # Start keyboard handling thread
        th = threading.Thread(target=mg.handle_keyboard)
        th.start()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()