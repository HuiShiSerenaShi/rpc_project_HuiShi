#!/usr/bin/env python2

import copy
import lzma
#from lzma import MF_HC4
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

    def __init__(self, home=[0, 0, 0, 0, 0, 1.57 / 2]):
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
        self.edo_move_group.set_num_planning_attempts(10)
        self.edo_move_group.allow_replanning(True)

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
        marker_height = 0.89
        self.marker_dict = {
            "A" : { 'xyz' : [ 0.405, 0.295, marker_height], 'rpy' : [173, 6, 168]},
            "B" : { 'xyz' : [ 0.295, 0.405, marker_height], 'rpy' : [173, 6, -174]},
            "C" : { 'xyz' : [ 0.155, 0.475, marker_height], 'rpy' : [173, 6, -155]},
            "D" : { 'xyz' : [ 0.000, 0.500, marker_height], 'rpy' : [173, 6, -138]},
            "E" : { 'xyz' : [ 0.325, 0.235, marker_height], 'rpy' : [180, 0, 180]},
            "F" : { 'xyz' : [ 0.235, 0.325, marker_height], 'rpy' : [180, 0, -170]},
            "G" : { 'xyz' : [ 0.125, 0.380, marker_height], 'rpy' : [180, 0, -152]},
            "H" : { 'xyz' : [ 0.000, 0.400, marker_height], 'rpy' : [180, 0, -134]},
            "I" : { 'xyz' : [ 0.240, 0.175, marker_height], 'rpy' : [-177.32, -4.21, -176.89]},
            "J" : { 'xyz' : [ 0.175, 0.240, marker_height], 'rpy' : [-177.32, -4.21, -176.89]},
            "K" : { 'xyz' : [ 0.090, 0.285, marker_height], 'rpy' : [-177.32, -4.206, -141.293]},
            "L" : { 'xyz' : [ 0.000, 0.300, marker_height], 'rpy' : [-177.325, -4.206, -123.493]},
            "O" : { 'xyz' : [ 0.245, -0.175, marker_height], 'rpy' : [-177.32, -4.21, 111.25]},
            "P" : { 'xyz' : [ 0.180, -0.240, marker_height], 'rpy' : [-177.32, -4.20, 92.84]},
            "Q" : { 'xyz' : [ 0.091, -0.288, marker_height], 'rpy' : [-177, -4.21, 75]},
            "R" : { 'xyz' : [ 0.000, -0.300, marker_height], 'rpy' : [-177, -4.32, 57]},
            "S" : { 'xyz' : [ 0.320, -0.235, marker_height], 'rpy' : [-180, 0, 100]},
            "T" : { 'xyz' : [ 0.235, -0.325, marker_height], 'rpy' : [-180, 0, 82]},
            "U" : { 'xyz' : [ 0.120, -0.380, marker_height], 'rpy' : [-180, 0, 64]},
            "V" : { 'xyz' : [ 0.000, -0.400, marker_height], 'rpy' : [-180, 0, 46]},
            "W" : { 'xyz' : [ 0.405, -0.295, marker_height], 'rpy' : [173, 6, 96]},
            "X" : { 'xyz' : [ 0.295, -0.405, marker_height], 'rpy' : [173, 6, 78]},
            "Y" : { 'xyz' : [ 0.155, -0.475, marker_height], 'rpy' : [173, 6, 60]},
            "Z" : { 'xyz' : [ 0.000, -0.500, marker_height], 'rpy' : [173, 6, 42]},
        }

        # Bring the robot home
        self.go_home()

        # Spawn the cylinders for pick and place
        self.spawn_cylinders()

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
            "set_gripper" :  { "desc" : "Sets the gripper's span to the given width in meters.",
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
            "pnp_target" :  { "desc" : "Spawns a sphere in the given marker.",
                            "args" : ['marker'],
                            "types" : [str],
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

        rospy.loginfo("Initialized edo MoveGroupInterface.")

        # Start keyboard handling thread
        self.console = edo_console.EdoConsole(self.commands)

    def set_home(self):
        # Change the home position
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

    def go_to_xyz_rpy(self, xyz, rpy):
        # Go to a pose defined by xyz coords and rpy orientation
        pose = self.pose_from_xyz_rpy(xyz, rpy)
        self.go_to_pose_goal(pose)

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

    def go_to_marker(self, marker):
        # Bring the robot to the given marker
        if marker.upper() not in self.marker_dict:
            rospy.loginfo("Unknown marker '{}'".format(marker))
        else:
            xyz = self.marker_dict[marker.upper()]['xyz'][:]
            rpy = self.marker_dict[marker.upper()]['rpy'][:]
            pose = self.pose_from_xyz_rpy(xyz, rpy)
            self.go_to_pose_goal(pose)

    def set_joint(self, joint, angle):
        # Set the given joint to the given angle
        joint_goal = self.edo_move_group.get_current_joint_values() 
        joint_goal[joint] = float(angle) * DEG2RAD
        #rospy.loginfo(f"New target joint state:\n{edo_utils.round_list(joint_goal)}")
        rospy.loginfo("New target joint state:\n{}".format(edo_utils.round_list(joint_goal)))

        self.go_to_joint_state(joint_goal)

    def move_joint(self, joint, angle):
        # Move the given joint by the given angle
        joint_goal = self.edo_move_group.get_current_joint_values()   
        joint_goal[joint] += float(angle) * DEG2RAD
        #rospy.loginfo(f"New target joint state:\n{edo_utils.round_list(joint_goal)}")
        rospy.loginfo("New target joint state:\n{}".format(edo_utils.round_list(joint_goal)))
        self.go_to_joint_state(joint_goal)

    def plan_cartesian_path(self, poses):
        waypoints = []
        for pose in poses:
            waypoints.append(copy.deepcopy(pose))
        # Generate a motion plan from the list of poses
        resolution = 0.0001 # Used to generate intermediate waypoints
        jump_threshold = 0.0
        (plan, fraction) = self.edo_move_group.compute_cartesian_path(waypoints, 
                            eef_step=resolution, jump_threshold=jump_threshold)
        #rospy.loginfo(f"Finished computing plan.")
        rospy.loginfo("Finished computing plan.")

        return plan, fraction

    def execute_plan(self, plan):
        # Execute a precomputed motion plan
        self.edo_move_group.execute(plan, wait=True)
        #rospy.loginfo(f"Finished executing plan.")
        rospy.loginfo("Finished executing plan.")


    def set_gripper_span(self, span):
        # Publish the desired gripper width
        msg = Float32()
        msg.data = span
        self.gripper_span_pub.publish(msg)
        #rospy.loginfo(f"Setting gripper span to: {span}")
        rospy.loginfo("Setting gripper span to: {0}".format(span))

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
        #rospy.loginfo(f"Current pose:\n{self.edo_move_group.get_current_pose().pose}")
        rospy.loginfo("Current pose:\n{0}".format(self.edo_move_group.get_current_pose().pose))
    def print_rpy(self):
        # Compute xyz
        coords = self.edo_move_group.get_current_pose().pose.position
        # Compute rpy
        orientation = self.edo_move_group.get_current_pose().pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        rpy = [x * RAD2DEG for x in euler_from_quaternion(quaternion)]
        # Print current xyz-rpy of the EE
        #rospy.loginfo(f"Current xyz: {[coords.x, coords.y, coords.z]}, current rpy: {rpy}")
        rospy.loginfo("Current xyz: [{0}, {1}, {2}], current rpy: {3}".format(coords.x, coords.y, coords.z, rpy))
    def print_joint(self):
        # Print the current joint values
        #rospy.loginfo(f"Current joint state:\n{self.edo_move_group.get_current_joint_values()}")
        rospy.loginfo("Current joint state:\n{0}".format(self.edo_move_group.get_current_joint_values()))
    def display_trajectory(self, plan):
        # Publish the trajectory for rviz to display
        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)

    def set_pnp_target(self, marker):
        if marker.upper() not in self.marker_dict:
            #rospy.loginfo(f"'{marker}' is not a known marker.")
            rospy.loginfo("'{0}' is not a known marker.".format(marker))
        else:
            #rospy.loginfo(f"Spawned a sphere on marker '{marker}'")
            rospy.loginfo("Spawned a sphere on marker '{0}'".format(marker))
            # Spawn a sphere at the given marker
            sphere = self.marker_dict[marker.upper()]['xyz'][:]
            self.spawn_model(sphere, 'sphere')

    def pick_and_place(self, pick, place):
        pick_target = pick.upper()
        place_target = place.upper()
        if pick_target not in self.marker_dict:
            #rospy.loginfo(f"Argument '{pick_target}' is not a known marker.")
            rospy.loginfo("Argument '{0}' is not a known marker.".format(pick_target))
        elif place_target not in self.marker_dict:
            rospy.loginfo("Argument '{0}' is not a known marker.".format(place_target))
            #rospy.loginfo(f"Argument '{place_target}' is not a known marker.")
        else:
            # Check if a sphere does exist at the marker
            model_name = None
            for model in self.spawned_models:
                if model[1] == pick_target:
                    model_name = model[0]
                    break
            for model in self.spawned_models:
                if model[1] == place_target:
                    model_name = 'conflict'
                    break
            if model_name == 'conflict':
                #rospy.loginfo(f"There already is a sphere on marker '{place_target}'.")
                rospy.loginfo("There already is a sphere on marker '{0}'.".format(place_target))
            else:
                rospy.loginfo("Executing pick and place.")
                # Set up waypoints
                pick_approach = self.marker_dict[pick_target]['xyz'][:]
                pick_approach[2] = 0.95
                pick = self.marker_dict[pick_target]['xyz'][:]
                place_approach = self.marker_dict[place_target]['xyz'][:]
                place_approach[2] = 0.95
                place = self.marker_dict[place_target]['xyz'][:]
                # Execute pick and place
                self.go_to_xyz_rpy(pick_approach, self.marker_dict[pick_target]['rpy'])
                self.set_gripper_span(0.09)
                rospy.sleep(2)
                self.go_to_xyz_rpy(pick, self.marker_dict[pick_target]['rpy'])
                # Close the gripper sligthly less than the real width, helps achieve grasp
                self.set_gripper_span(0.023)
                if model_name is not None:
                    # Attach the sphere to the gripper in the planning scene
                    grasping_group = "edo_gripper"
                    touch_links = self.robot.get_link_names(group=grasping_group)
                    self.scene.attach_box(self.eef_link, model_name, touch_links=touch_links)
                    self.wait_for_state_update(name=model_name, is_known=False, is_attached=True, timeout=4)
                rospy.sleep(2)
                self.go_to_xyz_rpy(pick_approach, self.marker_dict[pick_target]['rpy'])
                rospy.sleep(2)
                self.go_to_xyz_rpy(place_approach, self.marker_dict[place_target]['rpy'])
                rospy.sleep(2)
                self.go_to_xyz_rpy(place, self.marker_dict[place_target]['rpy'])
                self.set_gripper_span(0.09)
                if model_name is not None:
                    # Detach the sphere from the gripper in the planning scene
                    self.scene.remove_attached_object(self.eef_link, name=model_name)
                    self.wait_for_state_update(name=model_name, is_known=True, is_attached=False, timeout=4)
                    for model in self.spawned_models:
                        if model[1] == pick_target:
                            model = list(model)
                            break
                    model[1] = place_target
                    # Update the sphere's marker
                    self.spawned_models = [x for x in self.spawned_models if x[0] != model_name]
                    self.spawned_models.append(tuple(model))
                rospy.sleep(2)
                self.go_to_xyz_rpy(place_approach, self.marker_dict[place_target]['rpy'])
                rospy.sleep(2)
                self.go_home()

    def cartesian(self, m1, m2, m3, m4):
        if m1.upper() not in self.marker_dict:
            #rospy.loginfo(f"'{m1}' is not a known marker.")
            rospy.loginfo("'{0}' is not a known marker.".format(m1))
        elif m2.upper() not in self.marker_dict:
            #rospy.loginfo(f"'{m2}' is not a known marker.")
            rospy.loginfo("'{0}' is not a known marker.".format(m2))
        elif m3.upper() not in self.marker_dict:
            #rospy.loginfo(f"'{m3}' is not a known marker.")
            rospy.loginfo("'{0}' is not a known marker.".format(m3))
        elif m4.upper() not in self.marker_dict:
            #rospy.loginfo(f"'{m4}' is not a known marker.")
            rospy.loginfo("'{0}' is not a known marker.".format(m4))
        else:
            # Get the poses from the markers
            poses = [
                self.pose_from_xyz_rpy(self.marker_dict[m2.upper()]['xyz'], self.marker_dict[m2.upper()]['rpy']),
                self.pose_from_xyz_rpy(self.marker_dict[m3.upper()]['xyz'], self.marker_dict[m3.upper()]['rpy']),
                self.pose_from_xyz_rpy(self.marker_dict[m4.upper()]['xyz'], self.marker_dict[m4.upper()]['rpy']),
                self.pose_from_xyz_rpy(self.marker_dict[m1.upper()]['xyz'], self.marker_dict[m1.upper()]['rpy']),
            ]
            # Bring the robot close to the starting point
            self.go_to_pose_goal(poses[3])
            # Plan the path
            (plan, fraction) = self.plan_cartesian_path(poses)
            # Execute the plan only if it has fully been computed
            if fraction != 1.0:
                rospy.loginfo("No cartesian path exists that passes through the given markers.")
            else:
                rospy.loginfo("Executing cartesian path.")
                self.execute_plan(plan)

    def spawn_cylinders(self):
        # Spawn a cylinder at each marker on the surface
        for marker in self.marker_dict:
            #cylinder = self.marker_dict[marker]['xyz'].copy()
            cylinder = self.marker_dict[marker]['xyz'][:]

            cylinder[2] = 0.83
            self.spawn_model(cylinder, 'cylinder')

    def find_marker(self, xyz):
        # Find the marker from a given coordinate
        for marker in self.marker_dict:
            #mxyz = self.marker_dict[marker]['xyz'].copy()
            mxyz = self.marker_dict[marker]['xyz'][:]
            if xyz[0] == mxyz[0] and xyz[1] == mxyz[1]:
                return marker
        return ""

    def spawn_model(self, xyz, model, timeout=4):
        if model not in ['box', 'sphere', 'cylinder']:
            #rospy.loginfo(f"Invalid shape '{model}' for spawn command. Valid options are: 'box', 'sphere', 'cylinder'.")
            rospy.loginfo("Invalid shape '{0}' for spawn command. Valid options are: 'box', 'sphere', 'cylinder'.".format(model))
        elif type(xyz) == str and xyz not in self.marker_dict:
            #rospy.loginfo(f"'{xyz}' is not a known marker.")
            rospy.loginfo("'{0}' is not a known marker.".format(xyz))
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
                    marker = xyz
                    xyz = self.marker_dict[xyz.upper()]['xyz'][:]
                else:
                    marker = self.find_marker(xyz)
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
                # Add the model list of spawned models
                self.spawned_models.append((model_name, marker if model == 'sphere' else ''))
                # Add the model to the planning scene
                if model == 'box':
                    self.scene.add_box(model_name, pose_stamped, size=(0.025, 0.025, 0.025))
                elif model == 'sphere':
                    self.scene.add_sphere(model_name, pose_stamped, radius=0.0125)
                elif model == 'cylinder':
                    mesh_path = os.path.join(rospack.get_path('edo'), 'meshes/visual/cylinder.dae')
                    self.scene.add_mesh(name=model_name, pose=pose_stamped, filename=mesh_path)
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
            self.spawned_models = [x for x in self.spawned_models if x[0] != model_name]

    def kill(self):
        # Stop the robot
        self.edo_move_group.stop()
        # Delete spawned models
        rospy.wait_for_service("/gazebo/delete_model", timeout=4)
        delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        for s in self.spawned_models:
            delete_model(s[0])
            self.scene.remove_world_object(s[0])
        rospy.loginfo("Stop key received, stopping...")
        # Shutdown rospy and kill the process
        rospy.signal_shutdown("Stop key received.")
        os._exit(os.EX_OK)