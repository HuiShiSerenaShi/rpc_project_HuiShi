import rospy

from math import pi
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float64, Int8

class EdoGripperControl(object):

    def __init__(self) -> None:
        super(EdoGripperControl, self).__init__()
        rospy.init_node("edo_gripper_node", anonymous=True)
        # Node subscribers
        self.gripper_span_sub = rospy.Subscriber("set_gripper_span", Float32, self.on_set_gripper_span_msg)
        self.joint_state_sub = rospy.Subscriber("joint_states", JointState, self.on_joint_state_msg)
        # Node publishers
        self.gripper_span_pub = rospy.Publisher("edo_gripper_span", Float32, queue_size=10)
        self.gripper_state_pub = rospy.Publisher("edo_gripper_state", Int8, queue_size=10)
        self.left_base_pub = rospy.Publisher("edo_gripper_left_base_controller/command", Float64, queue_size=10)
        self.left_finger_pub = rospy.Publisher("edo_gripper_left_finger_controller/command", Float64, queue_size=10)
        self.right_base_pub = rospy.Publisher("edo_gripper_right_base_controller/command", Float64, queue_size=10)
        self.right_finger_pub = rospy.Publisher("edo_gripper_right_finger_controller/command", Float64, queue_size=10)
        # Node attributes
        self.desired_span = 0.0
        self.current_span = 0.0
        self.gripper_moving = False
        rospy.loginfo("Initialized EdoGripperControl.")

    def update(self):
        # Actually publish the desired span to the gripper topic
        msg = Float32()
        msg.data = self.current_span
        self.gripper_span_pub.publish(msg)
        # This keeps track of the state of the gripper (TODO: this is useless unless implemented via an action)
        self.gripper_moving = abs(self.current_span - self.desired_span) > 0.001
        if self.gripper_moving:
            self.set_gripper_span(self.desired_span)
        state_msg = Int8()
        state_msg.data = self.gripper_moving
        self.gripper_state_pub.publish(state_msg)

    def on_set_gripper_span_msg(self, msg: Float32):
        # Sanity check on the given span
        if msg.data <= 0:
            self.desired_span = 0
        elif msg.data >= 0.08:
            self.desired_span = 0.08
        else:
            self.desired_span = msg.data
        # Move the gripper to the desired span
        self.set_gripper_span(self.desired_span)

    def on_joint_state_msg(self, msg: JointState):
        # we are only looking for one of the current angles for the base joint
        base_angle = 0
        found = False
        for i in range(len(msg.position)):
            if msg.name[i] == "edo_gripper_left_base_joint":
                base_angle = msg.position[i]
                found = True
        if not found:
            rospy.logwarn("The joint state message does not contain 'edo_gripper_left_base_joint")
            return
        # Geometric conversions
        base_angle = (base_angle / pi) * 180.0
        self.current_span = (-(base_angle - 29.72)/0.7428) / 1000

    def set_gripper_span(self, span):
        rospy.logdebug(f"Set span: {self.desired_span}")
        self.current_span = span
        span *= 1000
        # this magic formula comes from manually measuring the distance and angle and create a trend line using excel
        fingerbase_angle = -0.7428 * span + 29.72
        fingertip_angle = -fingerbase_angle
        fingertip_angle = (fingertip_angle / 180) * pi
        fingerbase_angle = (fingerbase_angle / 180) * pi
        finger_msg = Float64()
        finger_msg.data = fingertip_angle
        baseMsg = Float64()
        baseMsg.data = fingerbase_angle
        self.left_finger_pub.publish(finger_msg)
        self.left_base_pub.publish(baseMsg)
        self.right_finger_pub.publish(finger_msg)
        self.right_base_pub.publish(baseMsg)

    def reset(self):
        self.desired_span = 0.05
        self.set_gripper_span(self.desired_span)

def main():
    egc = EdoGripperControl()
    # 10Hz control loop
    rate = rospy.Rate(10)
    try:
        while True:
            egc.update()
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("Edo gripper node shutdown.")
        return
    except KeyboardInterrupt:
        rospy.loginfo("Edo gripper node shutdown.")
        return

if __name__ == "__main__":
    main()