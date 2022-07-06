import os
import rospy
import rospkg
import time

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, qWarning, Signal, QObject
from python_qt_binding.QtWidgets import QSlider, QWidget, QLabel
from std_msgs.msg import Int16


class EdoSlider(QSlider):
    def __init__(self, parent=None):
        super(EdoSlider, self).__init__()
        self.valueChanged.connect(self.value_changed_handler)
        self.value_publisher = rospy.Publisher("/slider_value/", Int16, queue_size=10)

    def init_publisher(self, name):
        self.value_publisher = rospy.Publisher(name + "/slider_value/", Int16, queue_size=10)

    def value_changed_handler(self, data):
        msg = Int16()
        msg.data = data
        self.value_publisher.publish(msg)

class EdoGuiWidget(QWidget):
    """
    Widget for use with EdoGui class
    """
    def __init__(self, context):
        """
        :param context: plugin context hook to enable adding widgets as a ROS_GUI pane, ''PluginContext''
        """
        super(EdoGuiWidget, self).__init__()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('edo'), 'resource', 'form.ui')
        loadUi(ui_file, self, {'EdoSlider': EdoSlider})
        self.setObjectName('EdoGui')
        self.joint_1.init_publisher("joint_1")
        self.joint_2.init_publisher("joint_2")
        self.joint_3.init_publisher("joint_3")
        self.joint_4.init_publisher("joint_4")
        self.joint_5.init_publisher("joint_5")
        self.joint_6.init_publisher("joint_6")
        self.joint_7.init_publisher("joint_7")

    def handle_close(self, event):
        event.accept()