from PyQt5 import QtWidgets
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

import control_ui as ui

import os
import subprocess

import rospy


## Service import
from aruco_ros.srv import image
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class Window(QMainWindow, ui.Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        
        self.backhome.clicked.connect(self.nav_goal)

        rospy.init_node('ui_ros')
        
        try:
            rospy.wait_for_service('aruco_image', 3)
            self.aruco_mux_service = rospy.ServiceProxy('aruco_image', image)
            self.aruco_on.clicked.connect(self.aruco_on_clicked)
            self.aruco_off.clicked.connect(self.aruco_off_clicked)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr("Service failed: %s" % (e,))

    def aruco_on_clicked(self):
        self.aruco_mux_service(True)
        #os.system("rosservice call mux_aruco/select /camera/color/image_raw")
    
    def aruco_off_clicked(self):
        self.aruco_mux_service(False)
        #os.system("rosservice call mux_aruco/select /no")

    def nav_goal(self):

        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        rospy.loginfo('Waiting for the action server to start')
        client.wait_for_server()

        rospy.loginfo('Action server started, sending the goal')
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()

        # set frame
        goal.target_pose.header.frame_id = 'map'

        # set position
        goal.target_pose.pose.position.x = 0.0
        goal.target_pose.pose.position.y = 0.0
        # goal.target_pose.pose.position.z = 0.0

        # set orientation
        # goal.target_pose.pose.orientation.x = 0.0
        # goal.target_pose.pose.orientation.y = 0.0
        # goal.target_pose.pose.orientation.z = 1.0
        goal.target_pose.pose.orientation.w = 1.0

        client.send_goal(goal)

        print('Waiting for the result')
        #rospy.loginfo('Waiting for the result')
        client.wait_for_result()

        if client.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo('Succeeded')
        else:
            rospy.loginfo('Failed')

if __name__ == '__main__':
    # import sys
    # app = QtWidgets.QApplication(sys.argv)
    # window = Window()
    # window.show()
    # sys.exit(app.exec_())


    # First import the library
    import pyrealsense2 as rs

    # Create a context object. This object owns the handles to all connected realsense devices
    pipeline = rs.pipeline()
    pipeline.start()

    try:
        for i in range(0, 100):
            frames = pipeline.wait_for_frames()
            for f in frames:
                print(f)

    finally:
        pipeline.stop()