from PyQt5 import QtWidgets
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

import control_ui as ui

import os
import subprocess

import rospy
import rosnode

## Service import
from aruco_ros.srv import image
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist

## Rviz
from rviz import bindings as rviz

## Thread
import threading

## ARM
import RobotControl_func
import ros_gripper

class Window(QMainWindow, ui.Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        rospy.init_node('ui_ros')
        self.twist_pub = rospy.Publisher('cmd_vel', Twist)

        self.nav_start_button.clicked.connect(self.nav_start)
        self.nav_stop_button.clicked.connect(self.nav_stop)

        self.yolo_onoff_button.clicked.connect(self.yolo_onoff)
     
        self.map_widget = MyViz()
        self.gridLayout.addWidget(self.map_widget)

        # Timer
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.services_connect_check)
        self.timer.start(1000)
        
        # arrow key
        self.arrow_key_up_button.pressed.connect(self.arrow_key_pressed)
        self.arrow_key_up_button.released.connect(self.arrow_key_released)
        self.arrow_key_down_button.pressed.connect(self.arrow_key_pressed)
        self.arrow_key_down_button.released.connect(self.arrow_key_released)
        self.arrow_key_right_button.pressed.connect(self.arrow_key_pressed)
        self.arrow_key_right_button.released.connect(self.arrow_key_released)
        self.arrow_key_left_button.pressed.connect(self.arrow_key_pressed)
        self.arrow_key_left_button.released.connect(self.arrow_key_released)

        self.linearSpeed_slider.valueChanged.connect(self.slider_value_change)
        self.rotationSpeed_slider.valueChanged.connect(self.slider_value_change)

        # ARM
        self.robot = RobotControl_func.RobotControl_Func()
        self.robotiq_gripper = ros_gripper.gripperController()

        self.robotiq_gripper.move('r')
        self.robotiq_gripper.activate()
        self.robotiq_gripper.move('o')

        self.gripper_on_button.clicked.connect(self.gripper_on)
        self.gripper_off_button.clicked.connect(self.gripper_off)

        self.arm_init_button.clicked.connect(self.arm_init)
        self.arm_set_button.clicked.connect(self.arm_set)

    def arrow_key_pressed(self):
        button = self.sender()
        move_cmd = Twist()
        if button == self.arrow_key_up_button:
            button.setIcon(QIcon(QPixmap("icon/arrowkey_up_pressed.png")))
            move_cmd.linear.x = self.linearSpeed_slider.value() / 10.0
        elif button == self.arrow_key_down_button:
            button.setIcon(QIcon(QPixmap("icon/arrowkey_down_pressed.png")))
            move_cmd.linear.x = -self.linearSpeed_slider.value() / 10.0
        elif button == self.arrow_key_right_button:
            button.setIcon(QIcon(QPixmap("icon/arrowkey_right_pressed.png")))
            move_cmd.angular.z = -self.rotationSpeed_slider.value() / 10.0
        elif button == self.arrow_key_left_button:
            button.setIcon(QIcon(QPixmap("icon/arrowkey_left_pressed.png")))
            move_cmd.angular.z = self.rotationSpeed_slider.value() / 10.0
        self.twist_pub.publish(move_cmd)

    def arrow_key_released(self):
        button = self.sender()
        move_cmd = Twist()
        if button == self.arrow_key_up_button:
            button.setIcon(QIcon(QPixmap("icon/arrowkey_up.png")))
        elif button == self.arrow_key_down_button:
            button.setIcon(QIcon(QPixmap("icon/arrowkey_down.png")))
        elif button == self.arrow_key_right_button:
            button.setIcon(QIcon(QPixmap("icon/arrowkey_right.png")))
        elif button == self.arrow_key_left_button:
            button.setIcon(QIcon(QPixmap("icon/arrowkey_left.png")))
        self.twist_pub.publish(move_cmd)   

    def nav_goal(self):

        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        rospy.loginfo('Waiting for the action server to start')
        client.wait_for_server()

        rospy.loginfo('Action server started, sending the goal')
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()

        # set frame
        goal.target_pose.header.frame_id = 'map'

        try:
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
        except:
            rospy.loginfo('Exception')

    def nav_start(self):
        t = threading.Thread(target = self.nav_goal)
        t.start()

    def nav_stop(self):
        os.system("rostopic pub /move_base/cancel actionlib_msgs/GoalID -- {}")

    def services_connect_check(self):
        node_list = rosnode.get_node_names()

        if '/camera/realsense2_camera' in node_list:
            self.realsense_status_label.setStyleSheet("color: green")
        else:
            self.realsense_status_label.setStyleSheet("color: red")
        
        if '/detection_publisher' in node_list:
            self.detect_status_label.setStyleSheet("color: green")
        else:
            self.detect_status_label.setStyleSheet("color: red")
        
        if '/darknet_ros' in node_list:
            self.yolo_status_label.setStyleSheet("color: green")
        else:
            self.yolo_status_label.setStyleSheet("color: red")       
        
        if '/velodyne_nodelet_manager' in node_list:
            self.velodyne_status_label.setStyleSheet("color: green")
        else:
            self.velodyne_status_label.setStyleSheet("color: red")

        if '/rtabmap/rtabmap' in node_list:
            self.rtab_status_label.setStyleSheet("color: green")
        else:
            self.rtab_status_label.setStyleSheet("color: red")

        if '/tm_driver' in node_list:
            self.arm_status_label.setStyleSheet("color: green")
            pos = self.robot.get_TMPos()
            self.arm_x_LineEdit.setText("{:.2f}".format(pos[0]))
            self.arm_y_LineEdit.setText("{:.2f}".format(pos[1]))
            self.arm_z_LineEdit.setText("{:.2f}".format(pos[2]))
            self.arm_u_LineEdit.setText("{:.2f}".format(pos[3]))
            self.arm_v_LineEdit.setText("{:.2f}".format(pos[4]))
            self.arm_w_LineEdit.setText("{:.2f}".format(pos[5]))
        else:
            self.arm_status_label.setStyleSheet("color: red")

        if '/robotiq2FGripper' in node_list:
            self.gripper_status_label.setStyleSheet("color: green")
        else:
            self.gripper_status_label.setStyleSheet("color: red")

    def slider_value_change(self, value):
        slider = self.sender()
        if slider == self.linearSpeed_slider:
            self.linearSpeed_label.setNum(value)
        elif slider == self.rotationSpeed_slider:
            self.rotationSpeed_label.setNum(value)
    
    def yolo_onoff(self):
        print("yolo_onoff")
        print(self.robot.Tracker_on_off_client())

    def gripper_on(self):
        self.robotiq_gripper.move("o")
    
    def gripper_off(self):
        self.robotiq_gripper.move("c")

    def arm_init(self):
        self.robot.set_TMPos([376.0587, -436.61, 734.17, 179.98, 0, -135])

    def arm_set(self):
        try:
            x = float(self.arm_x_LineEdit.text()) + float(self.arm_setX_LineEdit.text())
            y = float(self.arm_y_LineEdit.text()) + float(self.arm_setY_LineEdit.text())
            z = float(self.arm_z_LineEdit.text()) + float(self.arm_setZ_LineEdit.text())
            u = float(self.arm_u_LineEdit.text()) + float(self.arm_setU_LineEdit.text())
            v = float(self.arm_v_LineEdit.text()) + float(self.arm_setV_LineEdit.text())
            w = float(self.arm_w_LineEdit.text()) + float(self.arm_setW_LineEdit.text())
            #print([x, y, z, u, v, w])
            self.robot.set_TMPos([x, y, z, u, v, w])
        except:
            pass


class MyViz( QWidget ):
    def __init__(self):
        QWidget.__init__(self)
        self.frame = rviz.VisualizationFrame()
        self.frame.setSplashPath( "" )
        self.frame.initialize()
        reader = rviz.YamlConfigReader()
        config = rviz.Config()

        #you will need a rviz config file. This config file basically has the information about what all from the rviz you want to display on your custom UI.
   
        reader.readFile( config, "my_rviz_file.rviz" )
        self.frame.load( config )

        #some settings for how you want your rviz screen to look like.
        self.setWindowTitle( config.mapGetChild( "Title" ).getValue() )
        #self.frame.setMenuBar( None )
        self.frame.setStatusBar( None )
        self.frame.setHideButtonVisibility( False )
        self.manager = self.frame.getManager()
        self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt( 0 )
        layout = QVBoxLayout()
        layout.addWidget( self.frame )
        self.setLayout( layout )

if __name__ == '__main__':
    import sys
    app = QtWidgets.QApplication(sys.argv)
    window = Window()
    window.show()
    sys.exit(app.exec_())