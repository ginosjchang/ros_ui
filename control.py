from PyQt5 import QtWidgets
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

import control_ui as ui

import os

import rospy
import rosnode

## Rviz
from rviz_ui import MyViz

## Thread
import threading

## ARM
#from ros_gripper import gripperController
from RobotControl_func import ArmControl_Func, AmmControl_Func
from RobotControl_func import GripperController

class Window(QMainWindow, ui.Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        self.nodeNum = 0
        self.armMode = "m"

        rospy.init_node('ui_ros')

        # Control Object
        self.Amm = AmmControl_Func()
        self.TMArm = ArmControl_Func()
        self.gripper = GripperController()
        #self.gripper.move("r")
        self.gripper.move("a")

        # RViz
        self.gridLayout.addWidget(MyViz())

        # Button
        ## Navigation
        self.nav_start_button.clicked.connect(self.nav_start)
        self.nav_stop_button.clicked.connect(self.Amm.nav_stop)
        ## Arrow Key
        self.arrow_key_up_button.pressed.connect(self.arrow_key_pressed)
        self.arrow_key_up_button.released.connect(self.arrow_key_released)
        self.arrow_key_down_button.pressed.connect(self.arrow_key_pressed)
        self.arrow_key_down_button.released.connect(self.arrow_key_released)
        self.arrow_key_right_button.pressed.connect(self.arrow_key_pressed)
        self.arrow_key_right_button.released.connect(self.arrow_key_released)
        self.arrow_key_left_button.pressed.connect(self.arrow_key_pressed)
        self.arrow_key_left_button.released.connect(self.arrow_key_released)
        ## Slider (speed control)
        self.linearSpeed_slider.valueChanged.connect(self.slider_value_change)
        self.rotationSpeed_slider.valueChanged.connect(self.slider_value_change)
        ## Yolo
        self.yolo_onoff_button.clicked.connect(self.TMArm.Tracker_on_off_client)
        self.yolo_pos_button.clicked.connect(self.TMArm.yolo_pos)
        ## Gripper
        self.gripper_open_button.clicked.connect(self.gripper.open)
        self.gripper_close_button.clicked.connect(self.gripper.close)
        self.gripper_reset_button.clicked.connect(self.gripper.reset)
        ## TMArm
        self.arm_init_button.clicked.connect(self.TMArm.go_initPos)
        self.arm_set_button.clicked.connect(self.arm_set)
        self.arm_m_mode_button.clicked.connect(self.arm_mode_m)
        self.arm_c_mode_button.clicked.connect(self.arm_mode_c)
        self.arm_stop_button.clicked.connect(self.TMArm.stop)
        #self.arm_resume_button.clicked.connect(self.TMArm.resume)
        #self.arm_pause_button.clicked.connect(self.TMArm.pause)

        # Timer
        self.connect_check_timer = QTimer(self)
        self.connect_check_timer.timeout.connect(self.services_connect_check)
        self.connect_check_timer.start(1000)

        self.arm_currentPos_timer = QTimer(self)
        self.arm_currentPos_timer.timeout.connect(self.arm_currentPos)

    def arrow_key_pressed(self):
        button = self.sender()
        x = 0
        z = 0
        if button == self.arrow_key_up_button:
            button.setIcon(QIcon(QPixmap("icon/arrowkey_up_pressed.png")))
            x = self.linearSpeed_slider.value() / 10.0
        elif button == self.arrow_key_down_button:
            button.setIcon(QIcon(QPixmap("icon/arrowkey_down_pressed.png")))
            x = -self.linearSpeed_slider.value() / 10.0
        elif button == self.arrow_key_right_button:
            button.setIcon(QIcon(QPixmap("icon/arrowkey_right_pressed.png")))
            z = -self.rotationSpeed_slider.value() / 10.0
        elif button == self.arrow_key_left_button:
            button.setIcon(QIcon(QPixmap("icon/arrowkey_left_pressed.png")))
            z = self.rotationSpeed_slider.value() / 10.0
        self.Amm.move(x,z)

    def arrow_key_released(self):
        button = self.sender()
        if button == self.arrow_key_up_button:
            button.setIcon(QIcon(QPixmap("icon/arrowkey_up.png")))
        elif button == self.arrow_key_down_button:
            button.setIcon(QIcon(QPixmap("icon/arrowkey_down.png")))
        elif button == self.arrow_key_right_button:
            button.setIcon(QIcon(QPixmap("icon/arrowkey_right.png")))
        elif button == self.arrow_key_left_button:
            button.setIcon(QIcon(QPixmap("icon/arrowkey_left.png")))
        self.Amm.move()

    def nav_start(self):
        t = threading.Thread(target = self.Amm.nav)
        t.start()

    def services_connect_check(self):
        try:
            node_list = rosnode.get_node_names()
        except rosnode.ROSNodeIOException as e:
            dlg = QMessageBox(self)
            dlg.setWindowTitle("Disconnect!")
            dlg.setText(str(e))
            dlg.setStandardButtons(QMessageBox.Retry)
            dlg.setIcon(QMessageBox.Warning)
            dlg.exec()
            return

        num = len(node_list)
        if(self.nodeNum == num): return
        else : self.nodeNum = num

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
            self.arm_currentPos_timer.start(1000)
        else:
            self.arm_status_label.setStyleSheet("color: red")
            self.arm_currentPos_timer.stop()

        if '/robotiq2FGripper' in node_list:
            self.gripper_status_label.setStyleSheet("color: green")
        else:
            self.gripper_status_label.setStyleSheet("color: red")
    
    def arm_currentPos(self):
        try:
            pos = self.TMArm.get_TMPos(mode=self.armMode)
            self.arm_x_LineEdit.setText("{:.2f}".format(float(pos[0])))
            self.arm_y_LineEdit.setText("{:.2f}".format(float(pos[1])))
            self.arm_z_LineEdit.setText("{:.2f}".format(float(pos[2])))
            self.arm_u_LineEdit.setText("{:.2f}".format(float(pos[3])))
            self.arm_v_LineEdit.setText("{:.2f}".format(float(pos[4])))
            self.arm_w_LineEdit.setText("{:.2f}".format(float(pos[5])))
        except (rospy.ROSException, rospy.ServiceException) as e:
            rospy.logerr(e)

    def slider_value_change(self, value):
        slider = self.sender()
        if slider == self.linearSpeed_slider:
            self.linearSpeed_label.setNum(value)
        elif slider == self.rotationSpeed_slider:
            self.rotationSpeed_label.setNum(value)
    
    def arm_set(self):
        try:
            pos = self.TMArm.get_TMPos(mode=self.armMode)
            pos[0] += float(self.arm_setX_LineEdit.text())
            pos[1] += float(self.arm_setY_LineEdit.text())
            pos[2] += float(self.arm_setZ_LineEdit.text())
            pos[3] += float(self.arm_setU_LineEdit.text())
            pos[4] += float(self.arm_setV_LineEdit.text())
            pos[5] += float(self.arm_setW_LineEdit.text())
            self.TMArm.set_TMPos(pos,mode=self.armMode)
        except (rospy.ROSException, rospy.ServiceException) as e:
            rospy.logerr(e)
    
    def arm_mode_m(self):
        self.armMode = "m"
    def arm_mode_c(self):
        self.armMode = "c"

if __name__ == '__main__':
    import sys
    app = QtWidgets.QApplication(sys.argv)
    window = Window()
    window.show()
    sys.exit(app.exec_())