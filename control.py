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

        rospy.init_node('ui_ros')

        # Control Object
        self.Amm = AmmControl_Func()
        self.Amm.except_signal.connect(self.warning_message_show)
        self.TMArm = ArmControl_Func()
        self.TMArm.pos_signal.connect(self.arm_pos)
        self.TMArm.except_signal.connect(self.warning_message_show)
        self.gripper = GripperController()
        self.gripper.move("a")

        # RViz
        self.gridLayout.addWidget(MyViz())

        # Button
        ## Navigation
        self.nav_start_button.clicked.connect(self.Amm.nav)
        self.nav_stop_button.clicked.connect(self.Amm.nav_stop)
        ## Yolo
        self.yolo_onoff_button.clicked.connect(self.TMArm.Tracker_on_off_client)
        self.yolo_pos_button.clicked.connect(self.TMArm.set_TMPos_yolo)
        ## Gripper
        self.gripper_open_button.clicked.connect(self.gripper.open)
        self.gripper_close_button.clicked.connect(self.gripper.close)
        self.gripper_reset_button.clicked.connect(self.gripper.reset)
        ## TMArm
        self.arm_init_button.clicked.connect(self.TMArm.set_TMPos_init)
        self.arm_set_button.clicked.connect(self.arm_set)
        self.arm_stop_button.clicked.connect(self.TMArm.stop)
        #self.arm_resume_button.clicked.connect(self.TMArm.resume)
        #self.arm_pause_button.clicked.connect(self.TMArm.pause)

        # Timer
        self.connect_check_timer = QTimer(self)
        self.connect_check_timer.timeout.connect(self.services_connect_check)
        self.connect_check_timer.start(1000)

    def services_connect_check(self):
        try:
            node_list = rosnode.get_node_names()
        except rosnode.ROSNodeIOException as e:
            self.warning_message_show(e)
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
            self.TMArm.start()
        else:
            self.arm_status_label.setStyleSheet("color: red")
            self.TMArm.terminate()
        if '/robotiq2FGripper' in node_list:
            self.gripper_status_label.setStyleSheet("color: green")
        else:
            self.gripper_status_label.setStyleSheet("color: red")
    
    def arm_pos(self, pos):
        self.arm_x_LineEdit.setText("{:.2f}".format(float(pos[0])))
        self.arm_y_LineEdit.setText("{:.2f}".format(float(pos[1])))
        self.arm_z_LineEdit.setText("{:.2f}".format(float(pos[2])))
        self.arm_u_LineEdit.setText("{:.2f}".format(float(pos[3])))
        self.arm_v_LineEdit.setText("{:.2f}".format(float(pos[4])))
        self.arm_w_LineEdit.setText("{:.2f}".format(float(pos[5])))
    
    def arm_set(self):
        try:
            pos = []
            pos.append(float(self.arm_setX_LineEdit.text()))
            pos.append(float(self.arm_setY_LineEdit.text()))
            pos.append(float(self.arm_setZ_LineEdit.text()))
            pos.append(float(self.arm_setU_LineEdit.text()))
            pos.append(float(self.arm_setV_LineEdit.text()))
            pos.append(float(self.arm_setW_LineEdit.text()))
            self.TMArm.set_TMPos_r(pos)
        except BaseException as e:
            self.warning_message_show(e)

    def warning_message_show(self, text):
        dlg = QMessageBox(self)
        dlg.setWindowTitle("Warning!")
        dlg.setText(str(text))
        dlg.setStandardButtons(QMessageBox.Ignore)
        dlg.setIcon(QMessageBox.Warning)
        dlg.exec()
        return

if __name__ == '__main__':
    import sys
    app = QtWidgets.QApplication(sys.argv)
    window = Window()
    window.show()
    sys.exit(app.exec_())