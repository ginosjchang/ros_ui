from PyQt5 import QtWidgets
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

## GUI
import ui.control_ui as control_ui
import ui.node_checker_ui as node_checker_ui
import ui.amm_control_ui as amm_control_ui
import ui.arm_control_ui as arm_control_ui
from ui.rviz_ui import MyViz

import rospy
import rosnode

from RobotControl_func import ArmControl_Func, AmmControl_Func, GripperController

class Window(QMainWindow, control_ui.Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        rospy.init_node('ui_ros')

        # Control Object
        self.Amm = AmmControl_Func()
        #self.Amm.except_signal.connect(self.warning_message_show)
        self.TMArm = ArmControl_Func()
        self.TMArm.except_signal.connect(self.warning_message_show)
        self.gripper = GripperController()

        # SubWindow
        self.ncwindow = Node_Checker_Window()
        self.tmwidow = TM_Control_Window(self.TMArm)
        self.ammwindow = Amm_Control_Window(self.Amm)

        # Action
        ## Veiw
        self.actionNode_checker.triggered.connect(self.ncwindow.show)
        self.actionAmm_control.triggered.connect(self.ammwindow.show)
        self.actionArm_control.triggered.connect(self.tmwidow.show)
        ## Gripper
        self.actionActive.triggered.connect(self.gripper.active)
        self.actionReset.triggered.connect(self.gripper.reset)
        self.actionOpen.triggered.connect(self.gripper.open)
        self.actionClose.triggered.connect(self.gripper.close)
        ## TM_Pos
        self.actionInit_Pos.triggered.connect(self.TMArm.set_TMPos_init)
        self.actionYolo_Pos.triggered.connect(self.TMArm.set_TMPos_yolo)
        self.actionGrab_Pos.triggered.connect(self.TMArm.set_TMPos_grab)
        ## Navigayion
        self.actionNavTable.triggered.connect(self.nav_table)
        self.actionOriginal.triggered.connect(self.Amm.set_nav_goal)

        # RViz
        self.gridLayout.addWidget(MyViz("ui/my_rviz_file.rviz"))

        # Button
        ## Navigation
        self.nav_start_button.clicked.connect(self.Amm.nav_worker.start)
        ## Yolo
        self.yolo_onoff_button.clicked.connect(self.TMArm.Tracker_on_off_client)
        ## Stop
        self.interrupt_button.clicked.connect(self.interrupt)
        ## TMArm
        #self.arm_stop_button.clicked.connect(self.TMArm.stop)
        #self.arm_resume_button.clicked.connect(self.TMArm.resume)
        #self.arm_pause_button.clicked.connect(self.TMArm.pause)
        self.grab_item_button.clicked.connect(self.grab)
        self.put_item_button.clicked.connect(self.put)

    def warning_message_show(self, text):
        dlg = QMessageBox(self)
        dlg.setWindowTitle("Warning!")
        dlg.setText(str(text))
        dlg.setStandardButtons(QMessageBox.Ignore)
        dlg.setIcon(QMessageBox.Warning)
        dlg.exec()
    
    def closeEvent(self, event):
        self.ncwindow.close()
        self.tmwidow.close()
        self.ammwindow.close()
    
    def interrupt(self):
        self.Amm.nav_stop()
        self.Amm.move(0,0)
        self.TMArm.stop()
    
    def grab(self):
        try:
            self.gripper.item_id = int(self.item_id_LineEdit.text())
            self.gripper.start()
            self.gripper.isput = False
        except BaseException as e:
            self.warning_message_show(e)
    
    def put(self):
        try:
            self.gripper.item_id = int(self.item_id_LineEdit.text())
            self.gripper.isput = True
            self.gripper.start()
        except BaseException as e:
            self.warning_message_show(e)
    
    def nav_table(self):
        self.Amm.set_nav_goal(12.276, -0.284, 1.0)
    
    def nav_zero(self):
        self.Amm.set_nav_goal()

class Node_Checker_Window(QWidget, node_checker_ui.Ui_NodeChecker):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setupUi(self)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.checker)
        self.nodeNum = 0
        self.checker_list =[
            ['/camera/realsense2_camera', self.rs_status_button],
            ['/darknet_ros', self.yolo_status_button],
            ['/detection_publisher', self.detect_status_button],
            ['/velodyne_nodelet_manager', self.velodyne_status_button],
            ['/rtabmap/rtabmap', self.rtab_status_button],
            ['/tm_driver', self.tm_status_button],
            ['/robotiq2FGripper', self.gripper_status_button]
            ]
    def showEvent(self, event):
        self.timer.start(1000)
    
    def closeEvent(self, event):
        self.timer.stop()
    
    def checker(self):
        try:
            node_list = rosnode.get_node_names()
        except BaseException as e:
            node_list = []

        num = len(node_list)
        if(self.nodeNum == num): return
        else : self.nodeNum = num

        for item in self.checker_list:
            if item[0] in node_list:
                item[1].setIcon(QIcon(QPixmap("icon/icons8-ok-48.png")))
            else:
                item[1].setIcon(QIcon(QPixmap("icon/icons8-close-512.png")))

class TM_Control_Window(QWidget, arm_control_ui.Ui_Arm_control):
    def __init__(self, TMArm, parent=None):
        super().__init__(parent)
        self.setupUi(self)
        self.TMArm = TMArm
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.arm_pos)
        self.arm_stop_button.clicked.connect(self.TMArm.stop)
        self.arm_set_button.clicked.connect(self.arm_set)

    def showEvent(self, event):
        self.timer.start(1000)
    
    def closeEvent(self, event):
        self.timer.stop()
    
    def arm_set(self):
        try:
            pos = self.TMArm.get_TMPos()
            pos[0] += float(self.arm_setX_LineEdit.text())
            pos[1] += float(self.arm_setY_LineEdit.text())
            pos[2] += float(self.arm_setZ_LineEdit.text())
            pos[3] += float(self.arm_setU_LineEdit.text())
            pos[4] += float(self.arm_setV_LineEdit.text())
            pos[5] += float(self.arm_setW_LineEdit.text())
            self.TMArm.set_TMPos(pos)
        except BaseException as e:
            warning_message_show(self, e)

    def arm_pos(self):
        try:
            pos = self.TMArm.get_TMPos()
            self.arm_x_LineEdit.setText("{:.2f}".format(float(pos[0])))
            self.arm_y_LineEdit.setText("{:.2f}".format(float(pos[1])))
            self.arm_z_LineEdit.setText("{:.2f}".format(float(pos[2])))
            self.arm_u_LineEdit.setText("{:.2f}".format(float(pos[3])))
            self.arm_v_LineEdit.setText("{:.2f}".format(float(pos[4])))
            self.arm_w_LineEdit.setText("{:.2f}".format(float(pos[5])))
        except BaseException as e:
            warning_message_show(self, e)

class Amm_Control_Window(QWidget, amm_control_ui.Ui_Amm_control):
    def __init__(self, Amm, parent=None):
        super().__init__(parent)
        self.setupUi(self)
        self.Amm = Amm
        # Button
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
    
    def arrow_key_pressed(self):
        button = self.sender()
        x = 0
        z = 0
        if button == self.arrow_key_up_button:
            button.setIcon(QIcon(QPixmap("icon/arrowkey_up_pressed.png")))
            x = self.linearSpeed_slider.value() / 10.0
            #print(self.Amm.forward_worker.isRunning())
            if self.Amm.forward_worker.isRunning() :
                self.Amm.forward_worker.terminate()
            else:
                self.Amm.forward_worker.start()
        elif button == self.arrow_key_down_button:
            button.setIcon(QIcon(QPixmap("icon/arrowkey_down_pressed.png")))
            x = -self.linearSpeed_slider.value() / 10.0
            if self.Amm.backward_worker.isRunning():
                self.Amm.backward_worker.terminate()
            else:
                self.Amm.backward_worker.start()
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
        #self.Amm.move()
    
    def slider_value_change(self, value):
        slider = self.sender()
        if slider == self.linearSpeed_slider:
            self.linearSpeed_label.setNum(value)
        elif slider == self.rotationSpeed_slider:
            self.rotationSpeed_label.setNum(value)

def warning_message_show(self, text):
    dlg = QMessageBox(self)
    dlg.setWindowTitle("Warning!")
    dlg.setText(str(text))
    dlg.setStandardButtons(QMessageBox.Ignore)
    dlg.setIcon(QMessageBox.Warning)
    dlg.exec()

if __name__ == '__main__':
    import sys
    app = QtWidgets.QApplication(sys.argv)
    window = Window()
    window.show()
    sys.exit(app.exec_())