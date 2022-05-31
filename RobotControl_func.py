# import pythoncom
# pythoncom.CoInitialize()
from __future__ import print_function

import time
import random
import numpy as np
import os

import rospy

from tm_msgs.msg import *
from tm_msgs.srv import *
from darknet_ros_nodes.srv import *
from darknet_ros_msgs.srv import *
from tmarm_grab.srv import *

import math
import threading

from PyQt5.QtCore import QThread, pyqtSignal, QMutex

class ArmControl_Func(QThread):
    pos_signal = pyqtSignal(list)
    except_signal = pyqtSignal(BaseException)

    def __init__(self):
        super().__init__()
        self.m = np.array([
            [math.cos(math.radians(135)), -math.sin(math.radians(135)), 0.0, 0.0, 0.0, 0.0],
            [math.sin(math.radians(135)), math.cos(math.radians(135)), 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        ])
        self.mv = np.array([
            [math.cos(math.radians(-135)), -math.sin(math.radians(-135)), 0.0, 0.0, 0.0, 0.0],
            [math.sin(math.radians(-135)), math.cos(math.radians(-135)), 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        ])

        self.__current_pos = [0,0,0,0,0,0]
        self.__mutex = QMutex()
        self.__init_pos = [376.0587, -436.61, 734.17, 179.98, 0, 45]
        self.__yolo_pos = [405., -612., 396., 90.0, 0., 45.]

    def set_TMPos(self, pos, speed = 20, line = False):
        # transself.set_TMPos_new(pos)form to TM robot coordinate
        tmp = []

        tmp.append(pos[0] / 1000)
        tmp.append(pos[1] / 1000)
        tmp.append(pos[2] / 1000)
        tmp.append(pos[3] * np.pi / 180)
        tmp.append(pos[4] * np.pi / 180)
        tmp.append(pos[5] * np.pi / 180)

        rospy.wait_for_service('tm_driver/set_event', timeout = 0.1)
        set_positions = rospy.ServiceProxy('tm_driver/set_positions', SetPositions)

        if line == False:
            set_positions(SetPositionsRequest.PTP_T, tmp, speed, 1, 0, False)
        else:
            set_positions(SetPositionsRequest.LINE_T, tmp, speed, 0.5, 0, False)

    def get_TMPos(self):
        data = rospy.wait_for_message("/feedback_states", FeedbackState, timeout = 0.3)

        current_pos = np.array(list(data.tool_pose))
        current_pos[0] = current_pos[0] * 1000
        current_pos[1] = current_pos[1] * 1000
        current_pos[2] = current_pos[2] * 1000
        current_pos[3] = current_pos[3] * 180 / np.pi
        current_pos[4] = current_pos[4] * 180 / np.pi
        current_pos[5] = current_pos[5] * 180 / np.pi

        return current_pos

    def set_TMPos_init(self):
        try:
            self.set_TMPos(self.__init_pos)
        except BaseException as e:
            self.except_signal.emit(e)

    def set_TMPos_yolo(self):
        try:
            self.set_TMPos(self.__yolo_pos)
        except BaseException as e:
            self.except_signal.emit(e)

    def stop(self):
        try:
            rospy.wait_for_service('tm_driver/set_event', timeout = 0.1)
            set_event = rospy.ServiceProxy('tm_driver/set_event', SetEvent)
            set_event(11,0,0)
        except (rospy.ROSException, rospy.ServiceException) as e:
            rospy.logerr(e)
    
    def pause(self):
        try:
            rospy.wait_for_service('tm_driver/set_event', timeout = 0.1)
            set_event = rospy.ServiceProxy('tm_driver/set_event', SetEvent)
            set_event(12,0,0)
        except (rospy.ROSException, rospy.ServiceException) as e:
            rospy.logerr(e)
    
    def resume(self):
        try:
            rospy.wait_for_service('tm_driver/set_event', timeout = 0.1)
            set_event = rospy.ServiceProxy('tm_driver/set_event', SetEvent)
            set_event(13,0,0)
        except (rospy.ROSException, rospy.ServiceException) as e:
            rospy.logerr(e)

    def Tracker_on_off_client(self):
        try:
            rospy.wait_for_service('darknet_ros/is_on', timeout = 0.1)
            rospy.wait_for_service('detection_publisher/Tracker_on_off', timeout = 0.1)
            Tracker_on_off_func = rospy.ServiceProxy('darknet_ros/is_on', IsOn)
            Tracker_on_off_func2 = rospy.ServiceProxy('detection_publisher/Tracker_on_off', Tracker_on_off)
            resp2 = Tracker_on_off_func2()
            resp1 = Tracker_on_off_func()
            #rospy.loginfo("Tracker_on_off " + resp2)
        except (rospy.ROSException, rospy.ServiceException) as e:
            rospy.logerr(e)
            rospy.logerr("Tracker_on_off_client fail")
    
    def grab_aruco_client(start=True):
        try:
            rospy.wait_for_service('grab_aruco', timeout=0.1)
            grab_aruco = rospy.ServiceProxy('grab_aruco', GrabArUco)
            resp1 = grab_aruco(start)
            if (resp1.end):
                print("Done!")
            else:
                print("Failed!")
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


import actionlib
from actionlib_msgs.msg import GoalStatus, GoalID
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist

class AmmControl_Func(QThread):
    result_signal = pyqtSignal(str)
    except_signal = pyqtSignal(BaseException)

    def __init__(self):
        super().__init__()
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 0)
        self.cancel_pub = rospy.Publisher("move_base/cancel", GoalID, queue_size = 1)
        self.__mutex = QMutex()
        self.__nav_goal = MoveBaseGoal()
        self.__nav_goal.target_pose.header.frame_id = 'map'
        self.set_nav_goal()
        
    def move(self, x=0, z=0):
        cmd = Twist()
        cmd.linear.x = x
        cmd.angular.z = z
         
        self.twist_pub.publish(cmd)

    def set_nav_goal(self, x=0.0, y=0.0, w=1.0):
        self.__mutex.lock()
        self.__nav_goal.target_pose.pose.position.x = x
        self.__nav_goal.target_pose.pose.position.y = y
        self.__nav_goal.target_pose.pose.orientation.w = w
        self.__mutex.unlock()

    def nav(self):
        self.run = self.__nav_thread
        self.start()

    def __nav_thread(self):
        try:
            self.__mutex.lock()
            time = rospy.Duration.from_sec(3)
            result = self.client.wait_for_server(timeout = time)
            if result is False: 
                self.__mutex.unlock()
                raise Exception('move_base time out')

            self.__nav_goal.target_pose.header.stamp = rospy.Time.now()

            self.client.send_goal(__nav_goal)
            self.__mutex.unlock()
            self.client.wait_for_result()

            if self.client.get_state() == GoalStatus.SUCCEEDED:
                self.result_signal.emit("Navigation Succeeded")
                rospy.loginfo('Navigation Succeeded')
            else:
                raise Exception('Navigation Fail')
        except BaseException as e:
            #rospy.logerr(e)
            self.except_signal.emit(e)
    
    def nav_stop(self):
        try:
            goal = GoalID()
            self.cancel_pub.publish(goal)
        except BaseException as e:
            #rospy.logerr(e)
            self.except_signal.emit(e)

import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from time import sleep

try:
    input = raw_input
except NameError:
    pass

class GripperController():
    def __init__(self):
        self.pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output,queue_size = 0)
        self.command = outputMsg.Robotiq2FGripper_robot_output()

    def genCommand(self, char, command):
        """Update the command according to the character entered by the user."""

        if char == 'a':
            command = outputMsg.Robotiq2FGripper_robot_output();
            command.rACT = 1
            command.rGTO = 1
            command.rSP  = 255
            command.rFR  = 150

        if char == 'r':
            command = outputMsg.Robotiq2FGripper_robot_output();
            command.rACT = 0

        if char == 'c':
            command.rPR = 255

        if char == 'o':
            command.rPR = 0

        #If the command entered is a int, assign this value to rPRA
        try:
            command.rPR = int(char)
            if command.rPR > 255:
                command.rPR = 255
            if command.rPR < 0:
                command.rPR = 0
        except ValueError:
            pass

        if char == 'f':
            command.rSP += 25
            if command.rSP > 255:
                command.rSP = 255

        if char == 'l':
            command.rSP -= 25
            if command.rSP < 0:
                command.rSP = 0


        if char == 'i':
            command.rFR += 25
            if command.rFR > 255:
                command.rFR = 255

        if char == 'd':
            command.rFR -= 25
            if command.rFR < 0:
                command.rFR = 0

        return command

    def askForCommand(self, command):
        """Ask the user for a command to send to the gripper."""

        currentCommand  = 'Simple 2F Gripper Controller\n-----\nCurrent command:'
        currentCommand += '  rACT = '  + str(command.rACT)
        currentCommand += ', rGTO = '  + str(command.rGTO)
        currentCommand += ', rATR = '  + str(command.rATR)
        currentCommand += ', rPR = '   + str(command.rPR )
        currentCommand += ', rSP = '   + str(command.rSP )
        currentCommand += ', rFR = '   + str(command.rFR )


        print(currentCommand)

        strAskForCommand  = '-----\nAvailable commands\n\n'
        strAskForCommand += 'r: Reset\n'
        strAskForCommand += 'a: Activate\n'
        strAskForCommand += 'c: Close\n'
        strAskForCommand += 'o: Open\n'
        strAskForCommand += '(0-255): Go to that position\n'
        strAskForCommand += 'f: Faster\n'
        strAskForCommand += 'l: Slower\n'
        strAskForCommand += 'i: Increase force\n'
        strAskForCommand += 'd: Decrease force\n'

        strAskForCommand += '-->'

        return input(strAskForCommand)
        
    def move(self,ch):
        """Main loop which requests new commands and publish them on the Robotiq2FGripperRobotOutput topic."""
        self.command = self.genCommand(ch, self.command)
        self.pub.publish(self.command)

    def open(self):
        self.move("o")
    
    def close(self):
        self.move("c")
    
    def reset(self):
        self.move("r")
    
    def active(self):
        self.move("a")