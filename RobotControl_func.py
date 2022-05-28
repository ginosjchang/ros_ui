# import pythoncom
# pythoncom.CoInitialize()
from __future__ import print_function

import time
import random
import numpy as np

import rospy

from tm_msgs.msg import *
from tm_msgs.srv import *
from darknet_ros_nodes.srv import *
from darknet_ros_msgs.srv import *

class ArmControl_Func():
    def __init__(self):
        super().__init__()

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

        data = rospy.wait_for_message("/feedback_states", FeedbackState, timeout = 0.1)

        current_pos = list(data.tool_pose)
        current_pos[0] = current_pos[0] * 1000
        current_pos[1] = current_pos[1] * 1000
        current_pos[2] = current_pos[2] * 1000
        current_pos[3] = current_pos[3] * 180 / np.pi
        current_pos[4] = current_pos[4] * 180 / np.pi
        current_pos[5] = current_pos[5] * 180 / np.pi

        return current_pos
    
    def go_initPos(self):
        self.set_TMPos([376.0587, -436.61, 734.17, 179.98, 0, -135])

    def Tracker_on_off_client(self):
        try:
            Tracker_on_off_func = rospy.ServiceProxy('darknet_ros/is_on', IsOn)
            Tracker_on_off_func2 = rospy.ServiceProxy('detection_publisher/Tracker_on_off', Tracker_on_off)
            resp2 = Tracker_on_off_func2()
            rospy.loginfo("Tracker_on_off " + resp2)
        except rospy.ServiceException as e:
            rospy.logerr("Tracker_on_off_client fail")


import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist

class AmmControl_Func():
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 0)
    
    def move(self, x=0, z=0):
        cmd = Twist()
        cmd.linear.x = x
        cmd.angular.z = z
         
        self.twist_pub.publish(cmd)

    def nav(self, x=0.0, y=0.0, w=0.1):
        try:
            self.client.wait_for_server(timeout = 1)

            goal = MoveBaseGoal()
            goal.target_pose.header.stamp = rospy.Time.now()

            # set frame
            goal.target_pose.header.frame_id = 'map'
            # set position and orientation
            goal.target_pose.pose.position.x = x
            goal.target_pose.pose.position.y = y
            goal.target_pose.pose.orientation.w = w

            self.client.send_goal_and_wait(goal)

            if self.client.get_state() == GoalStatus.SUCCEEDED:
                rospy.loginfo('Navigation Succeeded')
            else:
                rospy.loginfo('Navigation Failed')
        except:
            rospy.loginfo('Navigation Exception')
    
    def nav_stop(self):
        os.system("rostopic pub /move_base/cancel actionlib_msgs/GoalID -- {}")

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
            
    def activate(self):
        """Main loop which requests new commands and publish them on the Robotiq2FGripperRobotOutput topic."""
        self.command = self.genCommand('a', self.command)
        self.pub.publish(self.command)
        rospy.sleep(0.1)
        
    def move(self,ch):
        """Main loop which requests new commands and publish them on the Robotiq2FGripperRobotOutput topic."""
        self.command = self.genCommand(ch, self.command)
        self.pub.publish(self.command)
        rospy.sleep(0.1)

    def open(self):
        self.move("o")
    
    def close(self):
        self.move("c")