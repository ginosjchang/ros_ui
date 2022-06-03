# ros_ui
## Embeded Rviz
You can install via:
```bash=
sudo apt-get install ros-noetic-visualization-tutorials
```
Import it in your python file via:
```bash=
from rviz import bindings as rviz
```
Use config file data to set rviz config:
```bash=
reader.readFile( config, "my_rviz_file.rviz" )
```
Disable MenuBar with can't open/save config and panels control.
```bash=
self.frame.setMenuBar( None )
```
Disable setStatusBar
```bash=
self.frame.setStatusBar( None )
```
# RobotControl_func
## TM Arm Control class
### Functions
- set_TMPos(pos, speed=20, line=False) : Set TM position.
- get_TMPos() : Return TM current position.
- set_TMPos_init() : Set TM position to __init_pos.
- set_TMPos_yolo() : Set TM position to __yolo_pos.
- set_TMPos_grab() : Set TM position to __grab_pos.
- stop() : Clear buffer of TM action and stop immediately.
- pause() : Pause the action of TM.
- resume() : Resume the action of TM.
### Variables
- __init__pos = [376.0587, -436.61, 734.17, 179.98, 0.0, 45.0]
- __yolo_pos = [405.0, -612.0, 396.0, 90.0, 0.0, 45.0]
- __grab_pos = [-363.24, -801.15, 679.03, 180.0, 0.0, 45.0]
### Function Details
#### set_TMPos(pos, speed=20, line=False)
Set TM position to pos.  
+ Parameters:  
  + pos : A float list which contains 6 number represents [x, y, z, u, v, w]. The units are mm and degree.
  + speed : The speed of TM.
  + line : Move by parabola or line.
#### get_TMPos()
Return TM current position.  
+ Returns : list[6]  
  + Represents [x, y, z, u, v, w] in mm and degree.
## Amm Control class
### Functions
- move(x, z) : Move AMM.
- set_nav_goal(x=0.0, y=0.0, z=1.0) : Set the navigation goal.
- set_nav_speed(speed) : Set the AMM forward and backward speed.
- nav_stop() : Stop the navigation.
- __nav_thread() : Navigation thread.
- __keep_forward() : Forward thread.
- __keep_backward() : Backward thread.
### Variables
- __nav_goal : Navigation goal.
- __speed : Forward and Backward speed.
- nav_worker : QThread object to execute __nav_thread().
- forward_worker : QThread object to execute __keep_forward().
- backward_worker : QThread object to execute __keep_backward().
- twist_pub : Publisher to cmd_vel.
- cancel_pub : Publisher to move_base/cancel.
### Function Details
#### move(x, z) : 
Move AMM.
+ Parameters:
  + x : linear velocity of x.
  + z : angular velocity of z.
#### set_nav_goal(x=0.0, y=0.0, z=1.0) : 
Set the navigation goal.
+ Parameters:
  + x : x coordinate of map.
  + y : y coordinate of map.
  + z : orientation.
#### set_nav_speed(speed) : 
Set the AMM forward and backward speed.
 + Parameters:
  + speed : Set __speed = speed. 
## Gripper Control Object
### Functions
- genCommand(char, command) : Update the command according to the character entered by the user.
- askForCommand(command) : Ask the user for a command to send to the gripper.
- moave(ch) : Requests new command and publish them on th Robotiq2FGripperTobotOutput topic.
- open() : Open gripper.
- close() : Close gripper.
- reset() : Reset gripper.
- active() : Active gripper.
### Variables
- pub : Publisher to Robotiq2FGripper_robotOutput topic.
- command : The command of gripper.
