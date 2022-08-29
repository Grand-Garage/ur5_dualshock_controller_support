<p align="center">
<img src="./gg-logo.png" width="200">
</p>


# UR5 - Dualshock Controller Support

Control your universal robot directly via a Dualshok Controller (tested with PS4 game controller).<br>
This python script directly communicates with ROS2 by using the moveit2 extension. You can also control the [Hand-e gripper from Robotiq](https://robotiq.com/de/produkte/adaptiver-2-finger-robotergreifer-hand-e).

<br>
<p align="center">
<img src="./moveit2.png" width="300">
</p>

| Package | Link                                                                                  |
|--------------------| ------------------------------------------------------------------------------------- |
| Moveit2            | https://moveit.picknik.ai/humble/index.html                                                 |
| ROS2 Galactic      | https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html                                    |
| UR ROS2 Driver     | https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/ac9f5a5e1d019a8da1007d0e7f857458e4cbcc3e

<br><br>


<br>

## SETUP
Commit of the ros drivers:
ac9f5a5e1d019a8da1007d0e7f857458e4cbcc3e
Will not work with later versions properly! Also some of the later versions are broken!



colcon build --packages-select moveit2_dualshock_controller --symlink-install


### ROS2 installation

Install ROS2 as per installation instructions per your repository. As of time of writing this documentation, the well supported version was ROS2 Galactic. Example instructions for ubuntu can be found [here] https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html).

### Universal Robots ROS2 driver 
Get a Universal Robot ROS2 driver from the official [repository](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) by [Universal Robots](https://github.com/UniversalRobots). The README in the repositroy is pretty exhaustive. Keep in mind, that the driver is in its **beta**, meaning that the cutting edge commit of the driver might not work. The working version that the robot was running at the presentation was commit  [ac9f5a5e1d019a8da1007d0e7f857458e4cbcc3e](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/ac9f5a5e1d019a8da1007d0e7f857458e4cbcc3e). Some of the subsequent versions have been observed to have errors. They do fix them now and then. 

**Before building**, make the following substitutions in to the sources of the following files with files in the ur_driver_extras (uise Ctl+F in the file explorer to locate them):

- ur_macro.xacro : (optional)  : has been modified to consider the dimensions of the installed Robotiq gripper, so that the commands are referenced relative to the point right between the gripper fingers.
- ur_servo.yaml  : (essential) : has been modified for a slightly reduced command frequency and usage of velocity control. 
- ur_control_servo.launch.py : (optional, not admissible for newer versions of the driver) : should be added to the ur_bringup package. **Does not work with fresh versions of the driver (past ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {})** due to their refactoring efforts. Provides simplified launch of the system. Otherwise, the robot driver has to be launched with MoevIt Servo module separately.

You may encounter problems building older commits due to broken dependencies. You would need to fix them yourself, no other way, although it's not difficult (check terminal -> locate broken dependency -> fix the link in the dependency file of UR ROS2 driver).

Make sure to install MoveIt, as instructed in the UR ROS2 Driver README. 

** Play around with the driver, launch driver and moveit as per driver readme and try to send some movement commands! Verify that the current version of the driver works!**

### The ROS2 gripper and controller driver

Checkout this repository into the src directory with other installed modules.

```bash
export CATKIN_WS=~/workspace/ur_ros2_driver/
cd $CATKIN_WS
source install/setup.bash
colcon build --packages-select moveit2_dualshock_controller --symlink-install
```

<br>

## Launch

Activate your ROS2 environment (applies for each termianl window):

```bash
export CATKIN_WS=~/workspace/ur_ros2_driver/
cd $CATKIN_WS
source install/setup.bash
```

### Robot Driver

**If the ur_control_servo launch file installed and works**: Start the driver **with the velocity control selcted**:
```bash
ros2 launch ur_bringup ur_control_servo.launch.py ur_type:=ur5e robot_ip:=192.168.0.102 launch_rviz:=true use_fake_hardware:=false initial_joint_controller:=forward_velocity_controller
```

**Else**:
Follow the instructions in the readme of the driver. You should start the robot driver and then moveit servo module as instructed.

### Gripper Driver and Game controller

Start the gripper driver and the game controller module in a separate terminal:
```bash
ros2 run moveit2_dualshock_controller dualshock_controller_node
```

**Activate the servo module**:
```bash
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}
```

That's it! It should work now!

<br>

## Trivia

The installation will probably czuse hickups a few times. If you are installing the ac9f5a5e1d019a8da1007d0e7f857458e4cbcc3e version of the driver, you will need to fix dependencies.

For launching moveit servo without my custom launch file (which will not work with fresher versions of the driver), try the "launch_servo" option in the moveit launch file.

Test the robot driver, before moving on! If it doesnt work, try a slightly earlier version (preferably after the last "fix dependency commit " in the tree).
