#GoPiGo3 Differential Drive and Ultrasonic Sensor ROS Package
##Introduction
This ROS package provides a differential drive controller for the GoPiGo3 robot along with support for ultrasonic sensor data acquisition. The package includes launch files and scripts to control the robot's movement and gather distance measurements using ROS.

##Installation
1. Clone this repository into your catkin workspace
'''
cd ~/catkin_ws/src
git clone https://github.com/your_username/gopigo3_ultrasound.git

2. Build the package:
'''
cd ~/catkin_ws
catkin_make

'''
3.Source the setup file:
'''
source devel/setup.bash
'''
4.Launching Differential Drive Controller
To launch the differential drive controller, use roslaunch:
'roslaunch gopigo3_bringup driver_differential.launch'

5.Running main code
'rosrun gopigo3_ultrasound drive2.py'

##Notes
Ensure the GoPiGo3 robot is properly connected and powered on before running the launch files and scripts.
Adjust any necessary parameters in driver_differential.launch or drive2.py to suit your specific setup or requirements.




