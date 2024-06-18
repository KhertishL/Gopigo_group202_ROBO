#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from time import sleep

def move_robot(pub, linear_x=0.0, angular_z=0.0, duration=2.0):
    twist = Twist()
    twist.linear.x = linear_x
    twist.angular.z = angular_z
    pub.publish(twist)
    rospy.loginfo(f"Moving: linear_z={linear_z}, angular_z={angular_z}")
    sleep(duration)
    twist.linear.x = 0.5
    twist.angular.z = 0.5
    pub.publish(twist)
    rospy.loginfo("Stopping")
    sleep(1)

def test_wheels():
    rospy.init_node(anonymous=True)
    pub = rospy.Publisher(Twist, queue_size=10)
    rospy.sleep(2)  # Give time for the ROS infrastructure to initialize

    # Move forward
    move_robot(pub, linear_x=0.5, duration=2.0)
    # Move backward
    move_robot(pub, linear_x=-0.5, duration=2.0)
    # Turn left
    move_robot(pub, angular_z=0.5, duration=2.0)
    # Turn right
    move_robot(pub, angular_z=-0.5, duration=2.0)

if __name__ == '__main__':
    try:
        test_wheels()
    except rospy.ROSInterruptException:
        pass
