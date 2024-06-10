# ~/catkin_ws/src/gopigo3_ultrasound/src/left_sensor_node.py
import rospy
from sensor_msgs.msg import Range
from ultrasonic_sensor import setup_gpio, read_distance

TRIGGER_PIN = 17
ECHO_PIN = 27

def left_sensor_node():
    rospy.init_node('left_sensor_node', anonymous=True)
    pub = rospy.Publisher('left_distance', Range, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    setup_gpio(TRIGGER_PIN, ECHO_PIN)

    while not rospy.is_shutdown():
        distance = read_distance(TRIGGER_PIN, ECHO_PIN)
        range_msg = Range()
        range_msg.header.stamp = rospy.Time.now()
        range_msg.range = distance / 100.0  # Convert to meters
        pub.publish(range_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        left_sensor_node()
    except rospy.ROSInterruptException:
        GPIO.cleanup()
