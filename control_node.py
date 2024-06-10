# ~/catkin_ws/src/gopigo3_ultrasound/src/control_node.py
import rospy
from sensor_msgs.msg import Range
from gopigo3 import GoPiGo3

# Threshold distance in meters (10 cm)
OBSTACLE_THRESHOLD = 0.1

class GoPiGo3Control:
    def __init__(self):
        self.gpg = GoPiGo3()
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')

        rospy.init_node('control_node', anonymous=True)
        rospy.Subscriber('front_distance', Range, self.front_callback)
        rospy.Subscriber('left_distance', Range, self.left_callback)
        rospy.Subscriber('right_distance', Range, self.right_callback)

        self.rate = rospy.Rate(10)  # 10 Hz

    def front_callback(self, data):
        self.front_distance = data.range

    def left_callback(self, data):
        self.left_distance = data.range

    def right_callback(self, data):
        self.right_distance = data.range

    def move_robot(self):
        while not rospy.is_shutdown():
            if self.front_distance > OBSTACLE_THRESHOLD:
                # No obstacle in front, move forward
                self.gpg.set_motor_dps(self.gpg.MOTOR_LEFT + self.gpg.MOTOR_RIGHT, 300)
            else:
                # Obstacle detected within 10 cm, decide to turn
                self.gpg.set_motor_dps(self.gpg.MOTOR_LEFT + self.gpg.MOTOR_RIGHT, 0)  # Stop

                if self.left_distance > self.right_distance:
                    # Turn left
                    self.gpg.set_motor_dps(self.gpg.MOTOR_LEFT, -150)
                    self.gpg.set_motor_dps(self.gpg.MOTOR_RIGHT, 150)
                else:
                    # Turn right
                    self.gpg.set_motor_dps(self.gpg.MOTOR_LEFT, 150)
                    self.gpg.set_motor_dps(self.gpg.MOTOR_RIGHT, -150)

                # Wait a bit to complete the turn
                rospy.sleep(1)

            self.rate.sleep()

    def cleanup(self):
        self.gpg.set_motor_dps(self.gpg.MOTOR_LEFT + self.gpg.MOTOR_RIGHT, 0)
        self.gpg.reset_all()

if __name__ == '__main__':
    try:
        robot_control = GoPiGo3Control()
        robot_control.move_robot()
    except rospy.ROSInterruptException:
        pass
    finally:
        robot_control.cleanup()
