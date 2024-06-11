# ~/catkin_ws/src/gopigo3_odometry/src/odometry_node.py
import rospy
import math
import time
from gopigo3 import GoPiGo3
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from std_msgs.msg import Float64
import tf

class OdometryNode:
    def __init__(self):
        rospy.init_node('odometry_node', anonymous=True)

        self.gpg = GoPiGo3()

        self.encoder_left = 0
        self.encoder_right = 0

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.prev_time = time.time()

        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.odom_broadcaster = tf.TransformBroadcaster()

        self.rate = rospy.Rate(10)  # 10 Hz

    def get_encoder_ticks(self):
        return self.gpg.get_motor_encoder(self.gpg.MOTOR_LEFT), self.gpg.get_motor_encoder(self.gpg.MOTOR_RIGHT)

    def compute_odometry(self):
        current_time = time.time()
        dt = current_time - self.prev_time
        self.prev_time = current_time

        new_encoder_left, new_encoder_right = self.get_encoder_ticks()

        d_left = (new_encoder_left - self.encoder_left) * (math.pi * self.gpg.WHEEL_DIAMETER) / 360.0
        d_right = (new_encoder_right - self.encoder_right) * (math.pi * self.gpg.WHEEL_DIAMETER) / 360.0

        self.encoder_left = new_encoder_left
        self.encoder_right = new_encoder_right

        d = (d_left + d_right) / 2.0
        theta = (d_right - d_left) / self.gpg.WHEEL_BASE_WIDTH

        self.x += d * math.cos(self.th + theta / 2.0)
        self.y += d * math.sin(self.th + theta / 2.0)
        self.th += theta

    def publish_odometry(self):
        current_time = rospy.Time.now()

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0),
            odom_quat,
            current_time,
            "base_link",
            "odom"
        )

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        odom.pose.pose = Pose(Point(self.x, self.y, 0), Quaternion(*odom_quat))

        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist()

        self.odom_pub.publish(odom)

    def run(self):
        while not rospy.is_shutdown():
            self.compute_odometry()
            self.publish_odometry()
            self.rate.sleep()

    def cleanup(self):
        self.gpg.reset_all()

if __name__ == '__main__':
    try:
        odometry_node = OdometryNode()
        odometry_node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        odometry_node.cleanup()
