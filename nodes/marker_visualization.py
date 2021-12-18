import rospy, tf
import numpy as np
from geometry_msgs.msg import Twist, Point
from apriltag_ros.msg import AprilTagDetectionArray
from scipy.linalg import inv, logm
from time import sleep
from scipy.spatial.transform import Rotation as Rotation
from nav_msgs.msg import Odometry
from math import atan2
from tf.transformations import euler_from_quaternion

control_pub = None
listener = None

def main():
    global control_pub, listener
    rospy.init_node('controlTB')
    listener = tf.TransformListener()
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray,tag_data,queue_size=1)
    rospy.Subscriber("/odom", Odometry, newOdom)

    control_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        rospy.Timer(rospy.Duration(0.1), move_TB)
        # move_TB()
        # rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass