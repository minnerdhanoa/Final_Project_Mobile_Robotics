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

camera_base_transform = None 
tag_camera_transform = None 
goal_tag_transform = None 
x = 0.0
y = 0.0
theta = 0.0
goal_reached = False

def tag_data(message):
    global tag_camera_transform
    if tag_camera_transform is not None:
        return
    try:
        tag_pose = message.detections[0].pose.pose.pose
        translation = [tag_pose.position.x,tag_pose.position.y,tag_pose.position.z]
        quar = [tag_pose.orientation.w, tag_pose.orientation.x, tag_pose.orientation.y, tag_pose.orientation.z]
        rot = Rotation.from_quat(quar).as_matrix()
        tag_camera_transform = np.array([[rot[0][0],rot[0][1],rot[0][2],translation[0]],[rot[1][0],rot[1][1],rot[1][2],translation[1]],[rot[2][0],rot[2][1],rot[2][2],translation[2]],[0,0,0,1]])
        tag_camera_transform = inv(tag_camera_transform)
    except:
        return

def transforms():
    global camera_base_transform,goal_tag_transform
    sleep(2)
    (translation,quar) = listener.lookupTransform('/base_footprint', '/camera_rgb_optical_frame', rospy.Time(0))
    rot = Rotation.from_quat(quar).as_matrix()
    camera_base_transform = np.array([[rot[0][0],rot[0][1],rot[0][2],translation[0]],[rot[1][0],rot[1][1],rot[1][2],translation[1]],[rot[2][0],rot[2][1],rot[2][2],translation[2]],[0,0,0,1]])
    T_translation = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0.12],[0,0,0,1]])
    T_rotation = np.array([[0,0,-1,0],[-1,0,0,0],[0,1,0,0],[0,0,0,1]])
    goal_tag_transform = np.matmul(T_rotation, T_translation)

def newOdom(message):
    global x, y, theta
    x = message.pose.pose.position.x
    y = message.pose.pose.position.y
    rot_q = message.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    print(x)

# def move_TB(event):
#     global goal_reached
#     # if goal_reached: return
#     if camera_base_transform is not None and tag_camera_transform is not None and goal_tag_transform is not None:
#         goal = np.matmul(np.matmul(goal_tag_transform,tag_camera_transform),camera_base_transform)
#         goal_pose = np.array([[goal[0][0],goal[0][1],goal[0][3]],[goal[1][0],goal[1][1],goal[1][3]],[0,0,1]])
#         u_dot = (1/5) * logm(inv(goal_pose))

#         goal_final = Point()
#         goal_final.x = 0.88
#         goal_final.y = 0

#         while goal_reached is not True:
#             inc_x = goal_final.x - x
#             inc_y = goal_final.y - y
#             angle_to_goal = atan2(inc_y, inc_x)

#             if abs(angle_to_goal - theta) > 0.1:
#                 cmd = Twist()
#                 cmd.linear.x = u_dot[0][2]
#                 cmd.angular.z = u_dot[1][0]
#                 control_pub.publish(cmd)

#             else:
#                 goal_reached = True
#                 print('Goal Reached')
#                 cmd = Twist()
#                 control_pub.publish(cmd)

def move_TB(event):
    global goal_reached
    if goal_reached: return
    if camera_base_transform is not None and tag_camera_transform is not None and goal_tag_transform is not None:
        goal_reached = True
        goal = np.matmul(np.matmul(goal_tag_transform,tag_camera_transform),camera_base_transform)
        goal_pose = np.array([[goal[0][0],goal[0][1],goal[0][3]],[goal[1][0],goal[1][1],goal[1][3]],[0,0,1]])
        u_dot = (1/5) * logm(inv(goal_pose))
        print(u_dot)
        cmd = Twist()
        print(u_dot[0][2])
        cmd.linear.x = u_dot[0][2]
        print(u_dot[1][0])
        cmd.angular.z = u_dot[1][0]
        control_pub.publish(cmd)
        sleep(3.4)
        print('Goal reached')
        cmd = Twist() 
        control_pub.publish(cmd)


def main():
    global control_pub, listener
    rospy.init_node('controlTB')
    listener = tf.TransformListener()
    transforms()
    rospy.Subscriber("/tag_detections",AprilTagDetectionArray,tag_data,queue_size=1)
    rospy.Subscriber("/odometry/filtered", Odometry, newOdom,queue_size=1)

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