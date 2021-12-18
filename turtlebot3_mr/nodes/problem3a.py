import rospy
from geometry_msgs.msg import Twist
from apriltag_ros import AprilTagDetectionArray
import numpy
import tf
from time import sleep
import scipy
from scipy.linalg import logm, inv

def tag_data(message):
    global tag_camera_transform
    tag_pose = message.detections[0].pose.pose.pose
    translation = [tag_pose.position.x,tag_pose.position.y,tag_pose.position.z]
    quat = [tag_pose.orientation.w, tag_pose.orientation.x, tag_pose.orientation.y, tag_pose.orientation.z]
    rot = scipy.spatial.transform.Rotation.from_quat(quat).as_matrix()
    tag_camera_transform = numpy.array([[rot[0][0],rot[0][1],rot[0][2],translation[0]],[rot[1][0],rot[1][1],rot[1][2],translation[1]],[rot[2][0],rot[2][1],rot[2][2],translation[2]],[0,0,0,1]])
    tag_camera_transform = inv(tag_camera_transform)

def transforms():
    global camera_base_transform, goal_tag_transform
    sleep(5)
    (translation,quat) = listener.lookupTransform('/base_footprint', '/camera_rgb_optical_frame', rospy.Time(0))
    rot = scipy.spatial.transform.Rotation.from_quat(quat).as_matrix()
    camera_base_transform = numpy.array([[rot[0][0],rot[0][1],rot[0][2],translation[0]],[rot[1][0],rot[1][1],rot[1][2],translation[1]],[rot[2][0],rot[2][1],rot[2][2],translation[2]],[0,0,0,1]])
    
    goal_translation = numpy.array([[1,0,0,0],[0,1,0,0],[0,0,1,0.12],[0,0,0,1]])
    goal_rotation = numpy.array([[0,0,-1,0],[-1,0,0,0],[0,1,0,0],[0,0,0,1]])
    goal_tag_transform = numpy.matmul(goal_rotation, goal_translation)

def move_TB():
    goal = numpy.matmul(numpy.matmul(goal_tag_transform,tag_camera_transform),camera_base_transform)
    goal_pose = numpy.array([[goal[0][0],goal[0][1],goal[0][3]],[goal[1][0],goal[1][1],goal[1][3]],[0,0,1]])
    u_dot = (1/5) * logm(inv(goal_pose))
    cmd = Twist()
    cmd.linear.x = u_dot[0][2]
    cmd.angular.z = u_dot[1][0]
    pub.publish(cmd)
    sleep(5)
    cmd = Twist() 
    pub.publish(cmd)

def main():
    global listener, pub
    rospy.init_node('controlTB')
    listener = tf.TransformListener()
    transforms()
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    sub = rospy.Subscriber("/tagDetections", AprilTagDetectionArray, tag_data)
    rospy.Timer(rospy.Duration(0.1), move_TB)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
