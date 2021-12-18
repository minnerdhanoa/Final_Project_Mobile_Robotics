import rospy
from geometry_msgs.msg import Twist
from time import sleep, time

from rospy.rostime import Time

control_pub = None
# current_time = Time
# time_difference = Time
# def move_TB(event):
#     global current_time, time_difference
#     time_difference = rospy.Time.now() - current_time
#     if time_difference >= 5:
#         print("in loop")
#         cmd = Twist()
#         cmd.linear.x = 0.0
#         cmd.angular.z = 0.5
#         sleep(5)
#         control_pub.publish(cmd)
#         current_time = rospy.Time.now()
    
def main():
    global control_pub, time_difference, current_time
    rospy.init_node('controlTB')
    control_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(5)
    rate.sleep()

    while not rospy.is_shutdown():
    #     rospy.Timer(rospy.Duration(0.1), move_TB)
    #     # move_TB()
    #     # rate.sleep()
    # rospy.spin()
        # time_difference = Time(rospy.Time.now() - current_time)
        # if time_difference >= 5:
            print("in loop")
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 2.0
            sleep(5)
            control_pub.publish(cmd)
            current_time = rospy.Time.now()
            rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
