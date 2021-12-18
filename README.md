# EECE5550 Mobile Robotics

## Final Project Submission

### Commands On Raspi :
 step 1 :  connect to raspi
 ssh ubuntu@192.168.1.106

step2 : Do turtlebot bringup
roslaunch turtlebot3_bringup turtlebot3_robot.launch

step3 : Run Camera Node
roslaunch raspicam_node camerav2_1280x960_10fps.launch enable_raw:=true

rosrun tf static_transform_publisher 0.03 0 0.1 -1.57 0 -1.57 base_link camera 100

step4 : Run Apriltag Detection
roslaunch turtlebot3_mr apriltag_gazebo.launch

Check camera feed :
rqt_image_view --force-discover


### Commands on Local:

roscore

step1 : Launch move_base navigation
roslaunch turtlebot3_navigation move_base.launch 

step2 : Launch Gmapping 
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping


step3 : Launch explore_lite
roslaunch explore_lite explore_test.launch



