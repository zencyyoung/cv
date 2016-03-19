# cv
  This is the code can output dense optical flow of the data from the kinect and track the motion of the sequences.

##Dependences
  1. a bag file, collected from the ROS turtlebot Kinect1. if you want the bag file please let me know zency.young@icloud.com
  2. ros install 
##Run 
    $roscore
    $rosbad play --clock [bagname.bag]
    ###go to you catkin workspace 
    $catkin_make
    ####built the ros2opencv target
    $rosrun ros2opencv ros2opencv_node
    
##Q&A
  1.if your package is not installed successful or can not be found,
    check the topic list,and the 
      /camera/data_throttled_camera_info_relay
      /camera/data_throttled_image_depth_relay
    should on the topic list

      $rostopic list 
  or check your environment variables,your catkin_ws workspace path must be added.
    $echo $ROS_PACKAGE_PATH
