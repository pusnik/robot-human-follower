robot-human-follower
====================

Human detection and tracking system created for my bachelor thesis.

System is based on ROS - Robot Operating System, mobile
robot IRobot Roomba and RGBD sensor Kinect. For human detection we
first use the depth information of the Kinect and HOG algorithm for the
initial classication. For re-detection, algorithm narrows search window and
then for classication, resorts to more robust online Adaboost algorithm with
Haar features. For cases where we do not positively classify or lose the human
we use predictions of the Kalman filter. For robot navigation we used ROS
navigation stack.

System was tested on ROS Electric, Ubuntu 11.10, and IRobot Roomba 555 with Microsoft Kinect.


Robot in action can be seen here:

http://www.youtube.com/watch?v=DdaAWBreoqA&feature=plcp