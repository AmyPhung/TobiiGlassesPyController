# tobii_glasses_ros
A ROS package built on top of TobiiGlassesPyController for the Tobii Pro Glasses 2

This repository extends the functionality of [TobiiGlassesPyController]( https://github.com/ddetommaso/TobiiGlassesPyController) so that the gaze data and glasses video stream are available in ROS.

# Usage
Running `roslaunch tobii_glasses_ros bringup.launch` should bring up an OpenCV window with the live camera feed and current gaze position shown.

ROS Topics:
+ `/tobii_camera/image_raw` (sensor_msgs/Image) The video stream from the glasses (without the gaze position annotation)
+ `/tobii_gaze` (geometry_msgs/PointStamped) The 2D gaze position in pixel coordinates relative to the camera feed
