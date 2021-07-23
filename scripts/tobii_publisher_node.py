#!/usr/bin/env python
"""
tobii_publisher_node.py

Amy Phung
Last Updated: 7/23/2021

This script contains an ROS node that connects to a pair of Tobii Pro
Glasses 2 and publishes the glasses' video feed and 2D gaze pixel location

Note: This was tested using Ubuntu 18.04 and ROS Melodic
"""

# Python Imports
import time
import cv2
import numpy as np

# ROS Imports
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped

if hasattr(__builtins__, 'raw_input'):
      input=raw_input

# Relative Imports
from tobiiglassesctrl import TobiiGlassesController

class TobiiVideoPublisherNode():
    def __init__(self):
        # ROS setup
        rospy.init_node("tobii_video_publisher")

        calibrate = rospy.get_param("~calibrate", False)
        self.ipv4_address = rospy.get_param("~ipv4_address", "192.168.71.50")
        self.frame_id = rospy.get_param("~frame_id", "tobii_camera")

        self.img_pub = rospy.Publisher("tobii_camera/image_raw", Image, queue_size=1)
        self.gaze_pub = rospy.Publisher("tobii_gaze", PointStamped, queue_size=1)
        self.bridge = CvBridge()

        # Tobii glasses setup
        self.tobii = TobiiGlassesController(self.ipv4_address,
                                            video_scene=True)

        if calibrate:
            # TODO: Remove hardcode
            project_id = self.tobii.create_project("Tobii ROS node")
            participant_id = self.tobii.create_participant(project_id, "participant_test")
            calibration_id = self.tobii.create_calibration(project_id, participant_id)

            input("Put the calibration marker in front of the user, then press enter to calibrate")
            self.tobii.start_calibration(calibration_id)

            res = self.tobii.wait_until_calibration_is_done(calibration_id)

            if res is False:
            	print("Calibration failed!")
            	exit(1)
            else:
                print("Calibration complete!")

        # Connect to Tobii glasses camera feed
        self.cap = cv2.VideoCapture("rtsp://%s:8554/live/scene" % self.ipv4_address)

        # Check if camera opened successfully
        if (self.cap.isOpened()== False):
            print("Error opening video stream or file")

        # Read until video is completed
        self.tobii.start_streaming()

        # Create resizable live display window
        # TODO: bind this to a display param
        cv2.namedWindow('Tobii Pro Glasses 2 - Live Scene', cv2.WINDOW_NORMAL)

        # Private Variables
        self._prev_ts = 0 # Used for detecting new gaze data

    def run(self):
        while self.cap.isOpened() and not rospy.is_shutdown():
            ret, frame = self.cap.read()

            # Extract gaze data
            data = self.tobii.get_data()
            data_pts = data['pts']
            data_gp  = data['gp']

            time_offset = rospy.get_time() - data_pts['ts'] / 1000000.0
            t_ros_pts = data_pts['ts'] / 1000000.0 + time_offset # Image ROS time
            t_gp_pts = data_gp['ts'] / 1000000.0 + time_offset # Gaze ROS time

            if ret == True: # Check for valid image
                height, width = frame.shape[:2]
                # Convert frame to ROS message
                img_msg = self.bridge.cv2_to_imgmsg(frame)
                img_msg.header.frame_id = self.frame_id
                img_msg.header.stamp = rospy.Time.from_sec(t_ros_pts)

                # Publish frame
                self.img_pub.publish(img_msg)

                # Check for new gaze detection
                if data_gp['ts'] > 0 and self._prev_ts != data_gp['ts']:
                    # Convert gaze data to ROS message
                    gaze_msg = PointStamped()
                    gaze_msg.header.frame_id = img_msg.header.frame_id
                    gaze_msg.header.stamp = rospy.Time.from_sec(t_gp_pts)
                    gaze_msg.point.x = int(data_gp['gp'][0]*width)
                    gaze_msg.point.y = int(data_gp['gp'][1]*height)

                    # Publish gaze data
                    self.gaze_pub.publish(gaze_msg)

                    # Update previous timestamp
                    self._prev_ts = data_gp['ts']

                    # Display eye tracking location
                    gp = (gaze_msg.point.x, gaze_msg.point.y)
                    cv2.circle(frame, gp, 60, (0,0,255), 5)

            # Press Q on keyboard to exit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # Display the resulting frame
            cv2.imshow('Tobii Pro Glasses 2 - Live Scene', frame)

        # When everything done, release the video capture object
        self.cap.release()

        # Closes all the frames
        cv2.destroyAllWindows()

        self.tobii.stop_streaming()
        self.tobii.close()



if __name__ == "__main__":
    tobii_node = TobiiVideoPublisherNode()
    tobii_node.run()
