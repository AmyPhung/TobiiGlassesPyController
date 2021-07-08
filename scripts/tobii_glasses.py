#!/usr/bin/env python
"""
tobii_glasses.py

Amy Phung
Last Updated: 7/6/2021

This script contains an ROS node that connects to the tobii glasses and
publishes the relative location (in percentage of image height and width)
within the image the user is looking at. Assumes that the image is being
displayed using the ArucoWindow() object (from aruco_interface.py). Uses the
ArUco tags in the image to compute the relative position of the eye tracking
cursor with respect to the image frame.

Note: This was tested using Ubuntu 18.04 and ROS Melodic with the
Tobii Pro Glasses 2
"""

# Python Imports
import cv2
import numpy as np

# ROS Imports
import rospy
from sensor_msgs.msg import Image
from tobii_glasses.msg import PixelLabeled

if hasattr(__builtins__, 'raw_input'):
      input=raw_input

# Relative Imports
from tobiiglassesctrl import TobiiGlassesController
from aruco_interface import ArucoWindow
from helper_functions import computeTagDetections
from helper_functions import computeGazePixel
from cv_bridge import CvBridge


class TobiiGlassesNode():
    def __init__(self, ipv4_address, calibrate=True):
        self.ready = False
        self.ipv4_address = ipv4_address
        self.tobii = TobiiGlassesController(self.ipv4_address,
                                            video_scene=True)

        # ROS setup
        rospy.init_node("tobii_glasses")
        self.rate = rospy.Rate(10) # 10hz
        # self.display_img_sub = rospy.Subscriber("~camera", Image, self.display_img_callback)
        self.display_img_sub = rospy.Subscriber("/kinect2/sd/image_color_rect", Image, self.display_img_callback)
        self.display_img_msg = None
        self.cursor_pub = rospy.Publisher("gaze_pixel_position", PixelLabeled, queue_size=1)

        self.bridge = CvBridge()
        # self.image_pub = rospy.Publisher("~camera", Image, queue_size=1)

        if rospy.has_param("aruco_params"): # Load ROS parameters
            self.params = rospy.get_param('aruco_params')
            print(self.params)
        else:
            self.params = {'image_rescale': 1,
                           'tag_size': 100,
                           'tag_border': 10}

        # Tobii glasses setup
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
        cv2.namedWindow('Tobii Pro Glasses 2 - Live Scene', cv2.WINDOW_NORMAL)

        # Create window for image display
        self.display_window = ArucoWindow(tag_size=self.params['tag_size'],
                                          tag_border=self.params['tag_border'],
                                          image_rescale=self.params['image_rescale'])

        # TODO: Remove - temporary test
        self.display_frame = cv2.imread("img/sample_img.jpg")
        self.display_window.updateFrame(self.display_frame)

        self.tags = {0:[], 1:[], 2:[], 3:[]}

        self.ready = True

    def display_img_callback(self, msg):
        # Save ROS messsage
        self.display_img_msg = msg

        # Convert image from ROS message to cv frame
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # TODO: Remove this
        # Rotate image (needed for kinect)
        cv_image = cv2.rotate(cv_image, cv2.ROTATE_180)

        # Save cv frame
        self.display_frame = cv_image

        # Update visualization
        if self.ready:
            self.display_window.updateFrame(cv_image)

    def updateTagDetections(self, corners, ids):
        for (markerCorner, markerID) in zip(corners, ids):
            if markerID in self.tags:
                self.tags[markerID] = markerCorner

    def verifyData(self, tags, gaze_position):
        # TODO: only allow this computation if timestamps are synced (within reason)
        if not len(gaze_position):
            return False
        for tag in tags:
            if not len(tags[tag]):
                return False
        return True

    def createCursorMsg(self, x, y, img_height, img_width):
        cursor_msg = PixelLabeled()

        cursor_msg.header.stamp = rospy.Time.now()
        if self.display_img_msg:
            cursor_msg.header.frame_id = self.display_img_msg.header.frame_id

        cursor_msg.x = x
        cursor_msg.y = y
        cursor_msg.img_height = img_height
        cursor_msg.img_width = img_width

        return cursor_msg

    def run(self):
        while self.cap.isOpened() and not rospy.is_shutdown():
            # Capture frame-by-frame from Tobii glasses
            # Flush buffer
            for i in range(2):
                ret, tobii_frame = self.cap.read()
            ret, tobii_frame = self.cap.read()

            if ret == True:
                height, width = tobii_frame.shape[:2]
                data_gp  = self.tobii.get_data()['gp']
                gaze_position = ()

                if data_gp['ts'] > 0: # Check for gaze detection
                    gaze_position = (int(data_gp['gp'][0]*width),
                                     int(data_gp['gp'][1]*height))

                    # Display eye tracking location
                    cv2.circle(tobii_frame, gaze_position, 60, (0,0,255), 5)

                tobii_frame, corners, ids = computeTagDetections(tobii_frame)

                if len(corners):
                    self.updateTagDetections(corners, ids)

                # Only compute gaze position if all the data required
                if self.verifyData(self.tags, gaze_position):
                # if len(corners) and len(gaze_position):# TODO: only allow this if there are 4 tags
                    gaze_tobii_pos, gaze_window_pos, ctrl_corners_tobii, ctrl_corners_window = computeGazePixel(self.display_frame,
                        tobii_frame, self.tags, self.params, gaze_position)

                    # TODO: compute actual image pixel

                    # Publish result to ROS
                    # cursor_msg.header
                    self.createCursorMsg(gaze_window_pos[0], gaze_window_pos[1],
                                         10,3) #  TODO: THIS IS TEMPORARY
                    self.cursor_pub.publish(cursor_msg)

                    #TEMPORARY VISUALIZATION
                    cv2.circle(tobii_frame, (gaze_tobii_pos[0], gaze_tobii_pos[1]), 10, (0,255,255), -1)
                    for corner in ctrl_corners_tobii:
                        cv2.circle(tobii_frame, tuple(corner), 3, (255, 0, 255), -1)

                    # Update cursor in display window
                    self.display_window.updateCursor(gaze_window_pos[0], gaze_window_pos[1], ctrl_corners_window)

                # Display the resulting frame
                cv2.imshow('Tobii Pro Glasses 2 - Live Scene', tobii_frame)
                #
                # # Convert frame to ROS message
                # msg = self.bridge.cv2_to_imgmsg(frame, encoding="passthrough") #"bgr8"
                #
                # # Publish frame
                # self.image_pub.publish(msg)
                #
                # Press Q on keyboard to  exit
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                #
                # # Keep ROS time
                # self.rate.sleep()

            # Break the loop
            else:
                break

        # When everything done, release the video capture object
        self.cap.release()

        # Closes all the framesstd_msgs
        cv2.destroyAllWindows()

        self.tobii.stop_streaming()
        self.tobii.close()



if __name__ == "__main__":
    tobii_node = TobiiGlassesNode("192.168.1.101", calibrate=False)
    tobii_node.run()
