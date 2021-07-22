#!/usr/bin/env python

"""
Test script to check timing of publishing tobii camera frames to ROS
"""

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
import time
import cv2
import numpy as np

# ROS Imports
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from tobii_glasses.msg import PixelLabeled

if hasattr(__builtins__, 'raw_input'):
      input=raw_input

# Relative Imports
from tobiiglassesctrl import TobiiGlassesController
from aruco_interface import ArucoWindow
from helper_functions import computeTagDetections
from helper_functions import computeGazePixel



class TobiiVideoPublisherNode():
    def __init__(self, ipv4_address, calibrate=True):
        self.ready = False
        self.ipv4_address = ipv4_address
        self.tobii = TobiiGlassesController(self.ipv4_address,
                                            video_scene=True)

        # ROS setup
        rospy.init_node("tobii_video_publisher")
        self.img_pub = rospy.Publisher("tobii_camera", Image, queue_size=1)

        self.bridge = CvBridge()

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
        # TODO: bind this to a display param
        cv2.namedWindow('Tobii Pro Glasses 2 - Live Scene', cv2.WINDOW_NORMAL)

        self.ready = True

    def run(self):
        while self.cap.isOpened() and not rospy.is_shutdown():
            print("New Loop")

            # TODO: Decide if buffer flush is needed
            t1 = time.time()
            ret, frame = self.cap.read()
            print(time.time()-t1)

            if ret == True:
                height, width = frame.shape[:2]

                t2 = time.time()
                # Display the resulting frame
                cv2.imshow('Tobii Pro Glasses 2 - Live Scene', frame)
                print(time.time()-t2)

                t3 = time.time()
                # Convert frame to ROS message
                msg = self.bridge.cv2_to_imgmsg(frame)#, encoding="passthrough") #"bgr8"
                print(time.time()-t3)

                t4 = time.time()
                # Publish frame
                self.img_pub.publish(msg)
                print(time.time()-t4)

                # Press Q on keyboard to  exit
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            # Break the loop
            else:
                break

        # When everything done, release the video capture object
        self.cap.release()

        # Closes all the frames
        cv2.destroyAllWindows()

        self.tobii.stop_streaming()
        self.tobii.close()



if __name__ == "__main__":
    tobii_node = TobiiVideoPublisherNode("192.168.1.101", calibrate=False)
    tobii_node.run()


# import cv2
# import numpy as np
# import time
#
# # ROS Imports
# import rospy
# from cv_bridge import CvBridge
# bridge = CvBridge()
#
# if hasattr(__builtins__, 'raw_input'):
#       input=raw_input
#
# # Relative imports
# from tobiiglassesctrl import TobiiGlassesController
#
#
# # TODO: evaluate whether threading is necessary
#
# class TobiiVideoPublisherNode()
#     def __init__(self, ipv4_address, calibrate=True):
#
#
# def
# ipv4_address = "192.168.1.101"
#
# tobiiglasses = TobiiGlassesController(ipv4_address, video_scene=True)
# tobiiglasses.start_streaming()
#
# # input("Press ENTER to start the video scene")
#
# cap = cv2.VideoCapture("rtsp://%s:8554/live/scene" % ipv4_address)
#
# # Check if camera opened successfully
# if (cap.isOpened()== False):
#   print("Error opening video stream or file")
#
# # Read until video is completed
# while True:
#     start = time.time()
#     ret, frame = cap.read()
#     # Read frame timestamp
#     t = cap.get(cv2.CAP_PROP_POS_MSEC)
#     print("Frame time: " + str(t))
#
#     # Convert to ROS message
#     t1 = time.time()
#     image_message = bridge.cv2_to_imgmsg(frame)#, desired_encoding='passthrough')
#     print(t1-time.time())
#     # print(t)
# #     print('------')
# #     print(cap.get(cv2.CAP_PROP_FPS))
# #     print(t)
#
# #     # ret = cap.grab()
# #     # print(ret)
# #         # print(ret)
# #     #
# #     # print(time.time() - start)
# #     # start = time.time()
# #     # if time.time()-start > 1:
# #     #     print(frames/(time.time()-start))
# #     #     start = time.time()
# #     #     frames=0
# #     # else:
# #     #   # Capture frame-by-frame
# #     #   ret, frame = cap.read()
# #     #   print(ret)
# #     #   if ret == True:
# #     #       frames += 1
#
# #
# # #
# # #     height, width = frame.shape[:2]
#     # data_gp  = tobiiglasses.get_data()['gp']
#     # data_pts = tobiiglasses.get_data()['pts']
#     # print(data_pts['pts'])
# #     print(data_pts['pts'] - t*100)
#     # print(tobiiglasses.get_data())
#     data = tobiiglasses.get_data()
#     print(data)
#     # print(data["pts"]["ts"] - data["pts"]["pts"])
#
#     if i == 0:
#         offset = data["pts"]["ts"] - t*1000
#
#     print("Gaze time: " + str(data["pts"]["ts"])) # Gaze
#     # print(t*1000 + offset) # Video
#
#
#
#     # print(data["pts"]["ts"] - data["pts"]["pts"]*10)
#     # print(data["pts"]["ts"] - t*1000)
#
#
# # #     if offset > 0.0 and offset <= frame_duration:
#     # print(data)
#     # if data_gp['ts'] > 0:
#     data_gp  = data['gp']
#     data_pts = data['pts']
#     if data_gp['ts'] > 0:
#         print("HEREEEEEEEEE")
#         # print(data_pts["ts"])
#         print(data_pts["pts"]/100000.0)
#
#         offset = data_pts["pts"] - data_pts["ts"]
#         print((data_gp["ts"] + offset)/100000.0)
#
#         print(data_pts["pts"]/100000.0 - (data_gp["ts"] + offset)/100000.0)
#
#     # offset = data_gp['ts']/1000000.0 - data_pts['ts']/1000000.0
#     # print(offset)
#
#     height, width = frame.shape[:2]
#
#     if data_gp['ts'] > 0:
#         cv2.circle(frame,(int(data_gp['gp'][0]*width),int(data_gp['gp'][1]*height)), 30, (0,0,255), 2)
# # #     # Display the resulting frame
#     t2 = time.time()
#     cv2.imshow('Tobii Pro Glasses 2 - Live Scene', frame)
#     print(t2-time.time())
# #
#     i = i+1
#
#     # for i in range(1000000):
#     #     a = 1+1
#
#     # Press Q on keyboard to  exit
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#       break
# #
# #   # Break the loop
# #   else:
# #     break
# #
# # # When everything done, release the video capture object
# # cap.release()
# #
# # # Closes all the frames
# # cv2.destroyAllWindows()
#
# tobiiglasses.stop_streaming()
# tobiiglasses.close()
