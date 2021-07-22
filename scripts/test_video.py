#!/usr/bin/env python

"""
Test script to check timing of publishing tobii camera frames to ROS

Assume video is up to date

Takes a few seconds to measure  Adjusts timestamp of
the gaze ROS message to be synced with the frame
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
        self.ipv4_address = rospy.get_param("~ipv4_address", "192.168.1.101")
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

    # def syncTimestamps(self, num_samples=50):
    #     """ Updates `time_offset` by computing average difference between
    #     ROS time and video feed timestamps """
    #
    #     timestamps = np.zeros(num_samples)
    #
    #     for i in range(num_samples):
    #         ret, frame = self.cap.read()
    #         t_frame = self.cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
    #         t_ros = rospy.get_time()
    #         timestamps[i] = t_ros - t_frame
    #
    #     self.time_offset = timestamps.mean()



    def run(self):

        min_val = 0

        while self.cap.isOpened() and not rospy.is_shutdown():
            print("New Loop")

            t1 = time.time() ##################################################
            ret, frame = self.cap.read()
            print(time.time() - t1) ###########################################

            t2 = time.time() ##################################################
            # Gaze data
            data = self.tobii.get_data()
            print(data)
            data_pts = data['pts']
            data_gp  = data['gp']

            time_offset = rospy.get_time() - data_pts['ts'] / 1000000.0
            t_ros_pts = data_pts['ts'] / 1000000.0 + time_offset # Image ROS time
            t_gp_pts = data_gp['ts'] / 1000000.0 + time_offset # Gaze ROS time
            print(time.time() - t2) ###########################################

            if ret == True: # Check for valid image
                height, width = frame.shape[:2]
                t3 = time.time() ##################################################
                # Convert frame to ROS message
                img_msg = self.bridge.cv2_to_imgmsg(frame)#, encoding="passthrough") #"bgr8"
                img_msg.header.frame_id = self.frame_id
                img_msg.header.stamp = rospy.Time.from_sec(t_ros_pts)

                # Publish frame
                self.img_pub.publish(img_msg)
                print(time.time() - t3) ###########################################

                t4 = time.time() ##################################################
                if data_gp['ts'] > 0 and self._prev_ts != data_gp['ts']: # Check for new gaze detection
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
                print(time.time() - t4) ###########################################

            # Press Q on keyboard to exit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


            #
            # if data_gp['ts'] > 0: # Check for gaze detection
            #     # Convert from video timestamp (pts) to gaze data timestamp (ts)
            #     # print(t/1000.0) # current video timestamp
            #     # print(data_pts['pts'] / 100000.0)
            #     # print(data_pts['pts'] / 100000.0)
            #     # # print(data_pts['ts'] / 1000000.0)
            #     offset = data_pts['ts'] / 1000000.0 - data_pts['pts'] / 100000.0
            #     # print(data_gp['ts'] / 1000000.0 - offset) # current gaze timestamp
            #     # print(data_gp['ts'])
            #     # print(data_pts['ts'])
            #     # delta = data_pts['ts'] / 1000000.0 - data_gp['ts'] / 1000000.0
            #     # print(data_pts['ts'] / 1000000.0)
            #     # print(data_gp['ts'] / 1000000.0)
            #     # print(delta) # I THINK THIS IS IT - in this delta, the gaze timestamp is bigger when it's updating regularly
            #
            #     # if delta < min_val:
            #     #     min_val = delta
            #     #     print("Low: ", min_val)
            #     for i in range(1000000):
            #         a = 1+1
            #
            #
            #     # print(t/1000.0)
            #     # print(data_pts['pts'] / 100000.0)
            #
            #     # print()
            #     # print(data_gp['ts'])
            # #
            # # gaze_position = ()
            # #
            # #
            #     gaze_position = (int(data_gp['gp'][0]*width),
            #                      int(data_gp['gp'][1]*height))
            # #
            # #     # Display eye tracking location
            #     cv2.circle(frame, gaze_position, 60, (0,0,255), 5)
            #
            # tobii_frame, corners, ids = computeTagDetections(tobii_frame)
            #
            #
            # # Break the loop
            # else:
            #     break

            # t2 = time.time()
            # Display the resulting frame
            cv2.imshow('Tobii Pro Glasses 2 - Live Scene', frame)
            # print(time.time()-t2)

        # When everything done, release the video capture object
        self.cap.release()

        # Closes all the frames
        cv2.destroyAllWindows()

        self.tobii.stop_streaming()
        self.tobii.close()



if __name__ == "__main__":
    tobii_node = TobiiVideoPublisherNode()
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
