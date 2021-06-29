# live_scene_and_gaze.py : A demo for video streaming and synchronized gaze
#
# Copyright (C) 2018  Davide De Tommaso
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>

# Python Imports
import cv2
import numpy as np

# ROS Imports
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

if hasattr(__builtins__, 'raw_input'):
      input=raw_input

# Relative Imports
from tobiiglassesctrl import TobiiGlassesController

class TobiiGlassesNode():
    def __init__(self, ipv4_address, calibrate=True):
        self.ipv4_address = ipv4_address
        self.tobii = TobiiGlassesController(self.ipv4_address,
                                            video_scene=True)

        # ROS setup
        rospy.init_node("tobii_glasses")
        self.rate = rospy.Rate(10) # 10hz
        self.image_pub = rospy.Publisher("~camera", Image, queue_size=1)

        self.bridge = CvBridge() # For converting between opencv and ROS frames

        # Tobii glasses setup
        if calibrate:
            # TODO: Remove hardcode
            project_id = self.tobii.create_project("Test live_scene_and_gaze.py")
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

        self.cap = cv2.VideoCapture("rtsp://%s:8554/live/scene" % self.ipv4_address)

        # Check if camera opened successfully
        if (self.cap.isOpened()== False):
            print("Error opening video stream or file")

        # Read until video is completed
        self.tobii.start_streaming()

    def run(self):
        while self.cap.isOpened() and not rospy.is_shutdown():
            # Capture frame-by-frame
            ret, frame = self.cap.read()

            if ret == True:
                height, width = frame.shape[:2]
                data_gp  = self.tobii.get_data()['gp']
                if data_gp['ts'] > 0:
                    cv2.circle(frame, (int(data_gp['gp'][0]*width),
                               int(data_gp['gp'][1]*height)), 60, (0,0,255), 5)

                # Display the resulting frame
                # cv2.imshow('Tobii Pro Glasses 2 - Live Scene', frame)

                # Convert frame to ROS message
                msg = self.bridge.cv2_to_imgmsg(frame, encoding="passthrough") #"bgr8"

                # Publish frame
                self.image_pub.publish(msg)

                # Press Q on keyboard to  exit
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                # Keep ROS time
                self.rate.sleep()

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
    tobii_node = TobiiGlassesNode("192.168.1.101", calibrate=True)
    tobii_node.run()
