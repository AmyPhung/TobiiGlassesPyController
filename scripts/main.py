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

def computeTagDetections(image):
    # ArUco Tag Detection
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    arucoParams = cv2.aruco.DetectorParameters_create()
    (corners, ids, rejected) = cv2.aruco.detectMarkers(image,
        arucoDict, parameters=arucoParams)

    # Label ArUco tag detections
    # verify *at least* one ArUco marker was detected
    if len(corners) > 0:
    	# flatten the ArUco IDs list
    	ids = ids.flatten()
    	# loop over the detected ArUCo corners
    	for (markerCorner, markerID) in zip(corners, ids):
    		# extract the marker corners (which are always returned in
    		# top-left, top-right, bottom-right, and bottom-left order)
    		corners = markerCorner.reshape((4, 2))
    		(topLeft, topRight, bottomRight, bottomLeft) = corners
    		# convert each of the (x, y)-coordinate pairs to integers
    		topRight = (int(topRight[0]), int(topRight[1]))
    		bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
    		bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
    		topLeft = (int(topLeft[0]), int(topLeft[1]))

    		# draw the bounding box of the ArUCo detection
    		cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
    		cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
    		cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
    		cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
    		# compute and draw the center (x, y)-coordinates of the ArUco
    		# marker
    		cX = int((topLeft[0] + bottomRight[0]) / 2.0)
    		cY = int((topLeft[1] + bottomRight[1]) / 2.0)
    		cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
    		# draw the ArUco marker ID on the image
    		cv2.putText(image, str(markerID),
    			(topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
    			0.5, (0, 255, 0), 2)
    		print("[INFO] ArUco marker ID: {}".format(markerID))
    return image


class TobiiGlassesNode():
    def __init__(self, ipv4_address, calibrate=True):
        self.ipv4_address = ipv4_address
        self.tobii = TobiiGlassesController(self.ipv4_address,
                                            video_scene=True)

        # # ROS setup
        # rospy.init_node("tobii_glasses")
        # self.rate = rospy.Rate(10) # 10hz
        # self.image_pub = rospy.Publisher("~camera", Image, queue_size=1)
        #
        # self.bridge = CvBridge() # For converting between opencv and ROS frames

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

        # Create resizable display window
        cv2.namedWindow('Tobii Pro Glasses 2 - Live Scene', cv2.WINDOW_NORMAL)


    def run(self):
        while self.cap.isOpened():# and not rospy.is_shutdown():
            # Capture frame-by-frame
            ret, frame = self.cap.read()

            if ret == True:
                height, width = frame.shape[:2]
                data_gp  = self.tobii.get_data()['gp']

                if data_gp['ts'] > 0:
                    print(data_gp['gp'][0])
                    # Display eye tracking location
                    cv2.circle(frame, (int(data_gp['gp'][0]*width),
                               int(data_gp['gp'][1]*height)), 60, (0,0,255), 5)

                    frame = computeTagDetections(frame)

                # Display the resulting frame
                cv2.imshow('Tobii Pro Glasses 2 - Live Scene', frame)
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
    tobii_node = TobiiGlassesNode("192.168.1.101", calibrate=True)
    tobii_node.run()
