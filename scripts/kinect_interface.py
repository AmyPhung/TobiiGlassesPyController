#!/usr/bin/env python
import cv2
import numpy as np
import math

# ROS Imports
import rospy
import ros_numpy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from tobii_glasses.msg import PixelLabeled
from tobii_glasses.msg import PointLabeled




class KinectInterface():
    def __init__(self):

        rospy.init_node("kinect_interface")
        self.rate = rospy.Rate(10) # 10hz
        # self.display_img_sub = rospy.Subscriber("~camera", Image, self.display_img_callback)
        # self.kinect_img_sub = rospy.Subscriber("/kinect2/sd/image_color_rect", Image, self.kinectImgCallback)
        self.kinect_pts_sub = rospy.Subscriber("/kinect2/qhd/points", PointCloud2, self.kinectPtsCallback)
        self.gaze_pos_sub = rospy.Subscriber("gaze_pixel_position", PixelLabeled, self.gazePosCallback)

        # TODO: make this PointLabeled
        self.point_pub = rospy.Publisher("gaze_point_position", PointStamped, queue_size=1)

        # self.kinect_img_msg = None
        self.kinect_pts_msg = None
        self.kinect_pts_np = None
        self.gaze_pos_msg = None

        # TODO: use approximate time synchronizer

    # def kinectImgCallback(self, msg):
    #     self.kinect_img_msg = msg

    def kinectPtsCallback(self, msg):
        self.kinect_pts_msg = msg
        self.kinect_pts_np = ros_numpy.numpify(msg)

    def gazePosCallback(self, msg):
        # Ensure gaze message can't pass outside bounds
        if msg.x < 0:
            msg.x = 0
        elif msg.x >= msg.img_width:
            msg.x = msg.img_width - 1
        if msg.y < 0:
            msg.y = 0
        elif msg.y >= msg.img_height:
            msg.y = msg.img_height - 1

        self.gaze_pos_msg = msg

    def findPointInCloud(self, kinect_pts, gaze_msg):
        pt = kinect_pts[gaze_msg.img_height-gaze_msg.y-1][gaze_msg.img_width-gaze_msg.x-1]

        # Check to make sure 3D point is valid
        if not any(filter(math.isnan, pt)):
            pt_msg = Point()
            pt_msg.x, pt_msg.y, pt_msg.z = pt[0], pt[1], pt[2]
            return pt_msg

        return None

    def run(self):
        while not rospy.is_shutdown():
            # TODO: add error checking (in case image height and width don't match)
            if self.kinect_pts_msg and self.gaze_pos_msg:
                pt_msg = self.findPointInCloud(self.kinect_pts_np, self.gaze_pos_msg)
                if pt_msg:
                    pt_msg_stamped = PointStamped()
                    pt_msg_stamped.header = self.gaze_pos_msg.header
                    pt_msg_stamped.point = pt_msg
                    self.point_pub.publish(pt_msg_stamped)

            self.rate.sleep()

if __name__=="__main__":
    kinect_interface = KinectInterface()
    kinect_interface.run()
