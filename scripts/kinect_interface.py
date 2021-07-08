#!/usr/bin/env python
import cv2
import numpy as np

# ROS Imports
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
import ros_numpy




class KinectInterface():
    def __init__(self):
        rospy.init_node("kinect_interface")
        self.rate = rospy.Rate(10) # 10hz
        # self.display_img_sub = rospy.Subscriber("~camera", Image, self.display_img_callback)
        # TODO: subscriber for cursor position
        self.kinect_img_sub = rospy.Subscriber("/kinect2/sd/image_color_rect", Image, self.kinectImgCallback)
        self.kinect_pts_sub = rospy.Subscriber("/kinect2/sd/points", PointCloud2, self.kinectPtsCallback)

        self.kinect_img_msg = None
        self.kinect_pts_sub = None

        # TODO: use approximate time synchronizer

    def kinectImgCallback(self, msg):
        self.kinect_img_msg = msg
        print("img")
        print(len(msg.data))

    def kinectPtsCallback(self, msg):
        self.kinect_pts_msg = msg

        pc = ros_numpy.numpify(msg)
        # points=np.zeros((pc.shape[0],3))
        print("here")
        print(pc['x'].shape)
        # points[:,0]=pc['x']
        # points[:,1]=pc['y']
        # points[:,2]=pc['z']
        # p = pcl.PointCloud(np.array(points, dtype=np.float32))
        # print("pcl")
        # print(len(msg.data))
        # print(p.shape)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__=="__main__":
    kinect_interface = KinectInterface()
    kinect_interface.run()
