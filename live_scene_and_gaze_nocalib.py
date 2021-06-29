# live_scene_and_gaze.py : A demo for video streaming and synchronized gaze
#
# Copyright (C) 2021  Davide De Tommaso
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

import av
import cv2
import numpy as np
import pickle
from tobiiglassesctrl import TobiiGlassesController
import sys
import struct
import base64
import time

# ipv4_address = "192.168.100.10"
tobiiglasses = TobiiGlassesController("fe80::76fe:48ff:fe36:fa20%enx00e09c1019d9", video_scene=True)
tobiiglasses.start_streaming()

# #
# # #
# # # # cap = cv2.VideoCapture("rtsp://[fe80::76fe:48ff:fe36:fa20%wlp3s0]:8554/live/scene")
# # cap = cv2.VideoCapture("rtsp://[fe80::76fe:48ff:fe36:fa20]:49152/live/scene")
# cap = cv2.VideoCapture("rtsp://[fe80::76fe:48ff:fe36:fa20%enx00e09c1019d9]:8554/live/scene")
# print("HEERE!")
#
try:
    while True:
        pass
except KeyboardInterrupt:
    print('interrupted!')
#
#
# while(cap.isOpened()):
#   # Capture frame-by-frame
#   ret, frame = cap.read()
#   if ret == True:
#     height, width = frame.shape[:2]
#     data_gp  = tobiiglasses.get_data()['gp']
#     if data_gp['ts'] > 0:
#         cv2.circle(frame,(int(data_gp['gp'][0]*width),int(data_gp['gp'][1]*height)), 60, (0,0,255), 5)
#
#     # Display the resulting frame
#     cv2.imshow('Tobii Pro Glasses 2 - Live Scene',frame)
#
#     # Press Q on keyboard to  exit
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#       break
#
#   # Break the loop
#   else:
#     break
#
# # When everything done, release the video capture object
# cap.release()
#
# # Closes all the frames
# cv2.destroyAllWindows()




#
# conn = tobiiglasses.video_socket
#
# # while True:
# #     data = conn.recv(4096)
# #     print(data)
# #     # print ("From: " + str(address[0]) + " " + data)
# #     print sys.getsizeof(data)
# #     frame=pickle.loads(data)
# #     # print frame
# #     cv2.imshow('frame',frame)
#
# data = b""
# payload_size = struct.calcsize(">L")
# print("payload_size: {}".format(payload_size))
# while True:
#     while len(data) < payload_size:
#         data += conn.recv(4096)
#     # receive image row data form client socket
#     packed_msg_size = data[:payload_size]
#     data = data[payload_size:]
#     msg_size = struct.unpack(">L", packed_msg_size)[0]
#     while len(data) < msg_size:
#         data += conn.recv(4096)
#     frame_data = data[:msg_size]
#     data = data[msg_size:]
#     # unpack image using pickle
#     frame=pickle.loads(frame_data, fix_imports=True, encoding="bytes")
#     frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
#     frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
#     # Resize frame
#     bimg = ImageOps.expand(Image.fromarray(frame), border=(0, 80, 0, 80)).resize((256, 256), Image.ANTIALIAS)
#     image = np.array(bimg)
#     # show realtime image by PIL
#     imshow(frame)
#     show()
#     # Display the frame until new frame is available
#     clear_output(wait=True)
#
#
# frame = conn.recv_string()
# img = base64.b64decode(frame)
# npimg = np.fromstring(img, dtype=np.uint8)
# source = cv2.imdecode(npimg, 1)
# cv2.imshow("Stream", source)
# cv2.waitKey(1)

# data = ""
# payload_size = struct.calcsize("H")
# while True:
#     while len(data) < payload_size:
#         data += conn.recv(4096)
#     packed_msg_size = data[:payload_size]
#     data = data[payload_size:]
#     msg_size = struct.unpack("H", packed_msg_size)[0]
#     while len(data) < msg_size:
#         data += conn.recv(4096)
#     frame_data = data[:msg_size]
#     data = data[msg_size:]
#     ###
#
#     frame=pickle.loads(frame_data)
#     print frame
#     cv2.imshow('frame',frame)

tobiiglasses.stop_streaming()
tobiiglasses.close()

#
# cap = cv2.VideoCapture("rtsp://0.0.0.0:8554/live/scene")
#
#
# # cap = cv2.VideoCapture("rtsp://[fe80::76fe:48ff:fe36:fa20]:8554/live/scene")
#
# print("DONE!")
# # cap = cv2.VideoCapture("rtsp://[fe80::76fe:48ff:fe36:fa20]/live/scene.cgi?.mjpg")
#
# # tobiiglasses = TobiiGlassesController(ipv4_address)
# tobiiglasses = TobiiGlassesController(video_scene=True)
#
# # video = av.open("rtsp://%s:8554/live/scene" % ipv4_address, "r")
# video = av.open("rtsp://[fe80::76fe:48ff:fe36:fa20]:8554/live/scene", "r")
# # video = av.open("rtsp://127.0.0.1:8554/live/scene", "r")
# tobiiglasses.start_streaming()
#
# try:
#     for packet in video.demux():
#         for frame in packet.decode():
#             if isinstance(frame,av.video.frame.VideoFrame):
#                 #print(frame.pts)
#                 img = frame.to_ndarray(format='bgr24')
#                 height, width = img.shape[:2]
#                 data_gp  = tobiiglasses.get_data()['gp']
#                 if data_gp['ts'] > 0:
#                     cv2.circle(img,(int(data_gp['gp'][0]*width),int(data_gp['gp'][1]*height)), 60, (0,0,255), 6)
#                 cv2.imshow('Tobii Pro Glasses 2 - Live Scene',img)
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break
# except KeyboardInterrupt:
#     pass
# cv2.destroyAllWindows()
#
# tobiiglasses.stop_streaming()
# tobiiglasses.close()
