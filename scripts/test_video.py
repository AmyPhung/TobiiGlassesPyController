import cv2
import numpy as np
import time

if hasattr(__builtins__, 'raw_input'):
      input=raw_input

from tobiiglassesctrl import TobiiGlassesController

# ipv4_address = "192.168.71.50"
ipv4_address = "192.168.1.101"

tobiiglasses = TobiiGlassesController(ipv4_address, video_scene=True)
tobiiglasses.set_video_freq_50()
tobiiglasses.start_streaming()
video_freq = tobiiglasses.get_video_freq()
print(video_freq)

frame_duration = 1000.0/float(video_freq) #frame duration in ms

input("Press ENTER to start the video scene")

cap = cv2.VideoCapture("rtsp://%s:8554/live/scene" % ipv4_address)

# Check if camera opened successfully
if (cap.isOpened()== False):
  print("Error opening video stream or file")


start = time.time()
frames = 0

# Read until video is completed
while True:
    start = time.time()
    # frames = 0
    # for i in range(25):
    ret, frame = cap.read()
    # print(ret)
    # print(time.time()-start)
    # start = time.time()
    t = cap.get(cv2.CAP_PROP_POS_MSEC)
    print(t)
#     print('------')
#     print(cap.get(cv2.CAP_PROP_FPS))
#     print(t)
    cv2.imshow('Tobii Pro Glasses 2 - Live Scene', frame)
#     # ret = cap.grab()
#     # print(ret)
#         # print(ret)
#     #
#     # print(time.time() - start)
#     # start = time.time()
#     # if time.time()-start > 1:
#     #     print(frames/(time.time()-start))
#     #     start = time.time()
#     #     frames=0
#     # else:
#     #   # Capture frame-by-frame
#     #   ret, frame = cap.read()
#     #   print(ret)
#     #   if ret == True:
#     #       frames += 1
    for i in range(1000000):
        a = 1+1
#
# #
# #     height, width = frame.shape[:2]
    # data_gp  = tobiiglasses.get_data()['gp']
    # data_pts = tobiiglasses.get_data()['pts']
    # print(data_pts['pts'])
#     print(data_pts['pts'] - t*100)
    print(tobiiglasses.get_data())
# #     offset = data_gp['ts']/1000000.0 - data_pts['ts']/1000000.0
# #     if offset > 0.0 and offset <= frame_duration:
# #         cv2.circle(frame,(int(data_gp['gp'][0]*width),int(data_gp['gp'][1]*height)), 30, (0,0,255), 2)
# #     # Display the resulting frame
#

    # Press Q on keyboard to  exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
      break
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

tobiiglasses.stop_streaming()
tobiiglasses.close()
