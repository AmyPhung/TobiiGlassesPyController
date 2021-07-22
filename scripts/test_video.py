import cv2
import numpy as np
import time
from cv_bridge import CvBridge
bridge = CvBridge()


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

# input("Press ENTER to start the video scene")

cap = cv2.VideoCapture("rtsp://%s:8554/live/scene" % ipv4_address)

# Check if camera opened successfully
if (cap.isOpened()== False):
  print("Error opening video stream or file")


start = time.time()
frames = 0

i = 0
offset = 0

# Read until video is completed
while True:
    start = time.time()
    # frames = 0
    # for i in range(25):
    # print("------------------")
    # t0 = time.time()
    ret, frame = cap.read()
    # print(t0-time.time())
    # print(ret)
    # print(time.time()-start)
    # start = time.time()
    # t1 = time.time()
    t = cap.get(cv2.CAP_PROP_POS_MSEC)
    print("Frame time: " + str(t))

    t1 = time.time()
    image_message = bridge.cv2_to_imgmsg(frame)#, desired_encoding='passthrough')
    print(t1-time.time())
    # print(t)
#     print('------')
#     print(cap.get(cv2.CAP_PROP_FPS))
#     print(t)

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

#
# #
# #     height, width = frame.shape[:2]
    # data_gp  = tobiiglasses.get_data()['gp']
    # data_pts = tobiiglasses.get_data()['pts']
    # print(data_pts['pts'])
#     print(data_pts['pts'] - t*100)
    # print(tobiiglasses.get_data())
    data = tobiiglasses.get_data()
    print(data)
    # print(data["pts"]["ts"] - data["pts"]["pts"])

    if i == 0:
        offset = data["pts"]["ts"] - t*1000

    print("Gaze time: " + str(data["pts"]["ts"])) # Gaze
    # print(t*1000 + offset) # Video



    # print(data["pts"]["ts"] - data["pts"]["pts"]*10)
    # print(data["pts"]["ts"] - t*1000)


# #     if offset > 0.0 and offset <= frame_duration:
    # print(data)
    # if data_gp['ts'] > 0:
    data_gp  = data['gp']
    data_pts = data['pts']
    if data_gp['ts'] > 0:
        print("HEREEEEEEEEE")
        # print(data_pts["ts"])
        print(data_pts["pts"]/100000.0)

        offset = data_pts["pts"] - data_pts["ts"]
        print((data_gp["ts"] + offset)/100000.0)

        print(data_pts["pts"]/100000.0 - (data_gp["ts"] + offset)/100000.0)

    # offset = data_gp['ts']/1000000.0 - data_pts['ts']/1000000.0
    # print(offset)

    height, width = frame.shape[:2]

    if data_gp['ts'] > 0:
        cv2.circle(frame,(int(data_gp['gp'][0]*width),int(data_gp['gp'][1]*height)), 30, (0,0,255), 2)
# #     # Display the resulting frame
    t2 = time.time()
    cv2.imshow('Tobii Pro Glasses 2 - Live Scene', frame)
    print(t2-time.time())
#
    i = i+1

    # for i in range(1000000):
    #     a = 1+1

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
