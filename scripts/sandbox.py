import cv2
import numpy as np

TAG_SIZE = 100 # px
TAG_BORDER = 10 # px
IMAGE_RESCALE = 0.5 # %

# Load image
input_img = cv2.imread('sample_img3.jpg')#, cv2.IMREAD_GRAYSCALE)

# Manually to grayscale (needed to fix dimension issue)
input_img = 0.2989 * input_img[:,:,0] + \
            0.5870 * input_img[:,:,1] + \
            0.1140 * input_img[:,:,2]
print(input_img.shape)

# Resize image
img_height = int(input_img.shape[0]*IMAGE_RESCALE)
img_width  = int(input_img.shape[1]*IMAGE_RESCALE)
dim = (img_width, img_height)
resized = cv2.resize(input_img, dim, interpolation = cv2.INTER_AREA)

# Allocate space for output image
output_img = np.ones((img_height + 2*TAG_SIZE + 4*TAG_BORDER,
                      img_width + 2*TAG_SIZE + 4*TAG_BORDER,
                      1), dtype="uint8")*255 # 255 for white background

# Get ArUco tags
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

# Create tags
ul_tag = np.zeros((TAG_SIZE, TAG_SIZE, 1), dtype="uint8") # Upper left
ur_tag = np.zeros((TAG_SIZE, TAG_SIZE, 1), dtype="uint8") # Upper right
bl_tag = np.zeros((TAG_SIZE, TAG_SIZE, 1), dtype="uint8") # Bottom left
br_tag = np.zeros((TAG_SIZE, TAG_SIZE, 1), dtype="uint8") # Bottom right
cv2.aruco.drawMarker(arucoDict, 0, TAG_SIZE, ul_tag, 1)
cv2.aruco.drawMarker(arucoDict, 1, TAG_SIZE, ur_tag, 1)
cv2.aruco.drawMarker(arucoDict, 2, TAG_SIZE, bl_tag, 1)
cv2.aruco.drawMarker(arucoDict, 3, TAG_SIZE, br_tag, 1)

# Put tags in output image
output_img[TAG_BORDER:TAG_SIZE+TAG_BORDER,
           TAG_BORDER:TAG_SIZE+TAG_BORDER] \
           = ul_tag
output_img[TAG_BORDER:TAG_SIZE+TAG_BORDER,
           TAG_BORDER+TAG_SIZE+TAG_BORDER+img_width+TAG_BORDER:-TAG_BORDER] \
           = ur_tag
output_img[TAG_BORDER+TAG_SIZE+TAG_BORDER+img_height+TAG_BORDER:-TAG_BORDER,
           TAG_BORDER:TAG_SIZE+TAG_BORDER] \
           = bl_tag
output_img[TAG_BORDER+TAG_SIZE+TAG_BORDER+img_height+TAG_BORDER:-TAG_BORDER,
           TAG_BORDER+TAG_SIZE+TAG_BORDER+img_width+TAG_BORDER:-TAG_BORDER] \
           = br_tag



#
# print(a)
# print(a.shape)
#
# print(output_img[TAG_BORDER+TAG_SIZE+TAG_BORDER:-(TAG_BORDER+TAG_SIZE+TAG_BORDER),
#            TAG_BORDER+TAG_SIZE+TAG_BORDER:-(TAG_BORDER+TAG_SIZE+TAG_BORDER)][0:10,0:10])
# Put resized image in output image
output_img[TAG_BORDER+TAG_SIZE+TAG_BORDER:-(TAG_BORDER+TAG_SIZE+TAG_BORDER),
           TAG_BORDER+TAG_SIZE+TAG_BORDER:-(TAG_BORDER+TAG_SIZE+TAG_BORDER)] \
           = resized
cv2.imshow("test Tag", resized)
cv2.waitKey(0)

# output_img[TAG_BORDER:TAG_SIZE]

# # Detection
# arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
# arucoParams = cv2.aruco.DetectorParameters_create()
# (corners, ids, rejected) = cv2.aruco.detectMarkers(input_img, arucoDict,
# 	parameters=arucoParams)
# print(input_img.shape)
# print(ul_tag.shape)
# print(output_img[TAG_BORDER:TAG_SIZE+TAG_BORDER,
#                  TAG_BORDER:TAG_SIZE+TAG_BORDER].shape)
# print(output_img)
cv2.imshow("ArUCo Tag", output_img)
cv2.waitKey(0)

#
# id = 0
#
#
#
# tag = np.zeros((TAG_SIZE, TAG_SIZE, 1), dtype="uint8")
# cv2.aruco.drawMarker(arucoDict, id, TAG_SIZE, tag, 1)
# # write the generated ArUCo tag to disk and then display it to our
# # screen
#
# # cv2.imwrite(args["output"], tag)
# print(tag.shape)
# cv2.imshow("ArUCo Tag", tag)
# cv2.waitKey(0)
#
#
# # verify *at least* one ArUco marker was detected
# if len(corners) > 0:
# 	# flatten the ArUco IDs list
# 	ids = ids.flatten()
# 	# loop over the detected ArUCo corners
# 	for (markerCorner, markerID) in zip(corners, ids):
# 		# extract the marker corners (which are always returned in
# 		# top-left, top-right, bottom-right, and bottom-left order)
# 		corners = markerCorner.reshape((4, 2))
# 		(topLeft, topRight, bottomRight, bottomLeft) = corners
# 		# convert each of the (x, y)-coordinate pairs to integers
# 		topRight = (int(topRight[0]), int(topRight[1]))
# 		bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
# 		bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
# 		topLeft = (int(topLeft[0]), int(topLeft[1]))
#
# 		# draw the bounding box of the ArUCo detection
# 		cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
# 		cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
# 		cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
# 		cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
# 		# compute and draw the center (x, y)-coordinates of the ArUco
# 		# marker
# 		cX = int((topLeft[0] + bottomRight[0]) / 2.0)
# 		cY = int((topLeft[1] + bottomRight[1]) / 2.0)
# 		cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
# 		# draw the ArUco marker ID on the image
# 		cv2.putText(image, str(markerID),
# 			(topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
# 			0.5, (0, 255, 0), 2)
# 		print("[INFO] ArUco marker ID: {}".format(markerID))
#
# # show the output image
# cv2.imshow("Image", image)
# cv2.waitKey(0)

# cv2.imshow('Test Image', image)
# cv2.waitKey(0)


# while True:
#     input_img = cv2.imread('sample_img.jpg')
#     window.updateFrame(input_img)
#     if cv2.waitKey(1) == ord('q'):
#         break

cv2.destroyAllWindows()
