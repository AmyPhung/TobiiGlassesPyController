import cv2
import numpy as np

TAG_SIZE = 100 # px
TAG_BORDER = 10 # px
IMAGE_RESCALE = 1 # %

# Load image
input_img = cv2.imread('sample_img3.jpg')#, cv2.IMREAD_GRAYSCALE)

# Resize image
img_height = int(input_img.shape[0]*IMAGE_RESCALE)
img_width  = int(input_img.shape[1]*IMAGE_RESCALE)
dim = (img_width, img_height)
resized = cv2.resize(input_img, dim, interpolation = cv2.INTER_AREA)

# Allocate space for output image
output_img = np.ones((img_height + 2*TAG_BORDER,
                      img_width + 2*TAG_SIZE + 4*TAG_BORDER, 3),
                      dtype="uint8")*255 # 255 for white background

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
output_img[-(TAG_SIZE+TAG_BORDER):-TAG_BORDER,
           TAG_BORDER:TAG_SIZE+TAG_BORDER] \
           = bl_tag
output_img[-(TAG_SIZE+TAG_BORDER):-TAG_BORDER,
           TAG_BORDER+TAG_SIZE+TAG_BORDER+img_width+TAG_BORDER:-TAG_BORDER] \
           = br_tag

# Put resized image in output image
output_img[TAG_BORDER:-TAG_BORDER,
           TAG_BORDER+TAG_SIZE+TAG_BORDER:-(TAG_BORDER+TAG_SIZE+TAG_BORDER)] \
           = resized

cv2.imshow("Image with ArUco Tags", output_img)
cv2.waitKey(0)
cv2.destroyAllWindows()
