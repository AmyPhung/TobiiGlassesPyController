import cv2
import numpy as np

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
    		corners_reshaped = markerCorner.reshape((4, 2))
    		(topLeft, topRight, bottomRight, bottomLeft) = corners_reshaped
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
    return image, corners, ids

def computeGazePixel(src_img, live_img, tags, params, gaze_info):
    # computeGazePixel(self.display_frame, self.tags, self.params)
    """Computes pixel (u,v) in image """
    ctrl_corners_tobii = getControlCornersInTobii(tags)
    ctrl_corners_window = getControlCornersInWindow(src_img, params)

    print(ctrl_corners_tobii)
    print(ctrl_corners_window)
    # Compute mapping between Tobii camera feed and Aruco window


def getControlCornersInTobii(tags):
    """ Returns 'control corners' as the pixel position they appear in
    the Tobii camera feed

    - upper-right corner of upper-left tag
    - upper-left corner of upper-right tag
    - bottom-right corner of bottom-left tag
    - bottom-left corner of bottom-right tag
     _____   ________________   _____
    | tag X |                | X tag |
    |__0__| |                | |__1__|
            |     display    |
     _____  |      image     |  _____
    | tag | |                | | tag |
    |__3__X |________________| X__2__|

    These corners are used since they are closest to the display
    image corners

    For each tag in tags, the corners are listed in the following order:
    [upper-left, upper-right, bottom-right, bottom-left]. Each coordinate
    is written as [x, y] where the origin is the top-left corner of the
    image

    Returns the control corners in the same order
    """
    upper_left   = tags[0][0][1]
    upper_right  = tags[1][0][0]
    bottom_right = tags[2][0][3]
    bottom_left  = tags[3][0][2]

    return np.array([upper_left, upper_right, bottom_right, bottom_left])

def getControlCornersInWindow(src_img, params):
    """ Returns 'control corners' as the pixel location they appear in
    the ArUco window interface """

    img_height, img_width, _ = np.multiply(src_img.shape, params['image_rescale'])

    upper_left   = [params['tag_size'] + params['tag_border'],
                    params['tag_border']]

    upper_right  = [params['tag_size'] + params['tag_border']*3 + img_width,
                    params['tag_border']]

    bottom_right = [params['tag_size'] + params['tag_border']*3 + img_width,
                    params['tag_border'] + img_height]

    bottom_left  = [params['tag_size'] + params['tag_border'],
                    params['tag_border'] + img_height]

    return np.array([upper_left, upper_right, bottom_right, bottom_left])
