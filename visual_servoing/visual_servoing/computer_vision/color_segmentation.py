import cv2
import numpy as np

#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################

def image_print(img):
	"""
	Helper function to print out images, for debugging. Pass them in as a list.
	Press any key to continue.
	"""
	cv2.imshow("image", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

def cd_color_segmentation(img, template):
	"""
	Implement the cone detection using color segmentation algorithm
	Input:
		img: np.3darray; the input image with a cone to be detected. BGR.
		template_file_path; Not required, but can optionally be used to automate setting hue filter values.
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
				(x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
	"""
	########## PARAMETERS FOR CONE COLOR SEGMENTING ##########
	# https://docs.opencv.org/4.x/de/d25/imgproc_color_conversions.html#color_convert_rgb_hsv
	# Using values for Safety Orange (H: 28deg, S: 100%, V: 100%)
	CONE_HUE = 14 # H <-H/2 where Safety Orange H=28deg
	CONE_HUE_TOLERANCE = 5 # +- error
	CONE_HUE_MIN = CONE_HUE - CONE_HUE_TOLERANCE
	CONE_HUE_MAX = CONE_HUE + CONE_HUE_TOLERANCE

	CONE_SATURATION = 255 # S <- 255S where S=100%
	CONE_SATURATION_TOLERANCE = 60 # - error
	CONE_SATURATION_MIN = CONE_SATURATION - CONE_SATURATION_TOLERANCE
	CONE_SATURATION_MAX = CONE_SATURATION

	CONE_VALUE = 255 # S <- 255S where S=100%
	CONE_VALUE_TOLERANCE = 80 # - error
	CONE_VALUE_MIN = CONE_VALUE - CONE_VALUE_TOLERANCE
	CONE_VALUE_MAX = CONE_VALUE

	########## RETURN VALUES ##########
	ymin=len(img) # Set to highest value
	xmin=len(img[0])
	xmax=0 # Set to lowest value
	ymax=0

	# convert BGR color space to HSV space
	hsvimg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	thresholdimg = cv2.inRange(hsvimg, (CONE_HUE_MIN, CONE_SATURATION_MIN, CONE_VALUE_MIN), (CONE_HUE_MAX, CONE_SATURATION_MAX, CONE_VALUE_MAX))

	# iterate the image
	ci=0
	for col in hsvimg:
		ri=0
		for row in col:
			h, s, v = row
			if -CONE_HUE_TOLERANCE <= CONE_HUE-h <= CONE_HUE_TOLERANCE \
				and CONE_SATURATION-s <= CONE_SATURATION_TOLERANCE \
				and CONE_VALUE-v <= CONE_VALUE_TOLERANCE:
				
				if ri < xmin:
					xmin=ri
				elif ri > xmax:
					xmax = ri
				if ci < ymin:
					ymin = ci
				elif ci > ymax:
					ymax = ci
			ri+=1
		ci+=1
	# image_print(img)
	image_print(thresholdimg) # Shows the threshold based on parameters

	bounding_box = ((xmin,ymin),(xmax,ymax))

	# Return bounding box
	return bounding_box
