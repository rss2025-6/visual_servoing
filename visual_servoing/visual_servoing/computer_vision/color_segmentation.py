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



def max_flood_fill_area(img):
	"""
	Input:
		img: np.3darray; the input image with a cone to be detected.
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the largest flood (by area) in the image, unit in px
				(x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
	"""
	def rect_to_bounding(rect):
		x_low=rect[0]
		y_low=rect[1]
		x_high=rect[0]+rect[2]
		y_high=rect[1]+rect[3]
		return ((x_low, y_low), (x_high, y_high))

	height, width = img.shape
	max_area=0
	result=((0,0),(0,0))
	for x in range(height):
		for y in range(width):
			if img[x,y]==255:
				(_, img, _, rect)=cv2.floodFill(img, None, (y,x), 0)
				bbox_area=rect[2]*rect[3]
				if (bbox_area>max_area):
					image_print(img)
					# print(f"{rect} with area {bbox_area} and value {value}")
					max_area =bbox_area
					result=rect_to_bounding(rect)
	return result

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
	CONE_HUE_TOLERANCE = 4 # +- error
	CONE_HUE_MIN = CONE_HUE - CONE_HUE_TOLERANCE
	CONE_HUE_MAX = CONE_HUE + CONE_HUE_TOLERANCE

	CONE_SATURATION = 255 # S <- 255S where S=100%
	CONE_SATURATION_TOLERANCE = 40 # - error
	CONE_SATURATION_MIN = CONE_SATURATION - CONE_SATURATION_TOLERANCE
	CONE_SATURATION_MAX = CONE_SATURATION

	CONE_VALUE = 255 # S <- 255S where S=100%
	CONE_VALUE_TOLERANCE = 115 # - error
	CONE_VALUE_MIN = CONE_VALUE - CONE_VALUE_TOLERANCE
	
	CONE_VALUE_MAX = CONE_VALUE

	def color_segment(hsvimg):
		"""
		Input:
			img: np.3darray; the input image with a cone to be detected. BGR.
		Return:
			bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
					(x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
		"""
		# iterate the image
		height, width = hsvimg.shape[:2]
		for col in range(width):
			for row in range(height):
				h, s, v = hsvimg[row,col]
				if -CONE_HUE_TOLERANCE <= CONE_HUE-h <= CONE_HUE_TOLERANCE \
					and CONE_SATURATION-s <= CONE_SATURATION_TOLERANCE \
					and CONE_VALUE-v <= CONE_VALUE_TOLERANCE:

					if row < xmin:
						xmin=row
					elif row > xmax:
						xmax = row
					if col < ymin:
						ymin = col
					elif col > ymax:
						ymax = col
		return ((xmin, ymin),(xmax, ymax))

	########## RETURN VALUES ##########
	ymin=len(img) # Set to highest value
	xmin=len(img[0])
	xmax=0 # Set to lowest value
	ymax=0

	# convert BGR color space to HSV space
	hsvimg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	thresholdimg = cv2.inRange(hsvimg, 
							(CONE_HUE_MIN, CONE_SATURATION_MIN, CONE_VALUE_MIN), 
							(CONE_HUE_MAX, CONE_SATURATION_MAX, CONE_VALUE_MAX))
	



	# image_print(img)
	# image_print(thresholdimg) # Shows the threshold based on parameters

	# bounding_box = color_segment(hsvimage)
	bounding_box = max_flood_fill_area(thresholdimg)

	# Return bounding box
	return bounding_box
