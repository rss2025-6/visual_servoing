import cv2
import imutils
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
	Helper function to print out images, for debugging.
	Press any key to continue.
	"""
	winname = "Image"
	cv2.namedWindow(winname)        # Create a named window
	cv2.moveWindow(winname, 40,30)  # Move it to (40,30)
	cv2.imshow(winname, img)
	cv2.waitKey()
	cv2.destroyAllWindows()

def cd_sift_ransac(img, template):
    """
    Implement the cone detection using SIFT + RANSAC algorithm.
    Input:
        img: np.3darray; the input image with a cone to be detected
    Return:
        bbox: ((x1, y1), (x2, y2)); the bounding box in image coordinates (Y increasing downwards),
            where (x1, y1) is the top-left pixel of the box
            and (x2, y2) is the bottom-right pixel of the box.
    """
	# Minimum number of matching features
	MIN_MATCH = 10 # Adjust this value as needed
	# Create SIFT
	sift = cv2.xfeatures2d.SIFT_create()

	# Compute SIFT on template and test image
	kp1, des1 = sift.detectAndCompute(template,None)
	kp2, des2 = sift.detectAndCompute(img,None)

	# Find matches
	bf = cv2.BFMatcher()
	matches = bf.knnMatch(des1,des2,k=2)

	# Find and store good matches
	good = []
	for m,n in matches:
		if m.distance < 0.75*n.distance:
			good.append(m)

	# If enough good matches, find bounding box
	if len(good) > MIN_MATCH:
		src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
		dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

		# Create mask
		M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
		matchesMask = mask.ravel().tolist()

		h, w = template.shape
		pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)

		########## YOUR CODE STARTS HERE ##########

		# https://docs.opencv.org/3.4/d1/de0/tutorial_py_feature_homography.html
		dst = cv2.perspectiveTransform(pts, M)
		img2 = cv2.polylines(img,[np.int32(dst)],True,255,3,cv2.LINE_AA)
		image_print(img2)

		# Extract bounding box
		x_min = w
		y_min = h
		x_max = y_max = 0
		for col in dst:
			for row in col:
				x,y = row
				if x > x_max:
					x_max = x
				if x < x_min:
					x_min = x
				if y > y_max:
					y_max = y
				if y < y_min:
					y_min = y
				# print(f"x: {x}, y: {y}")
		# print(f"${dst}")
		
		########### YOUR CODE ENDS HERE ###########

		# Return bounding box
		return ((x_min, y_min), (x_max, y_max))
	else:

		print(f"[SIFT] not enough matches; matches: ", len(good))

		# Return bounding box of area 0 if no match found
		return ((0,0), (0,0))

def cd_template_matching(img, template):
    """
    Implement the cone detection using template matching algorithm.
    Input:
        img: np.3darray; the input image with a cone to be detected
    Return:
        bbox: ((x1, y1), (x2, y2)); the bounding box in px (Y increases downward),
            where (x1, y1) is the top-left corner and (x2, y2) is the bottom-right corner.
    """
	template_canny = cv2.Canny(template, 50, 200)

	# Perform Canny Edge detection on test image
	grey_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	img_canny = cv2.Canny(grey_img, 50, 200)

	# Get dimensions of template
	(img_height, img_width) = img_canny.shape[:2]

	# Keep track of best-fit match
	best_match = None
	best_match_image = None
	bounding_box = ((0,0),(0,0))

	# Loop over different scales of image
	for scale in np.linspace(1.5, .5, 50):
		# Resize the image
		resized_template = imutils.resize(template_canny, width = int(template_canny.shape[1] * scale))
		(h,w) = resized_template.shape[:2]
		# Check to see if test image is now smaller than template image
		if resized_template.shape[0] > img_height or resized_template.shape[1] > img_width:
			continue

		########## YOUR CODE STARTS HERE ##########
		# Use OpenCV template matching functions to find the best match
		# across template scales.
		# https://docs.opencv.org/4.x/d4/dc6/tutorial_py_template_matching.html
		for method in [cv2.TM_CCOEFF_NORMED, cv2.TM_CCORR_NORMED, cv2.TM_SQDIFF_NORMED]:
			imgcop = img_canny.copy()
			result = cv2.matchTemplate(imgcop, resized_template, method)
			min_value, max_value, min_loc, max_loc = cv2.minMaxLoc(result)

			if method == cv2.TM_SQDIFF_NORMED:
				top_left = min_loc
				max_value = 1.-min_value
			else:
				top_left = max_loc
			bottom_right = (top_left[0] + w, top_left[1]+h)

			# draws rectangle into imgcomp
			cv2.rectangle(imgcop, top_left, bottom_right, 255, 2)
			# print(f"scale: {scale}, method: {method}, max_value: {max_value}")
			
			if best_match == None or max_value > best_match:
				best_match = max_value
				best_match_image = imgcop
				bounding_box = ((top_left[0], top_left[1]), (bottom_right[0], bottom_right[1]))
			
		# Remember to resize the bounding box using the highest scoring scale
		# x1,y1 pixel will be accurate, but x2,y2 needs to be correctly scaled
	image_print(best_match_image)
	########### YOUR CODE ENDS HERE ###########
	return bounding_box
