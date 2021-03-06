# USAGE
# python distance_between.py --image images/example_01.png --width 0.955
# python distance_between.py --image images/example_02.png --width 0.955
# python distance_between.py --image images/example_03.png --width 3.5

# import the necessary packages
from scipy.spatial import distance as dist
from imutils import perspective
from imutils import contours
import numpy as np
import argparse
import imutils
import cv2

def euler_to_quaternion(roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]

def midpoint(ptA, ptB):
	return ((ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5)

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required=True,
	help="path to the input image")
ap.add_argument("-w", "--width", type=float, required=True,
	help="width of the left-most object in the image (in inches)")
args = vars(ap.parse_args())


# load the image, convert it to grayscale, and blur it slightly
image = cv2.imread(args["image"])
scale_percent = 20 # percent of original size
width = int(image.shape[1] * scale_percent / 100)
height = int(image.shape[0] * scale_percent / 100)
dim = (width, height) 
image = cv2.resize(image, dim)

gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
gray = cv2.GaussianBlur(gray, (7, 7), 0)

# perform edge detection, then perform a dilation + erosion to
# close gaps in between object edges
edged = cv2.Canny(gray, 50, 100)
edged = cv2.dilate(edged, None, iterations=1)
edged = cv2.erode(edged, None, iterations=1)

# find contours in the edge map
cnts = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL,
	cv2.CHAIN_APPROX_SIMPLE)
cnts = imutils.grab_contours(cnts)

# sort the contours from left-to-right and, then initialize the
# distance colors and reference object
(cnts, _) = contours.sort_contours(cnts)
colors = ((0, 0, 255), (240, 0, 159), (0, 165, 255), (255, 255, 0),
	(255, 0, 255))
refObj = None

distance_coordincates = []
transform_coordinates = []

# loop over the contours individually
for c in cnts:
	# if the contour is not sufficiently large, ignore it
	if cv2.contourArea(c) < 100:
		continue

	# compute the rotated bounding box of the contour
	box = cv2.minAreaRect(c)
	box = cv2.cv.BoxPoints(box) if imutils.is_cv2() else cv2.boxPoints(box)
	box = np.array(box, dtype="int")

	# order the points in the contour such that they appear
	# in top-left, top-right, bottom-right, and bottom-left
	# order, then draw the outline of the rotated bounding
	# box
	box = perspective.order_points(box)

	# compute the center of the bounding box
	cX = np.average(box[:, 0])
	cY = np.average(box[:, 1])
	# if this is the first contour we are examining (i.e.,
	# the left-most contour), we presume this is the
	# reference object
	if refObj is None:
		# unpack the ordered bounding box, then compute the
		# midpoint between the top-left and top-right points,
		# followed by the midpoint between the top-right and
		# bottom-right
		(tl, tr, br, bl) = box
		(tlblX, tlblY) = midpoint(tl, bl)
		(trbrX, trbrY) = midpoint(tr, br)

		# compute the Euclidean distance between the midpoints,
		# then construct the reference object
		D = dist.euclidean((tlblX, tlblY), (trbrX, trbrY))
		refObj = (box, (cX, cY), D / args["width"])
		continue

	# draw the contours on the image
	orig = image.copy()
	cv2.drawContours(orig, [box.astype("int")], -1, (0, 255, 0), 2)
	cv2.drawContours(orig, [refObj[0].astype("int")], -1, (0, 255, 0), 2)

	# stack the reference coordinates and the object coordinates
	# to include the object center
	refCoords = np.vstack([refObj[0], refObj[1]])
	objCoords = np.vstack([box, (cX, cY)])

        # print("coords", objCoords)
        boxCoords = objCoords[:4]
        point1, point2 = boxCoords[0], boxCoords[1]
        dx, dy = point2[0] - point1[0], point2[1] - point1[1]
        rotation = np.arctan(dy/dx)
        # print("rotation: " + str(rotation))
        quaternion = euler_to_quaternion(0,0,rotation)
        print("rotation as quaternion: " + str(rotation))
        # print("all box coords: " + str(objCoords))
        print("centroid...? in cm: " + str(objCoords[4] / refObj[2]))
	counter = 0
	# loop over the original points
	for ((xA, yA), (xB, yB), color) in zip(refCoords, objCoords, colors):
		# draw circles corresponding to the current points and
		# connect them with a line
		if counter < 4:
			counter += 1
			continue
		else:
			counter = 0
			cv2.circle(orig, (int(xA), int(yA)), 5, color, -1)
			cv2.circle(orig, (int(xB), int(yB)), 5, color, -1)
			cv2.line(orig, (int(xA), int(yA)), (int(xB), int(yB)),
				color, 2)

			# compute the Euclidean distance between the coordinates,
			# and then convert the distance in pixels to distance in
			# units
			D = dist.euclidean((xA, yA), (xB, yB)) / refObj[2]
			(mX, mY) = midpoint((xA, yA), (xB, yB))
			cv2.putText(orig, "{:.1f}cm".format(D), (int(mX), int(mY - 10)),
			cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
			x = abs(xB / refObj[2] - xA / refObj[2])
			y = abs(yB / refObj[2] - yA / refObj[2])
			print("x distance: ", x)
			print("y distance: ", y)
			transform_coordinates.append([x, y] + quaternion)

			# print(xA / refObj[2], yA / refObj[2]) # cordinates of first dot connecting the two objects
			# print(xB / refObj[2], yB / refObj[2]) # cordinates of second dot connecting the two objects

			# show the output image
			cv2.imshow("Image", orig)
			cv2.waitKey(0)
print(transform_coordinates)
a = np.array(transform_coordinates)
np.save("transform_coordinates",a)
			
