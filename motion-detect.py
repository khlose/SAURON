# import the necessary packages
import imutils
import time
import cv2
import numpy
 

camera = cv2.VideoCapture(0)


# initialize the first frame in the video stream
firstFrame = None

# loop over the frames of the video
while True:
	# grab the current frame and initialize the occupied/unoccupied
	# text
	(grabbed, frame) = camera.read()

 
	# if the frame could not be grabbed, then we have reached the end
	# of the video
	if not grabbed:
		break
 
	cv2.imshow("video",frame)
 
# cleanup the camera and close any open windows
#from tutorial: https://www.pyimagesearch.com/2015/05/25/basic-motion-detection-and-tracking-with-python-and-opencv/
camera.release()
cv2.destroyAllWindows()
