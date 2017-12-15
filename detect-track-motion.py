# OpenCV for tracking/display
import cv2
from threading import Thread
import imutils
import numpy as np

# change to disable multithread
multithread = True
GlobalFrame = None
GlobalGray = None
redlower = (17,15,100)
redupper = (50,56,200)


class threadedCamera:
    def __init__(self, src=0):
        self.stream = cv2.VideoCapture(src)
        (self.grabbed, self.frame) = self.stream.read()
        self.back = None
        self.gray = None
        self.cnts = None
        self.stopped = False

    def start(self):
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        while True:
            if self.stopped:
                return
            (self.grabbed, self.frame) = self.stream.read()
            GlobalFrame = self.frame
            self.erodeDilate()

    def read(self):
        return self.frame

    def stop(self):
        self.stopped = True

    def erodeDilate(self):
        if self.back == None:
            self.back = self.gray
        self.gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        # Blur footage to prevent artifacts
        self.gray = cv2.GaussianBlur(gray, (21, 21), 0)

        GlobalGray = self.gray

        hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        mask0 = cv2.inRange(hsv, lower_red, upper_red)

        # upper mask (170-180)
        lower_red = np.array([170, 50, 50])
        upper_red = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)
        mask = mask0 + mask1
        output_img = cv2.bitwise_and(hsv, hsv, mask=mask)


        blurred = cv2.GaussianBlur(gray, (11, 11), 0)
        thresh = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)[1]
        thresh = cv2.erode(thresh, None, iterations=2)
        thresh = cv2.dilate(thresh, None, iterations=4)


        #cv2.imshow("gray",gray)
        cv2.imshow("filtered",output_img)

        thresh2 = cv2.threshold(frame_delta, 25, 255, cv2.THRESH_BINARY)[1]
        # Dialate threshold to further reduce error
        thresh2 = cv2.erode(thresh2, None, iterations=2)
        thresh2 = cv2.dilate(thresh2, dilated, iterations=17)
        # Check for contours in our threshold
        _, self.cnts, hierarchy2 = cv2.findContours(thresh2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #cv2.imshow("thresh",thresh2)

    def readContour(self):
        return self.cnts

    def reset(self):
        self.back = None
    def release(self):
        self.stream.release()

def moveToAlign(left,top,right,bottom):
    #width 640 height 480
    horizontal_diff = left - 320
    degHorizontal = horizontal_diff*5.15/42
    vertical_diff = top - 240
    degVertical = vertical_diff*5.15/42
    #42 pixels = 5.15 deg
    negFlagH = 1
    negFlagV = 1
    if horizontal_diff < 0:
        negFlagH = -1
    if vertical_diff < 0:
        negFlagV = -1

    moveHorizontal(negFlagH*degHorizontal)
    moveVertical(negFlagV*degVertical)

    return

def calcDistanceFromLaser(frame):
    return

def moveHorizontal():
    return

def moveVertical():
    return

if __name__ == '__main__':
    # Are we finding motion or tracking
    status = 'motion'
    # How long have we been tracking
    idle_time = 0

    # Background for motion detection
    back = None

    # Webcam footage (or video) (blocking I/O)
    if multithread:
        threadedVid = threadedCamera(src=0).start()
    else:
        video = cv2.VideoCapture(0)
    (x, y, w, h) = (0, 0, 0, 0)
    dilated = None

    # LOOP
    while True:
        # Check first frame (blocking I/O)
        #print "check1"

        if multithread:
            frame = threadedVid.read()
        else:
            ok, frame = video.read()

        # Grayscale footage
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Blur footage to prevent artifacts
        gray = cv2.GaussianBlur(gray, (21, 21), 0)

        # Check for background
        if back is None:
            # Set background to current frame
            back = gray

        if status == 'motion':

            # Difference between current frame and background
            frame_delta = cv2.absdiff(back, gray)

            # Create a threshold to exclude minute movements
            # threshold(src treshold,max value,)
            if multithread:
                cnts = threadedVid.readContour()
            else:
                thresh2 = cv2.threshold(frame_delta, 25, 255, cv2.THRESH_BINARY)[1]
                # Dialate threshold to further reduce error
                thresh2 = cv2.erode(thresh2, None, iterations=2)
                thresh2 = cv2.dilate(thresh2, dilated, iterations=17)
                # Check for contours in our threshold
                _, cnts, hierarchy2 = cv2.findContours(thresh2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Check each contour
            if cnts != None and len(cnts) != 0:
                print "Not None"
                # If the contour is big enough

                # Set largest contour to first contour
                largest = 0
                left = 2000
                right = 0
                top = 2000
                bottom = -1000

                # coord -x ===== +x
                #      -y
                #       =
                #       =
                #      +y

                # For each contour (unit in pixel??)
                for i in range(len(cnts)):
                    (x, y, w, h) = cv2.boundingRect(cnts[i])
                    if x < left: left = x
                    if w > right: right = w
                    if y < top: top = y
                    if h > bottom: bottom = h
                    # create bounding box
                bbox = (int(left), int(top), int(right), int(bottom))
                #moveToAlign(left,top,right,bottom)

                status = 'tracking'


        # If we have been tracking for more than a few seconds
        if idle_time >= 10:
            # Reset to motion
            status = 'motion'
            # Reset timer
            idle_time = 0

            # Reset background, frame, and tracker
            back = None
            ok = None

            threadedVid.reset()

        # Incriment timer
        idle_time += 1

        # Check if we've quit
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

# QUIT
threadedVid.release()
threadedVid.stop()
cv2.destroyAllWindows()