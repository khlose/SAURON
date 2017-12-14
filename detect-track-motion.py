# OpenCV for tracking/display
import cv2
from threading import Thread
import imutils

#change to disable multithread
multithread = True
GlobalFrame = None

class threadedCamera:
    def __init__(self,src = 0):
        self.stream = cv2.VideoCapture(src)
        (self.grabbed,self.frame) = self.stream.read()
        self.back = None
        self.gray = None
        self.cnts = None
        self.stopped = False

    def start(self):
        Thread(target=self.update,args=()).start()
        return self

    def update(self):
        while True:
            if self.stopped:
                return
            (self.grabbed,self.frame)=self.stream.read()
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
        thresh2 = cv2.threshold(frame_delta, 25, 255, cv2.THRESH_BINARY)[1]
        # Dialate threshold to further reduce error
        thresh2 = cv2.erode(thresh2, None, iterations=2)
        thresh2 = cv2.dilate(thresh2, dilated, iterations=17)
        # Check for contours in our threshold
        _, self.cnts, hierarchy2 = cv2.findContours(thresh2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    def readContour(self):
        return self.cnts

    def reset(self):
        self.back = None
    def release(self):
        self.stop()
        self.stream.release()


class threadeMotionTracker():
    def __init__(self,type="KCF"):
        self.tracker = cv2.Tracker_create("KCF")
        self.status = 'idle'

    def initTracker(self,frame,bbox):
        self.tracker.init(frame,bbox)

    def start(self):
        Thread(target=self.update,args=()).start()
        return self

    def updateCoord(self,x,y,w,h):
        self.x = x
        self.y = y
        self.w = w
        self.h = h

    def updateStatus(self,status):
        self.status = status

    def update(self):
        if not self.stop:
            if(self.status == 'tracking'):
                print "update"
                self.tracker.update(GlobalFrame)

    def stop(self):
        self.stop = True

    def release(self):
        self.stop()



if __name__ == '__main__':
    # Are we finding motion or tracking
    status = 'motion'
    # How long have we been tracking
    idle_time = 0

    # Background for motion detection
    back = None
    threadedTracker = threadeMotionTracker(type="KCF")


    # Webcam footage (or video) (blocking I/O)
    threadedVid = threadedCamera(src = 0).start()

    (x, y, w, h) = (0, 0, 0, 0)
    dilated = None

    # LOOP
    while True:
        # Check first frame (blocking I/O)
        print "check1"

        frame = threadedVid.read()


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
            cnts = threadedVid.readContour()


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

                # For each contour
                for i in range(len(cnts)):
                    (x, y, w, h) = cv2.boundingRect(cnts[i])
                    if x < left: left = x
                    if w > right: right = w
                    if y < top: top = y
                    if h > bottom: bottom = h
                    # create bounding box
                bbox = (int(left),int(top),int(right),int(bottom))
                threadedTracker.initTracker(frame,bbox)
                status = 'tracking'
                threadedTracker.updateStatus('tracking')

        # If we have been tracking for more than a few seconds
        if idle_time >= 10:
            # Reset to motion
            status = 'motion'
            threadedTracker.updateStatus('motion')
            # Reset timer
            idle_time = 0

            # Reset background, frame, and tracker
            back = None
            threadedTracker = None
            ok = None
            threadedVid.reset()
            # Recreate tracker
            threadedTracker = threadeMotionTracker(type="KCF")

        # Incriment timer
        idle_time += 1

        # Check if we've quit
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

# QUIT
threadedVid.release()
threadedTracker.release()
cv2.destroyAllWindows()
