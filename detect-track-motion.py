# OpenCV for tracking/display
import cv2
from threading import Thread
import imutils

#change to disable multithread
multithread = True

class threadedCamera:
    def __init__(self,src = 0):
        self.stream = cv2.VideoCapture(src)
        (self.grabbed,self.frame) = self.stream.read()

        self.stopped = False

    def start(self):
        Thread(target=self.update,args=()).start()
        return self

    def update(self):
        while True:
            if self.stopped:
                return
            (self.grabbed,self.frame)=self.stream.read()

    def read(self):
        return self.frame

    def stop(self):
        self.stopped = True

if __name__ == '__main__':
    # Are we finding motion or tracking
    status = 'motion'
    # How long have we been tracking
    idle_time = 0

    # Background for motion detection
    back = None
    # An MIL tracker for when we find motion
    # tracker = cv2.TrackerKCF_create ()
    tracker = cv2.Tracker_create("KCF")

    # Webcam footage (or video) (blocking I/O)
    if multithread:
        threadedVid = threadedCamera(src = 0).start()
    else:
        video = cv2.VideoCapture(0)
    (x, y, w, h) = (0, 0, 0, 0)
    dilated = None

    # LOOP
    while True:
        # Check first frame (blocking I/O)

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
            thresh2 = cv2.threshold(frame_delta, 25, 255, cv2.THRESH_BINARY)[1]

            # Dialate threshold to further reduce error

            thresh2 = cv2.erode(thresh2, None, iterations=2)
            thresh2 = cv2.dilate(thresh2, dilated, iterations=17)

            # Check for contours in our threshold
            _, cnts, hierarchy2 = cv2.findContours(thresh2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Check each contour
            if len(cnts) != 0:
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
                ok = tracker.init(frame,bbox)
                status = 'tracking'


        # If we are tracking
        #if status == 'tracking':
        if status == 'tracking':
            # Update our tracker
            ok, bbox = tracker.update(frame)

        # If we have been tracking for more than a few seconds
        if idle_time >= 10:
            # Reset to motion
            status = 'motion'
            # Reset timer
            idle_time = 0

            # Reset background, frame, and tracker
            back = None
            tracker = None
            ok = None

            # Recreate tracker
            tracker = cv2.Tracker_create("KCF")

        # Incriment timer
        idle_time += 1

        # Check if we've quit
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

# QUIT
video.release()
cv2.destroyAllWindows()
