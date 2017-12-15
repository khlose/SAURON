# OpenCV for tracking/display
from __future__ import division
import cv2
from threading import Thread
import imutils
import RPi.GPIO as GPIO
import Adafruit_PCA9685
import math

# change to disable multithread
multithread = True
GlobalFrame = None
GlobalGray = None

pwm = Adafruit_PCA9685.PCA9685()
servo_min = 150  # Min pulse length out of 4096
servo_max = 600  # Max pulse length out of 4096
# Helper function to make setting a servo pulse width simpler.
def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 60       # 60 Hz
    print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096     # 12 bits of resolution
    print('{0}us per bit'.format(pulse_length))
    pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, pulse)
# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)
#0 servos
serv1=375
serv2=375
pwm.set_pwm(0,0,serv1)
pwm.set_pwm(1,0,serv2)

#GPIO Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(20,GPIO.OUT)
GPIO.setup(26,GPIO.OUT)


class threadedCamera:
    def __init__(self, src=0):
        self.stream = cv2.VideoCapture(src)
        (self.grabbed, self.frame) = self.stream.read()
        self.back = None
        self.gray = None
        self.cnts = None
        self.stopped = False
        self.left = 0

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

        self.gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        # Blur footage to prevent artifacts
        self.gray = cv2.GaussianBlur(self.gray, (21, 21), 0)

        if self.back == None:
            self.back = self.gray
        frame_delta = cv2.absdiff(self.back,self.gray)
        thresh2 = cv2.threshold(frame_delta, 25, 255, cv2.THRESH_BINARY)[1]
        # Dialate threshold to further reduce error
        thresh2 = cv2.erode(thresh2, None, iterations=2)
        thresh2 = cv2.dilate(thresh2, dilated, iterations=17)
        # Check for contours in our threshold
        _, self.cnts, hierarchy2 = cv2.findContours(thresh2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        _, _, _, max_loc = cv2.minMaxLoc(thresh2)
        #cv2.circle(thresh2, max_loc, 5, (255, 0, 0), 2)
        #print "max loc : " + str(max_loc)
        #cv2.imshow("thres", thresh2)
        self.left = max_loc[0]

    def readContour(self):
        return self.cnts

    def read_left(self):

        return self.left

    def reset(self):
        self.back = None
    def release(self):
        self.stream.release()

def moveToAlign(left):
    #width 640 height 480

    horizontal_diff = left - 320
    degHorizontal = horizontal_diff*5.15/42


    moveHorizontal(degHorizontal)

    return

def calcDistanceFromLaser(frame):
    #getRedLasereDOt
    _,_,r = cv2.split(frame)
    (minVal,maxVal,minLoc,maxLoc) = cv2.minMaxLoc(r)
    xCoord = maxLoc[0]
    #change this
    #moveVertical((xCoord-320)*5.15/42)
    return

def moveHorizontal(angle):
    #print "angle before add: " + str(angle)
    angle = angle + 85
    pwm_val = 2.835*angle + 95
    #print "angle: " +str(angle) + "pwm_val_raw" + str(pwm_val) + "pwm_val Hori= " + str(int(math.ceil(pwm_val)))
    int_pwm = int(math.ceil(pwm_val))
    if int_pwm < 210:
        int_pwm = 210
    if int_pwm > 420:
        int_pwm = 420
    print "detecting Target Writing : " + str(int_pwm)
    pwm.set_pwm(0, 0, int_pwm)
    return

def moveOrigin():
    pwm.set_pwm(0, 0, 360)
    #pwm.set_pwm(1, 0, 375)
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
    moveOrigin()
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

                left = 255

                moveToAlign(threadedVid.read_left())
                #calcDistanceFromLaser(frame)
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
            moveOrigin()
        # Incriment timer
        idle_time += 1

        # Check if we've quit
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

# QUIT
threadedVid.release()
threadedVid.stop()
cv2.destroyAllWindows()