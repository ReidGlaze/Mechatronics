import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
centerdistResolution = 80
class arduComm:
    def __init__(self,balloonModePin,forwPin,backPin,rightPin,leftPin):
        self.bmPin = balloonModePin
        self.fPin = forwPin
        self.bPin = backPin
        self.rPin = rightPin
        self.lPin = leftPin
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(self.bmPin, GPIO.OUT)
        GPIO.setup(self.fPin, GPIO.OUT)
        GPIO.setup(self.bPin, GPIO.OUT)
        GPIO.setup(self.lPin, GPIO.OUT)
        GPIO.setup(self.rPin, GPIO.OUT)
    def setOutput(self,to):
        GPIO.output(self.bmPin, to[0])
        GPIO.output(self.fPin, to[1])
        GPIO.output(self.bPin, to[2])
        GPIO.output(self.lPin, to[3])
        GPIO.output(self.rPin, to[4])
    def goForward(self):
        print("FWD")
        self.setOutput([GPIO.HIGH,GPIO.HIGH,GPIO.LOW,GPIO.LOW,GPIO.LOW])
    def goBack(self):
        print("BACK")
        self.setOutput([GPIO.HIGH,GPIO.LOW,GPIO.HIGH,GPIO.LOW,GPIO.LOW])
    def goLeft(self):
        print("LEFT")
        self.setOutput([GPIO.HIGH,GPIO.LOW,GPIO.LOW,GPIO.HIGH,GPIO.LOW])
    def goRight(self):
        print("RIGHT")
        self.setOutput([GPIO.HIGH,GPIO.LOW,GPIO.LOW,GPIO.LOW,GPIO.HIGH])
    def stop(self):
        print("STOP")
        self.setOutput([GPIO.LOW,GPIO.LOW,GPIO.LOW,GPIO.LOW,GPIO.LOW])
class bFinderOutput:
    def __init__(self,area,centroid,didFind):
        self.area = area
        self.centroid = centroid
        self.didFind = didFind
def findBalloon(imageHSV, filter, colorTag):
11
     areaCutoff = 15000
    mask = filter(imageHSV)
    contours,heirarchy =
cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    maxArea = 0
    maxIndex = -1
    for i in range(len(contours)):
        currentArea = cv2.contourArea(contours[i])
        if currentArea > maxArea:
            maxArea = currentArea
            maxIndex = i
    if (not maxIndex == -1) and (maxArea > areaCutoff):
        moment = cv2.moments(contours[maxIndex])
        cx = int(moment['m10']/moment['m00'])
        cy = int(moment['m01']/moment['m00'])
        print(colorTag)
        print("max Area: ", maxArea)
        print("cx ", cx," cy ",cy)
        return bFinderOutput(maxArea,(cx,cy),True)
    else:
        return bFinderOutput(0,(0,0),False)
def redFilter(image):
    maskMin1 = np.array([0,50,20])
    maskMax1 = np.array([5,255,255])
    maskMin2 = np.array([175,50,20])
    maskMax2 = np.array([180,255,255])
    mask1 = cv2.inRange(frameHSV, maskMin1,maskMax1)
    mask2 = cv2.inRange(frameHSV, maskMin2,maskMax2)
    return cv2.bitwise_or(mask1, mask2)
def blueFilter(image):
    maskMin = np.array([100,50,50])
    maskMax = np.array([140,255,255])
    return cv2.inRange(frameHSV,maskMin,maskMax)
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 10)
comm = arduComm(29,31,33,35,37)
if (cap.isOpened() == False):
  print("Unable to read camera feed")
while(True):
    startTime = time.time_ns()
    ret, frame = cap.read()
    frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    redBalloon = findBalloon(frameHSV,redFilter,"RED")
    frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    blueBalloon = findBalloon(frameHSV,blueFilter,"BLUE")
    if (redBalloon.didFind):
        offset = redBalloon.centroid[0] - 320
12

if (offset < centerdistResolution and (not offset >
-centerdistResolution)):
          comm.goRight()
        if (offset > -centerdistResolution and (not offset <
centerdistResolution)):
          comm.goLeft()
        if (offset<centerdistResolution and offset >
-centerdistResolution):
          comm.goForward()
    elif (blueBalloon.didFind):
        if blueBalloon.area > 70000:
            comm.goLeft()
    else:
      #print("STOP")
      comm.stop()
    #print("Frame Rate: ",1/(time.time_ns() - startTime)*1e9)
# When everything done, release the video capture and video write objects
cap.release()
cv2.destroyAllWindows()