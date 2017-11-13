# python 3 code
import socket
from time import *
from pynput import keyboard

"""pynput: On Mac OSX, one of the following must be true:
* The process must run as root. OR
* Your application must be white listed under Enable access for assistive devices. Note that this might require that you package your application, since otherwise the entire Python installation must be white listed."""
import sys
import threading
import enum
import urllib.request
import cv2
import numpy
import copy
import imutils
import math

socketLock = threading.Lock()
imageLock = threading.Lock()

IP_ADDRESS = '192.168.1.101'  # SET THIS TO THE RASPBERRY PI's IP ADDRESS


# You should fill this in with your states
class States(enum.Enum):
    LISTEN = enum.auto()
    WAIT = enum.auto()
    FIND_CURRENT_HEADING = enum.auto()
    TURN = enum.auto()
    MOVE_TOWARD_GOAL = enum.auto()


class StateMachine(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)  # MUST call this to make sure we setup the thread correctly
        # CONFIGURATION PARAMETERS
        global IP_ADDRESS
        self.IP_ADDRESS = IP_ADDRESS
        self.CONTROLLER_PORT = 5001
        self.TIMEOUT = 10  # If its unable to connect after 10 seconds, give up.  Want this to be a while so robot can init.
        self.STATE = States.FIND_CURRENT_HEADING
        self.RUNNING = True
        self.DIST = False
        self.previousDirection = 'none'
        self.video = ImageProc()
        # Start video
        self.video.start()

        # connect to the motorcontroller
        try:
            with socketLock:
                self.sock = socket.create_connection((self.IP_ADDRESS, self.CONTROLLER_PORT), self.TIMEOUT)
                self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print("Connected to RP")
        except Exception as e:
            print("ERROR with socket connection", e)
            sys.exit(0)

        # connect to the robot
        """ The i command will initialize the robot.  It enters the create into FULL mode which means it can drive off tables and over steps: be careful!"""
        with socketLock:
            self.sock.sendall("i /dev/ttyUSB0".encode())
            print("Sent command")
            result = self.sock.recv(128)
            print(result)
            if result.decode() != "i /dev/ttyUSB0":
                self.RUNNING = False

        self.sensors = Sensing(self.sock)
        # Start getting data
        self.sensors.start()

        # Collect events until released
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

    def run(self):

        # BEGINNING OF THE CONTROL LOOP
        while (self.RUNNING):
            sleep(0.1)
            if self.STATE == States.LISTEN:
                # pass
                # TODO: Work here
            elif self.STATE == States.FIND_CURRENT_HEADING:
                with socketLock:
                    if self.video.currentHeadingCoords['x'] is not None and self.video.currentHeadingCoords['y'] is not None and self.video.desiredHeading is not None:
                        currentHeadingColors = math.degrees(math.atan2((self.video.cY-self.video.cYpink),(self.video.cX-self.video.cXpink)))
                        print("desired heading: " + str(self.video.desiredHeading))
                        print ("colored current heading: " + str(currentHeadingColors))
                        if currentHeadingColors < (self.video.desiredHeading + self.video.headingAngleThreshold) and currentHeadingColors > (self.video.desiredHeading - self.video.headingAngleThreshold):
                            self.STATE = States.MOVE_TOWARD_GOAL
                        else:
                            self.STATE = States.TURN
            elif self.STATE == States.TURN:
                with socketLock:
                    # turn right
                    self.sock.sendall("a spin_right(50)".encode())
                    print("Sent command")
                    result = self.sock.recv(128)
                    print(result)
                    self.STATE = States.FIND_CURRENT_HEADING
            elif self.STATE == States.MOVE_TOWARD_GOAL:
                with socketLock:
                    print("DistToGoal: " + str(self.video.distanceToGoal) + " Threshold: " + str(self.video.distanceToGoalThreshold))
                    if self.video.distanceToGoal < self.video.distanceToGoalThreshold:
                        print("We made it")
                    else:
                        # drive straight
                        self.sock.sendall("a drive_straight(100)".encode())
                        print("Sent command")
                        result = self.sock.recv(128)
                        print(result)
                        sleep(1)
                        self.sock.sendall("a drive_straight(0)".encode())
                        print("Sent command")
                        result2 = self.sock.recv(128)
                        print(result2)



        # END OF CONTROL LOOP

        # First stop any other threads talking to the robot
        self.sensors.RUNNING = False
        self.video.RUNNING = False

        sleep(1)  # Wait for threads to wrap up

        # Need to disconnect
        """ The c command stops the robot and disconnects.  The stop command will also reset the Create's mode to a battery safe PASSIVE.  It is very important to use this command!"""
        with socketLock:
            self.sock.sendall("c".encode())
            print(self.sock.recv(128))
            self.sock.close()

        # If the user didn't request to halt, we should stop listening anyways
        self.listener.stop()

        # self.sensors.join()
        # self.video.join()

    def on_press(self, key):
        # WARNING: DO NOT attempt to use the socket directly from here
        try:
            print('alphanumeric key {0} pressed'.format(key.char))
        except AttributeError:
            print('special key {0} pressed'.format(key))

    def on_release(self, key):
        # WARNING: DO NOT attempt to use the socket directly from here
        print('{0} released'.format(key))
        if key == keyboard.Key.ctrl:
            # Stop listener
            self.RUNNING = False
            self.sensors.RUNNING = False
            self.video.RUNNING = False
            return False
        elif key == keyboard.Key.shift:
            if self.video.trackBeach:
                self.video.trackBeach = False
            else:
                self.video.trackBeach = True
        elif key == keyboard.Key.cmd:
            if self.video.raceMode:
                self.video.raceMode = False
            else:
                self.video.raceMode = True


# END OF STATEMACHINE


class Sensing(threading.Thread):
    def __init__(self, socket):
        threading.Thread.__init__(self)  # MUST call this to make sure we setup the thread correctly
        self.RUNNING = True
        self.sock = socket

    def run(self):
        while self.RUNNING:
            sleep(0.1)
            # This is where I would get a sensor update
            # Store it in this class
            # You can change the polling frequency to optimize performance, don't forget to use socketLock
            with socketLock:
                self.sock.sendall("a distance".encode())
                print(self.sock.recv(128))


# END OF SENSING

class ImageProc(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)  # MUST call this to make sure we setup the thread correctly
        global IP_ADDRESS
        self.IP_ADDRESS = IP_ADDRESS
        self.PORT = 8081
        self.RUNNING = True
        self.latestImg = []
        self.hsvImg = []
        self.feedback = []
        self.resultHSV = []
        self.maskHSV = []
        self.trackBeach = False
        self.hsvThresholds = {'low_hue': 20, 'high_hue': 40, 'low_saturation': 110, 'high_saturation': 130,
                              'low_value': 140, 'high_value': 160,
                              'beachBall': {'low_hue': 92, 'high_hue': 125, 'low_saturation': 100,
                                            'high_saturation': 255, 'low_value': 100, 'high_value': 255},
                              'greenCone': {'low_hue': 49, 'high_hue': 62, 'low_saturation': 151,
                                            'high_saturation': 168, 'low_value': 102, 'high_value': 120},
                              'orangeRobot': {'low_hue': 10, 'high_hue': 25, 'low_saturation': 215,
                                            'high_saturation': 240, 'low_value': 240, 'high_value': 255},
                              'pinkRobot': {'low_hue': 150, 'high_hue': 179, 'low_saturation': 130,
                                              'high_saturation': 160, 'low_value': 215, 'high_value': 250}}
        self.thresholds = {'low_red': 168, 'high_red': 228, 'low_green': 207, 'high_green': 255, 'low_blue': 14,
                           'high_blue': 74}
        self.objectXPos = 0
        self.objectYPos = 0
        self.accessedPos = False
        self.raceMode = False
        self.cam = cv2.VideoCapture(0)
        self.clickedX = None
        self.clickedY = None
        self.distanceToGoal = None
        self.currentHeading = None
        self.currentHeadingCoords = {'x': None, 'y': None}
        self.coordDifference = {'x': None, 'y': None}
        self.desiredHeading = None
        self.cX = 0
        self.cY = 0
        self.cXpink = 0
        self.cYpink = 0
        self.distanceToGoalThreshold = 20
        self.headingAngleThreshold = 10

    def run(self):
        url = "http://" + self.IP_ADDRESS + ":" + str(self.PORT)
        stream = urllib.request.urlopen(url)
        while (self.RUNNING):
            sleep(0.1)
            bytes = b''
            '''while self.RUNNING:
                bytes += stream.read(8192)  # image size is about 40k bytes, so this loops about 5 times
                a = bytes.find(b'\xff\xd8')
                b = bytes.find(b'\xff\xd9')
                if a > b:
                    bytes = bytes[b + 2:]
                    continue
                if a != -1 and b != -1:
                    jpg = bytes[a:b + 2]
                    # bytes= bytes[b+2:]
                    # print("found image", a, b, len(bytes))
                    break
            img = cv2.imdecode(numpy.fromstring(jpg, dtype=numpy.uint8), cv2.IMREAD_COLOR)
            '''
            # Resize to half size so that image processing is faster
            retValue, img = self.cam.read()
            img = cv2.resize(img, ((int)(len(img[0]) / 6), (int)(len(img) / 6)))
            # if self.clickedX is not None and self.clickedY is not None:
            #     cv2.circle(self.latestImg, (self.clickedX, self.clickedY), 7, (255, 255, 255), -1)

            with imageLock:
                self.latestImg = copy.deepcopy(img)  # Make a copy not a reference

            self.doImgProc(img)  # pass by reference for all non-primitve types in Python

            # after image processing you can update here to see the new version
            with imageLock:
                self.feedback = copy.deepcopy(img)
                self.hsvFeedback = copy.deepcopy(self.hsvImg)

    def setThresh(self, name, value):
        self.thresholds[name] = value

    def setHsvThresh(self, name, value):
        if self.trackBeach:
            self.hsvThresholds['beachBall'][name] = value
        elif self.raceMode:
            self.hsvThresholds['greenCone'][name] = value
        else:
            self.hsvThresholds[name] = value

    def click(self, event, x, y, flags, params):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.clickedX = x
            self.clickedY = y

    def doImgProc(self, imgToModify):
        pixel = self.latestImg[60, 80]
        print("pixel (160, 120) is ", pixel, "in B,G,R order.")
        self.accessedPos = False

        self.hsvImg = cv2.cvtColor(imgToModify, cv2.COLOR_BGR2HSV)
        # TODO: Work here
        self.resultHSV = self.latestImg
        binaryImage = numpy.zeros((len(self.latestImg), len(self.latestImg[0]), 1), numpy.uint8)
        binaryImage2 = numpy.zeros((len(self.latestImg), len(self.latestImg[0]), 1), numpy.uint8)
        for y in range(len(self.latestImg)):
            for x in range(len(self.latestImg[0])):
                #track roomba
                if self.hsvImg[y, x][2] >= self.hsvThresholds['orangeRobot']['low_value'] and self.hsvImg[y, x][2] <= \
                        self.hsvThresholds['orangeRobot']['high_value'] and self.hsvImg[y, x][1] >= \
                        self.hsvThresholds['orangeRobot']['low_saturation'] and self.hsvImg[y, x][1] <= \
                        self.hsvThresholds['orangeRobot']['high_saturation'] and self.hsvImg[y, x][0] >= \
                        self.hsvThresholds['orangeRobot']['low_hue'] and self.hsvImg[y, x][0] <= \
                        self.hsvThresholds['orangeRobot']['high_hue']:
                    self.resultHSV[y, x][2] = 255
                    self.resultHSV[y, x][1] = 0
                    self.resultHSV[y, x][0] = 0
                    binaryImage[y, x] = 255
                    imgToModify[y, x][2] = 255
                    imgToModify[y, x][0] = 0
                    imgToModify[y, x][1] = 0
                #track pink paper
                if self.hsvImg[y, x][2] >= self.hsvThresholds['pinkRobot']['low_value'] and self.hsvImg[y, x][2] <= \
                        self.hsvThresholds['pinkRobot']['high_value'] and self.hsvImg[y, x][1] >= \
                        self.hsvThresholds['pinkRobot']['low_saturation'] and self.hsvImg[y, x][1] <= \
                        self.hsvThresholds['pinkRobot']['high_saturation'] and self.hsvImg[y, x][0] >= \
                        self.hsvThresholds['pinkRobot']['low_hue'] and self.hsvImg[y, x][0] <= \
                        self.hsvThresholds['pinkRobot']['high_hue']:
                    self.resultHSV[y, x][2] = 255
                    self.resultHSV[y, x][1] = 0
                    self.resultHSV[y, x][0] = 0
                    binaryImage2[y, x] = 255
                    imgToModify[y, x][2] = 255
                    imgToModify[y, x][0] = 0
                    imgToModify[y, x][1] = 0

        # Maybe improve here with dilate/erodes on rhw binaryImage
        kernel = numpy.ones((5, 5), numpy.uint8)
        kernel2 = numpy.ones((5, 5), numpy.uint8)
        binaryImage = cv2.dilate(binaryImage, kernel)
        binaryImage = cv2.erode(binaryImage, kernel)
        binaryImage2 = cv2.dilate(binaryImage2, kernel2)
        binaryImage2 = cv2.erode(binaryImage2, kernel2)

        # find contours in the thresholded image
        cnts = cv2.findContours(binaryImage, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if imutils.is_cv2() else cnts[1]

        cnts2 = cv2.findContours(binaryImage2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts2 = cnts2[0] if imutils.is_cv2() else cnts2[1]

        # loop over the contours
        for c in cnts:
            # compute the center of the contour
            M = cv2.moments(c)
            try:
                self.cX = int(M["m10"] / M["m00"])
                self.cY = int(M["m01"] / M["m00"])
                # information below is used to help the roomba figure out where the object it relative the roomba's position
                # place best object into this:
                self.objectXPos = self.cX
                self.objectYPos = self.cY
                self.accessedPos = True  # set this to true to let state machine know that you've already accessed this position

            except ZeroDivisionError:
                print("found something with zero pixels")
                continue
            # draw the contour and center of the shape on the image
            cv2.drawContours(self.resultHSV, [c], -1, (0, 255, 0), 2)
            cv2.circle(self.resultHSV, (self.cX, self.cY), 7, (255, 255, 255), -1)
            cv2.putText(self.resultHSV, "center", (self.cX - 20, self.cY - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            self.currentHeadingCoords['x'] = self.cX
            self.currentHeadingCoords['y'] = self.cY
            if self.clickedX is not None and self.clickedY is not None:
                self.distanceToGoal = math.hypot((self.clickedY - self.cY),(self.clickedX - self.cX))
                self.desiredHeading = math.degrees(math.atan2((self.clickedY-self.cY),(self.clickedX-self.cX)))
                cv2.putText(self.resultHSV, str(round(self.distanceToGoal, 4))+ "|" + str(self.desiredHeading), (10,50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1, cv2.LINE_AA)
        # loop over the contours
        for c2 in cnts2:
            # compute the center of the contour
            M = cv2.moments(c2)
            try:
                self.cXpink = int(M["m10"] / M["m00"])
                self.cYpink = int(M["m01"] / M["m00"])
            except ZeroDivisionError:
                print("found something with zero pixels")
                continue
            # draw the contour and center of the shape on the image
            cv2.drawContours(self.resultHSV, [c2], -1, (0, 255, 0), 2)
            cv2.circle(self.resultHSV, (self.cXpink, self.cYpink), 7, (255, 255, 255), -1)
            cv2.putText(self.resultHSV, "center2", (self.cXpink - 20, self.cYpink - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)



# END OF IMAGEPROC


if __name__ == "__main__":

    cv2.namedWindow("Create View", flags=cv2.WINDOW_AUTOSIZE)
    cv2.moveWindow("Create View", 21, 21)

    cv2.namedWindow('sliders')
    cv2.moveWindow('sliders', 680, 21)

    cv2.namedWindow('HSV Sliders')
    cv2.moveWindow('HSV Sliders', 400, 21)

    sm = StateMachine()
    sm.start()

    cv2.setMouseCallback("Create View", sm.video.click, sm.video)

    # Probably safer to do this on the main thread rather than in ImgProc init
    cv2.createTrackbar('low_red', 'sliders', sm.video.thresholds['low_red'], 255,
                       lambda x: sm.video.setThresh('low_red', x))
    cv2.createTrackbar('high_red', 'sliders', sm.video.thresholds['high_red'], 255,
                       lambda x: sm.video.setThresh('high_red', x))
    cv2.createTrackbar('low_green', 'sliders', sm.video.thresholds['low_green'], 255,
                       lambda x: sm.video.setThresh('low_green', x))
    cv2.createTrackbar('high_green', 'sliders', sm.video.thresholds['high_green'], 255,
                       lambda x: sm.video.setThresh('high_green', x))
    cv2.createTrackbar('low_blue', 'sliders', sm.video.thresholds['low_blue'], 255,
                       lambda x: sm.video.setThresh('low_blue', x))
    cv2.createTrackbar('high_blue', 'sliders', sm.video.thresholds['high_blue'], 255,
                       lambda x: sm.video.setThresh('high_blue', x))

    # Implement sliders to automatically set hsv thresholds for each color
    cv2.createTrackbar('low_hue', 'HSV Sliders', sm.video.hsvThresholds['low_hue'], 179,
                       lambda x: sm.video.setHsvThresh('low_hue', x))
    cv2.createTrackbar('high_hue', 'HSV Sliders', sm.video.hsvThresholds['high_hue'], 179,
                       lambda x: sm.video.setHsvThresh('high_hue', x))
    cv2.createTrackbar('low_saturation', 'HSV Sliders', sm.video.hsvThresholds['low_saturation'], 255,
                       lambda x: sm.video.setHsvThresh('low_saturation', x))
    cv2.createTrackbar('high_saturation', 'HSV Sliders', sm.video.hsvThresholds['high_saturation'], 255,
                       lambda x: sm.video.setHsvThresh('high_saturation', x))
    cv2.createTrackbar('low_value', 'HSV Sliders', sm.video.hsvThresholds['low_value'], 255,
                       lambda x: sm.video.setHsvThresh('low_value', x))
    cv2.createTrackbar('high_value', 'HSV Sliders', sm.video.hsvThresholds['high_value'], 255,
                       lambda x: sm.video.setHsvThresh('high_value', x))

    while len(sm.video.latestImg) == 0:
        sleep(1)

    while (sm.RUNNING):
        with imageLock:
            cv2.imshow("Create View", sm.video.latestImg)
            if sm.video.clickedX is not None and sm.video.clickedY is not None:
                cv2.circle(sm.video.latestImg, (sm.video.clickedX, sm.video.clickedY), 7, (255, 255, 255), -1)
            cv2.imshow("sliders", sm.video.feedback)

            cv2.imshow("HSV Sliders", sm.video.resultHSV)
        cv2.waitKey(5)

    cv2.destroyAllWindows()

    sleep(1)

    # sm.join()
