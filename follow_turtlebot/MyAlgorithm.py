import threading
import time
from datetime import datetime
import cv2
import numpy as np
from math import pi

from sensors.cameraFilter import CameraFilter
from parallelIce.navDataClient import NavDataClient
from parallelIce.cmdvel import CMDVel
from parallelIce.extra import Extra
from parallelIce.pose3dClient import Pose3DClient


time_cycle = 80


class MyAlgorithm(threading.Thread):
    class pid(object):
        def __init__(self, kp, kd, ki):
            # Constant of PID control
            self.kp = kp
            self.kd = kd
            self.ki = ki
            self.error = 0
            self.acumulate_error = 0

        def calculateU(self, e):
            proportional = self.kp * e
            derivate = self.kd * (e - self.error)
            self.acumulate_error = self.acumulate_error + e
            integral = self.ki * (self.acumulate_error)
            u = -(proportional) - (derivate) - (integral)
            self.error = e
            return u

    def __init__(self, camera, navdata, pose, cmdvel, extra):
        self.camera = camera
        self.navdata = navdata
        self.pose = pose
        self.cmdvel = cmdvel
        self.extra = extra
        self.minError = 0.01

        # Constructor PID
        self.pidX = self.pid(2.655, 0.000112, 0.00029)
        self.pidY = self.pid(2.655, 0.000112, 0.00029)

        self.stop_event = threading.Event()
        self.kill_event = threading.Event()
        self.lock = threading.Lock()
        threading.Thread.__init__(self, args=self.stop_event)

    def run(self):

        self.stop_event.clear()

        while (not self.kill_event.is_set()):

            start_time = datetime.now()

            if not self.stop_event.is_set():
                self.execute()

            finish_Time = datetime.now()

            dt = finish_Time - start_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
            #print (ms)
            if (ms < time_cycle):
                time.sleep((time_cycle - ms) / 1000.0)

    def stop(self):
        self.stop_event.set()

    def play(self):
        if self.is_alive():
            self.stop_event.clear()
        else:
            self.start()

    def kill(self):
        self.kill_event.set()

    def execute(self):
       # Add your code here
        detec = False

        input_image = self.camera.getImage()

        if input_image is not None:
            blur = cv2.GaussianBlur(input_image, (3, 3), 0)
            color_HSV = cv2.cvtColor(blur, cv2.COLOR_RGB2HSV)

            H_max = 4.3*(180/(2*pi))
            H_min = 1.98*(180/(2*pi))
            S_max = 0.8*(255/1)
            S_min = 0.3*(255/1)
            V_max = 255.5
            V_min = 51.81

            bk_image = cv2.inRange(color_HSV, np.array(
                [H_min, S_min, V_min]), np.array([H_max, S_max, V_max]))

            kernel = np.ones((19, 19), np.uint8)
            image_HSV_close = cv2.morphologyEx(bk_image, cv2.MORPH_CLOSE, kernel)
            self.camera.setThresoldImage(image_HSV_close)

            image_HSV_cp = np.copy(image_HSV_close)
            input_image_cp = np.copy(input_image)
            image, contours, hierarchy = cv2.findContours(
                image_HSV_close, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
            print(len(contours))

            if len(contours) != 0:
                cnt = contours[0]
                approx = cv2.approxPolyDP(cnt, 0.1*cv2.arcLength(cnt, True), True)
                cnt_approx = cv2.approxPolyDP(cnt, 3, True)
                x, y, w, h = cv2.boundingRect(cnt_approx)
                image_contour = cv2.rectangle(input_image_cp, (x, y), (x+w, y+h), (0, 255, 0), 2)
                self.camera.setColorImage(input_image_cp)
                detec = True
            else:
                self.camera.setColorImage(blur)

        if (detec == True):
            print ("Turtle Detected")
            # central position of the image in the color filter
            ini_pos = np.array([160, -90])
            coord_tur = np.array([x+w/2, -y+h/2])
            print (coord_tur)
            vect_1 = ini_pos - coord_tur
            vel_x = vect_1[0]*0.01
            vel_y = vect_1[1]*(-0.01)
            print (vel_x, vel_y)

            # sending speed commands
            if abs(vel_x) < self.minError and abs(vel_y) < self.minError:
                self.cmdvel.sendCMDVel(0, 0, 0, 0, 0, 0)
                print ("Turtle Stop")
            elif abs(vel_y) < self.minError and abs(vel_x) > self.minError:
                self.cmdvel.sendCMDVel(0, vel_x, 0, 0, 0, 0)
                print ("Following X")
            elif abs(vel_x) < self.minError and abs(vel_y) > self.minError:
                self.cmdvel.sendCMDVel(vel_y, 0, 0, 0, 0, 0)
                print ("Following Y")
            else:
                self.cmdvel.sendCMDVel(vel_y, vel_x, 0, 0, 0, 0)
                print ("Following turtle")
