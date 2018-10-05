#!/usr/bin/python
# -*- coding: utf-8 -*-
import threading
import time
from datetime import datetime

import math
import jderobot
import cv2
import numpy as np
import matplotlib.pyplot as plt

time_cycle = 80


class MyAlgorithm(threading.Thread):

    def __init__(self, camera, motors):
        self.camera = camera
        self.motors = motors
        self.threshold_image = np.zeros((640, 360, 3), np.uint8)
        self.color_image = np.zeros((640, 360, 3), np.uint8)
        self.stop_event = threading.Event()
        self.kill_event = threading.Event()
        self.lock = threading.Lock()
        self.threshold_image_lock = threading.Lock()
        self.color_image_lock = threading.Lock()
        threading.Thread.__init__(self, args=self.stop_event)

        self.prev_cte = 0
        self.int_cte = 0
        self.speed = 1
        self.step = 0
        self.params = [0.2, 0.004, 3.0]
        self.dp = [1., 1., 1.]
        self.total_err = 0

    def getImage(self):
        self.lock.acquire()
        img = self.camera.getImage().data
        self.lock.release()
        return img

    def set_color_image(self, image):
        img = np.copy(image)
        if len(img.shape) == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
        self.color_image_lock.acquire()
        self.color_image = img
        self.color_image_lock.release()

    def get_color_image(self):
        self.color_image_lock.acquire()
        img = np.copy(self.color_image)
        self.color_image_lock.release()
        return img

    def set_threshold_image(self, image):
        img = np.copy(image)
        if len(img.shape) == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
        self.threshold_image_lock.acquire()
        self.threshold_image = img
        self.threshold_image_lock.release()

    def get_threshold_image(self):
        self.threshold_image_lock.acquire()
        img = np.copy(self.threshold_image)
        self.threshold_image_lock.release()
        return img

    def run(self):

        while (not self.kill_event.is_set()):
            start_time = datetime.now()
            if not self.stop_event.is_set():
                self.algorithm()
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

    def pid(self, x, width, flag=None):

        kp, ki, kd = self.params
        prev_cte = self.prev_cte
        int_cte = self.int_cte
        cte = width//2 - x
        diff_cte = (cte - prev_cte) / self.speed
        prev_cte = cte
        int_cte = int_cte + cte
        steer = -kp*cte - kd*diff_cte - ki*int_cte
        if flag == 'move':
            self.prev_cte = prev_cte
            self.int_cte = int_cte
            self.total_err += cte**2
            return steer
        else:
            return (self.total_err + cte**2)/self.step

    def twiddle(self, x, width, tol=.001):

        p = self.params
        dp = self.dp
        best_err = self.pid(x, width)

        it = 0
        while sum(dp) > tol:
            for i in range(len(p)):
                p[i] += dp[i]
                err = self.pid(x, width)
                if err < best_err:
                    best_err = err
                    dp[i] *= 1.1
                else:
                    p[i] -= 2 * dp[i]
                    err = self.pid(x, width)
                    if err < best_err:
                        best_err = err
                        dp[i] *= 1.1
                    else:
                        p[i] += dp[i]
                        dp[i] *= 0.9
            it += 1

        self.params = p
        self.dp = dp

    def algorithm(self):
        # GETTING THE IMAGES
        image = self.getImage()

        print "Runing"

        img = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        height, width, channels = img.shape
        crop = img[230:400, :, :]
        lower = np.array([0, 235, 60], dtype='uint8')
        upper = np.array([180, 255, 255], dtype='uint8')
        mask = cv2.inRange(crop, lower, upper)
        extraction = cv2.bitwise_and(crop, crop, mask=mask)
        m = cv2.moments(mask, False)
        try:
            x, y = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            x, y = width//2, height//2
        #extraction_img = cv2.circle(extraction,(int(x), int(y)), 2,(0,255,0),3)
        self.step += 1
        # error < 0, steer left
        #self.twiddle(x, width)
        steer = self.pid(x, width, flag='move')
        w = -steer*0.01
        if abs(w) < .0001:
            v = 20
        elif abs(w) < .001:
            v = 15
        elif abs(w) < .01:
            v = 10  # 5
        elif abs(w) < .1:
            v = 5  # 2.5
        elif abs(w) < .5:
            v = 2.5
        else:
            v = 1  # 1
        self.speed = v
        # w > 0, steer left
        #self.speed = v
        print(v, w, x, self.params, self.dp)

        self.motors.sendV(v)
        self.motors.sendW(w)

        # SHOW THE FILTERED IMAGE ON THE GUI
        self.set_threshold_image(mask)
