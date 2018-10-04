import threading
import time
from datetime import datetime
import cv2
import numpy as np

from sensors.cameraFilter import CameraFilter


time_cycle = 80


class MyAlgorithm(threading.Thread):

    def __init__(self, camera):
        self.camera = camera

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
            # print (ms)
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

        input_image = self.camera.getImage()
        if input_image is not None:
            a = 50
            img_rgb = np.copy(input_image)
            img_blur = cv2.GaussianBlur(img_rgb, (5, 5), 0)
            lower_red = np.array([0, 0, 0])
            upper_red = np.array([255, 70, 70])
            mask_red = cv2.inRange(img_rgb, lower_red, upper_red)
            lower_blue = np.array([0, 0, 0])
            upper_blue = np.array([70, 70, 255])
            mask_blue = cv2.inRange(img_rgb, lower_blue, upper_blue)
            img_mask = mask_red + mask_blue

            self.camera.setColorImage(img_rgb)
            '''
            If you want show a threshold image (black and white image)
            self.camera.setThresholdImage(bk_image)
            '''
            self.camera.setThresholdImage(img_mask)
