import threading
import time
from datetime import datetime
import numpy as np

import math
import jderobot
from Beacon import Beacon

from parallelIce.cameraClient import CameraClient
from parallelIce.navDataClient import NavDataClient
from parallelIce.cmdvel import CMDVel
from parallelIce.extra import Extra
from parallelIce.pose3dClient import Pose3DClient

time_cycle = 80
i = 0


class MyAlgorithm(threading.Thread):

    def __init__(self, camera, navdata, pose, cmdvel, extra):
        self.camera = camera
        self.navdata = navdata
        self.pose = pose
        self.cmdvel = cmdvel
        self.extra = extra

        self.beacons = []
        self.initBeacons()
        self.minError = 0.01

        self.stop_event = threading.Event()
        self.kill_event = threading.Event()
        self.lock = threading.Lock()
        threading.Thread.__init__(self, args=self.stop_event)

    def initBeacons(self):
        self.beacons.append(Beacon('baliza1', jderobot.Pose3DData(
            0, 5, 0, 0, 0, 0, 0, 0), False, False))
        self.beacons.append(Beacon('baliza2', jderobot.Pose3DData(
            5, 0, 0, 0, 0, 0, 0, 0), False, False))
        self.beacons.append(Beacon('baliza3', jderobot.Pose3DData(
            0, -5, 0, 0, 0, 0, 0, 0), False, False))
        self.beacons.append(
            Beacon('baliza4', jderobot.Pose3DData(-5, 0, 0, 0, 0, 0, 0, 0), False, False))
        self.beacons.append(Beacon('baliza5', jderobot.Pose3DData(
            10, 0, 0, 0, 0, 0, 0, 0), False, False))
        self.beacons.append(Beacon('inicio', jderobot.Pose3DData(
            0, 0, 0, 0, 0, 0, 0, 0), False, False))

    def getNextBeacon(self):
        for beacon in self.beacons:
            if beacon.isReached() == False:
                return beacon

        return None

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
        # use: self.pose.getPose3d().x
        #      self.pose.getPose3d().y
        # to get the coordinates of the drone in the x,y plane
        global i
        if (len(self.beacons) > i):
            act_pos = np.array(
                [self.pose.getPose3d().x, self.pose.getPose3d().y, self.pose.getPose3d().z])
            coord_b1 = np.array([self.beacons[i].getPose().x,
                                 self.beacons[i].getPose().y, self.beacons[i].getPose().z])
#            print (act_pos)
#            print (coord_b1)
            vect_1 = coord_b1 - act_pos
#            print (vect_1)
            vel_1 = vect_1
#            print (vel_1)

            if abs(vect_1[0]) < self.minError and abs(vect_1[1]) < self.minError:
                self.cmdvel.sendCMDVel(0, 0, 0, 0, 0, 0)
                i = i+1
                if (i < 6):
                    print ("Beacon Reached: ", i)
                else:
                    print ("Initial position")
            else:
                self.cmdvel.sendCMDVel(vel_1[0], vel_1[1], 0, 0, 0, 0)
#                print ("1")
        pass
