
# -*- coding: utf-8 -*-

import sys
from docutils.nodes import image
from sensors import sensor
import numpy as np
import time
import threading
from pyProgeo.progeo import Progeo
import cv2


class MyAlgorithm():

    def __init__(self, sensor):
        self.sensor = sensor
        self.imageRight = np.zeros((320, 240, 3), np.uint8)
        self.imageLeft = np.zeros((320, 240, 3), np.uint8)
        self.lock = threading.Lock()
        print("Left Camera Configuration File:")
        self.camLeftP = Progeo(sys.argv[1], "CamACalibration")
        print("Rigth Camera Configuration File:")
        self.camRightP = Progeo(sys.argv[1], "CamBCalibration")
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.done = False
        self.time_process = None
        self.n_points = 0
        self.counter = 0
        self.min_corr = 0.9
        self.block_size = 31
        self.lbda1 = 3
        self.lbda2 = -3


    def setRightImageFiltered(self, image):
        self.lock.acquire()
        size = image.shape
        if len(size) == 2:
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        self.imageRight = image
        self.lock.release()

    def setLeftImageFiltered(self, image):
        self.lock.acquire()
        size = image.shape
        if len(size) == 2:
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        self.imageLeft = image
        self.lock.release()

    def get_vector(self, a, b, lr):
        if lr == 'l':
            pointInOpt = self.camLeftP.graficToOptical([b, a, 1])
            point3d = self.camLeftP.backproject(pointInOpt)
            point3d_cam = self.camLeftP.getCameraPosition()
        elif lr == 'r':
            pointInOpt = self.camRightP.graficToOptical([b, a, 1])
            point3d = self.camRightP.backproject(pointInOpt)
            point3d_cam = self.camRightP.getCameraPosition()
        vector = np.append(point3d_cam, 1) - point3d
        point3d_1 = (self.lbda1 * vector) + point3d
        point3d_2 = (self.lbda2 * vector) + point3d
        return point3d_1, point3d_2

    def get_border(self, imageLeft, imageRight):
        imageLeft = cv2.cvtColor(imageLeft, cv2.COLOR_BGR2GRAY)
        imageRight = cv2.cvtColor(imageRight, cv2.COLOR_BGR2GRAY)
        imageLeft = cv2.Canny(imageLeft, 100, 200)
        imageRight = cv2.Canny(imageRight, 100, 200)
        return imageLeft, imageRight

    def correlation(self, patch1, patch2):
        product = np.mean((patch1 - patch1.mean()) * (patch2 - patch2.mean()))
        stds = patch1.std() * patch2.std()
        if stds == 0:
            return 0
        else:
            product /= stds
            return product

    def get_point_min_distance(self, p1, p2, p3, p4):

        EPS = sys.float_info.epsilon
        pa, pb, p13, p43, p21 = [None, None, None], [None, None, None], [None, None, None], [None, None, None], [None, None, None]
        p13[0] = p1[0] - p3[0]
        p13[1] = p1[1] - p3[1]
        p13[2] = p1[2] - p3[2]
        p43[0] = p4[0] - p3[0]
        p43[1] = p4[1] - p3[1]
        p43[2] = p4[2] - p3[2]
        if ((abs(p43[0]) < EPS) & (abs(p43[1]) < EPS) & (abs(p43[2]) < EPS)):
            return False, pa, pb
        p21[0] = p2[0] - p1[0]
        p21[1] = p2[1] - p1[1]
        p21[2] = p2[2] - p1[2]
        if ((abs(p21[0]) < EPS) & (abs(p21[1]) < EPS) & (abs(p21[2]) < EPS)):
            return False, pa, pb
        d1343 = p13[0] * p43[0] + p13[1] * p43[1] + p13[2] * p43[2]
        d4321 = p43[0] * p21[0] + p43[1] * p21[1] + p43[2] * p21[2]
        d1321 = p13[0] * p21[0] + p13[1] * p21[1] + p13[2] * p21[2]
        d4343 = p43[0] * p43[0] + p43[1] * p43[1] + p43[2] * p43[2]
        d2121 = p21[0] * p21[0] + p21[1] * p21[1] + p21[2] * p21[2]

        denom = d2121 * d4343 - d4321 * d4321

        numer = d1343 * d4321 - d1321 * d4343
        mua = numer / denom
        mub = (d1343 + d4321 * (mua)) / d4343

        pa[0] = p1[0] + mua * p21[0]
        pa[1] = p1[1] + mua * p21[1]
        pa[2] = p1[2] + mua * p21[2]
        pb[0] = p3[0] + mub * p43[0]
        pb[1] = p3[1] + mub * p43[1]
        pb[2] = p3[2] + mub * p43[2]

        if (abs(denom) < EPS) | (pa[0]==None) | (pa[1]==None) | (pa[2]==None) | (pb[0]==None) | (pb[1]==None) | (pb[2]==None):
            return False, pa, pb
        else:
            return True, pa, pb


    def execute(self):

        # OBTAINING IMAGES
        imageLeft = self.sensor.getImageLeft()
        imageRight = self.sensor.getImageRight()
        imageLeft_C = imageLeft.copy()

        # INIT VARIABLES
        pd = int(self.block_size / 2.)
        n, m, _ = imageLeft.shape
        mask = np.zeros((n, m), dtype=np.uint8)

        if self.done:
            return
        init_time = time.clock()

        # OBTAINING EDGES OF IMAGE
        imageLeft_border, imageRight_border = self.get_border(imageLeft, imageRight)

        # IMAGES TO SHOW
        imageLeft_show = cv2.cvtColor(imageLeft_border, cv2.COLOR_GRAY2BGR)
        imageRight_show = imageLeft_show.copy() * 0

        # MAIN PROCESS
        self.n_points = 0 #accepted points
        idxs_borderL = np.where(imageLeft_border != 0)  # Index of edge values in the left image
        total_points = idxs_borderL[0].shape[0]

        for idx_l in range(total_points):
            i, j = idxs_borderL[0][idx_l], idxs_borderL[1][idx_l]
            cv2.circle(imageLeft_show, (j, i), 1, (255, 0, 0), -1)
            corr, max_corr = 0, 0
            PNEW = np.asarray([0, 0])  # corresponding point in the right image
            pf = [None, None, None]  # 3D final point

            point3d_1, point3d_2 = self.get_vector(i, j, 'l')
            projected1 = self.camRightP.project(point3d_1)
            projected2 = self.camRightP.project(point3d_2)
            P1 = self.camRightP.opticalToGrafic(projected1)
            P2 = self.camRightP.opticalToGrafic(projected2)
            cv2.line(mask, (int(P1[0]), int(P1[1])), (int(P2[0]), int(P2[1])), 255, 3) # epipolar line


            idxs_borderR = np.where((mask != 0) & (imageRight_border != 0))  # edges over epipolar margin
            for idx in range(idxs_borderR[0].shape[0]):
                patchI = imageLeft[i - pd: i + pd + 1, j - pd: j + pd + 1]
                patchD = imageRight[idxs_borderR[0][idx] - pd: idxs_borderR[0][idx] + pd + 1,
                         idxs_borderR[1][idx] - pd: idxs_borderR[1][idx] + pd + 1]

                if not ((patchI.shape[0] != self.block_size) | (patchI.shape[1] != self.block_size) | (
                        patchD.shape[0] != self.block_size) | (patchD.shape[1] != self.block_size)):
                    corr = self.correlation(patchI, patchD)

                if corr > max_corr:
                    max_corr = corr
                    PNEW = [idxs_borderR[0][idx], idxs_borderR[1][idx]]
            mask *= 0


            if max_corr > self.min_corr: # min correlation
                point3d_3, point3d_4 = self.get_vector(PNEW[0], PNEW[1], 'r')
                check, pa, pb = self.get_point_min_distance(point3d_1, point3d_2, point3d_3, point3d_4)

                if check == True:
                    self.n_points += 1
                    cv2.putText(imageRight_show, str(self.n_points) + '/' + str(total_points), (40, 80), self.font, 1, (255, 255, 255), 2, cv2.LINE_AA)
                    cv2.circle(imageLeft_show, (j, i), 1, (0, 255, 0), -1)
                    self.setRightImageFiltered(imageRight_show)
                    pf[0], pf[1], pf[2] = (pa[0] + pb[0]) / 2., (pa[1] + pb[1]) / 2., (pa[2] + pb[2]) / 2.
                    self.sensor.drawPoint(pf, (imageLeft_C[i, j, 0] / 255., imageLeft_C[i, j, 1] / 255., imageLeft_C[i, j, 2] / 255.))


            new_time = time.clock()
            cv2.putText(imageRight_show, str(new_time-init_time) + ' s', (40, 120), self.font, 1, (255, 255, 255), 2, cv2.LINE_AA)
            self.setLeftImageFiltered(imageLeft_show)
            imageRight_show = imageRight_show * 0

            if (idx_l + 1) == total_points:
                self.time_process = str(new_time - init_time)
            if self.time_process != None:
                cv2.putText(imageRight_show, self.time_process + ' s', (40, 170), self.font, 1, (255, 255, 255), 2, cv2.LINE_AA)
