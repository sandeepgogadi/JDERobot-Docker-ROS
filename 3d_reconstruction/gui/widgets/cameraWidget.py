
from PyQt5.QtCore import QSize
from PyQt5.QtGui import QImage, QPixmap
import cv2


class CameraWidget:
    IMG_WIDTH=320
    IMG_HEIGHT=240

    def __init__(self,winParent):
        self.winParent=winParent
        self.labelImageLeft=winParent.imageLeft
        self.labelImageRight=winParent.imageRight
        self.labelImageRightFiltered = winParent.imageRightFiltered
        self.labelImageLeftFiltered = winParent.imageLeftFiltered
        
    '''
    def initImages(self):
        imgLeft = self.winParent.getSensor().getImageLeft()
        self.setLeftImageFiltered(imgLeft)
        imgRight = self.winParent.getSensor().getImageRight()
        self.setRightImageFiltered(imgRight)'''

    def updateImage(self):

        imgLeft = self.winParent.getSensor().getImageLeft()
        if imgLeft is not None:
            resized = cv2.resize(imgLeft,(self.IMG_WIDTH,self.IMG_HEIGHT))
            image = QImage(resized.data, resized.shape[1], resized.shape[0], resized.shape[1]*resized.shape[2], QImage.Format_RGB888);
            size=QSize(imgLeft.shape[1],imgLeft.shape[0])
            #self.label.resize(size)
            self.labelImageLeft.setPixmap(QPixmap.fromImage(image))

        imgRight = self.winParent.getSensor().getImageRight()
        if imgRight is not None:
            resized = cv2.resize(imgRight,(self.IMG_WIDTH,self.IMG_HEIGHT))
            image = QImage(resized.data, resized.shape[1], resized.shape[0], resized.shape[1]*resized.shape[2], QImage.Format_RGB888);
            size=QSize(imgRight.shape[1],imgRight.shape[0])
            #self.label.resize(size)
            self.labelImageRight.setPixmap(QPixmap.fromImage(image))


        #print the filtered images

        imgLeftFiltered = self.getLeftImageFiltered()
        if imgLeftFiltered is not None:
            resized = cv2.resize(imgLeftFiltered,(self.IMG_WIDTH,self.IMG_HEIGHT))
            image = QImage(resized.data, resized.shape[1], resized.shape[0], resized.shape[1]*resized.shape[2], QImage.Format_RGB888);
            size=QSize(imgLeftFiltered.shape[1],imgLeftFiltered.shape[0])
            #self.label.resize(size)
            self.labelImageLeftFiltered.setPixmap(QPixmap.fromImage(image))

        imgRightFiltered = self.getRightImageFiltered()
        if imgRightFiltered is not None:
            resized = cv2.resize(imgRightFiltered,(self.IMG_WIDTH,self.IMG_HEIGHT))
            image = QImage(resized.data, resized.shape[1], resized.shape[0], resized.shape[1]*resized.shape[2], QImage.Format_RGB888);
            size=QSize(imgRightFiltered.shape[1],imgRightFiltered.shape[0])
            #self.label.resize(size)
            self.labelImageRightFiltered.setPixmap(QPixmap.fromImage(image))
        '''
        def setRightImageFiltered(self, image):
        self.winParent.getAlgorithm().lock.acquire()
        size=image.shape
        if len(size) == 2:
            image=cv2.cvtColor(image,cv2.COLOR_GRAY2BGR)
        self.imageRight=image
        self.winParent.getAlgorithm().lock.release()


    def setLeftImageFiltered(self, image):
        self.winParent.getAlgorithm().lock.acquire()
        size=image.shape
        if len(size) == 2:
            image=cv2.cvtColor(image,cv2.COLOR_GRAY2BGR)
        self.imageLeft=image
        self.winParent.getAlgorithm().lock.release()'''

    def getRightImageFiltered(self):
        self.winParent.getAlgorithm().lock.acquire()
        tempImage=self.winParent.getAlgorithm().imageRight
        self.winParent.getAlgorithm().lock.release()
        return tempImage

    def getLeftImageFiltered(self):
        self.winParent.getAlgorithm().lock.acquire()
        tempImage=self.winParent.getAlgorithm().imageLeft
        self.winParent.getAlgorithm().lock.release()
        return tempImage
