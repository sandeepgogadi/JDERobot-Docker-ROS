import cv2
import numpy as np

def draw_lines(img_left, img_right, lines, pts_left, pts_right):
    h,w = img_left.shape
    img_left = cv2.cvtColor(img_left, cv2.COLOR_GRAY2BGR)
    img_right = cv2.cvtColor(img_right, cv2.COLOR_GRAY2BGR)

    for line, pt_left, pt_right in zip(lines, pts_left, pts_right):
        x_start,y_start = map(int, [0, -line[2]/line[1] ])
        x_end,y_end = map(int, [w, -(line[2]+line[0]*w)/line[1] ])
        color = tuple(np.random.randint(0,255,2).tolist())
        cv2.line(img_left, (x_start,y_start), (x_end,y_end), color,1)
        cv2.circle(img_left, tuple(pt_left), 5, color, -1)
        cv2.circle(img_right, tuple(pt_right), 5, color, -1)

    return img_left, img_right

def get_descriptors(gray_image, feature_type):
    if feature_type == 'surf':
        feature_extractor = cv2.xfeatures2d.SURF_create()

    elif feature_type == 'sift':
        feature_extractor = cv2.xfeatures2d.SIFT_create()

    else:
        raise TypeError("Invalid feature type; should be either 'surf' or 'sift'")

    keypoints, descriptors = feature_extractor.detectAndCompute(gray_image, None)
    return keypoints, descriptors

if __name__=='__main__':
    args = build_arg_parser().parse_args()
    img_left = cv2.imread(args.img_left,0)  # left image
    img_right = cv2.imread(args.img_right,0)  # right image
    feature_type = args.feature_type

    if feature_type not in ['sift', 'surf']:
        raise TypeError("Invalid feature type; has to be either 'sift' or 'surf'")

    scaling_factor = 1.0
    img_left = cv2.resize(img_left, None, fx=scaling_factor,
                fy=scaling_factor, interpolation=cv2.INTER_AREA)
    img_right = cv2.resize(img_right, None, fx=scaling_factor,
                fy=scaling_factor, interpolation=cv2.INTER_AREA)

    kps_left, des_left = get_descriptors(img_left, feature_type)
    kps_right, des_right = get_descriptors(img_right, feature_type)

    # FLANN parameters
    FLANN_INDEX_KDTREE = 0
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks=50)

    # Get the matches based on the descriptors
    flann = cv2.FlannBasedMatcher(index_params, search_params)
    matches = flann.knnMatch(des_left, des_right, k=2)

    pts_left_image = []
    pts_right_image = []

    # ratio test to retain only the good matches
    for i,(m,n) in enumerate(matches):
        if m.distance < 0.7*n.distance:
            pts_left_image.append(kps_left[m.queryIdx].pt)
            pts_right_image.append(kps_right[m.trainIdx].pt)

    pts_left_image = np.float32(pts_left_image)
    pts_right_image = np.float32(pts_right_image)
    F, mask = cv2.findFundamentalMat(pts_left_image, pts_right_image, cv2.FM_LMEDS)

    # Selecting only the inliers
    pts_left_image = pts_left_image[mask.ravel()==1]
    pts_right_image = pts_right_image[mask.ravel()==1]

    # Drawing the lines on left image and the corresponding feature points on the right image
    lines1 = cv2.computeCorrespondEpilines (pts_right_image.reshape(-1,1,2), 2, F)
    lines1 = lines1.reshape(-1,3)
    img_left_lines, img_right_pts = draw_lines(img_left, img_right, lines1, pts_left_image, pts_right_image)

    # Drawing the lines on right image and the corresponding feature points on the left image
    lines2 = cv2.computeCorrespondEpilines (pts_left_image.reshape(-1,1,2), 1,F)
    lines2 = lines2.reshape(-1,3)
    img_right_lines, img_left_pts = draw_lines(img_right, img_left, lines2, pts_right_image, pts_left_image)


    def writeTxt(self,path,points,color):
        '''
        Funcion para escribir en un txt los puntos con su color
            Escribe de uno en uno
        '''
        with open(path, 'a') as file:
            file.write("%f;%f;%f;%f;%f;%f\n" % (points[0], points[1], points[2],
                                                color[0], color[1], color[2]))



    def findKeypoints(self,imageR,imageL):
        '''
        Funcion que calcula los puntos de interes donde vamos a recrear la escena
        '''
        grayR = cv2.cvtColor(imageR,cv2.COLOR_RGB2GRAY)

        edgesR = cv2.Canny(grayR, 100, 200)
        keypointsR = np.asarray(np.where(edgesR == 255)).T

        keypointsR = np.concatenate((keypointsR, np.ones((keypointsR.shape[0], 1))), 1)

        grayL = cv2.cvtColor(imageL,cv2.COLOR_RGB2GRAY)

        edgesL = cv2.Canny(grayL, 100, 200)
        keypointsL =  np.asarray(np.where(edgesL == 255)).T
        keypointsL = np.concatenate((keypointsL, np.ones((keypointsL.shape[0], 1))), 1)


        self.setRightImageFiltered(cv2.cvtColor(edgesR,cv2.COLOR_GRAY2RGB))
        self.setLeftImageFiltered(cv2.cvtColor(edgesL,cv2.COLOR_GRAY2RGB))

        return keypointsR.astype(np.int), keypointsL.astype(np.int),edgesR,edgesL



    def getVectors(self,pointsUV1,posCam,cam="left"):
        '''
        Calcula los vectores de las rectas en el espacio [XYZ1]
        '''
        if cam=="left":
            if len(pointsUV1.shape)==1: #un unico punto
                pointInOpts = self.camLeftP.graficToOptical(np.array([pointsUV1[1], pointsUV1[0], 1]))
                point3ds = self.camLeftP.backproject(pointInOpts)
                return np.asarray((point3ds[:3]-posCam).tolist()+[1])
            else:
                pointInOpts = np.asarray([self.camLeftP.graficToOptical(np.array([pointIn[1],pointIn[0],1]))
                                          for pointIn in pointsUV1])
                point3ds = np.asarray([self.camLeftP.backproject(pointInOpt) for pointInOpt in pointInOpts])
        else:
            if len(pointsUV1.shape)==1: #un unico punto
                pointInOpts = self.camRightP.graficToOptical(np.array([pointsUV1[1], pointsUV1[0], 1]))
                point3ds = self.camRightP.backproject(pointInOpts)
                return np.asarray((point3ds[:3]-posCam).tolist()+[1])

            else:
                pointInOpts = np.asarray([self.camRightP.graficToOptical(np.array([pointIn[1], pointIn[0], 1]))
                                          for pointIn in pointsUV1])
                point3ds = np.asarray([self.camRightP.backproject(pointInOpt) for pointInOpt in pointInOpts])
        return np.concatenate((point3ds[:,:3]-posCam*np.ones(pointInOpts.shape), np.ones((pointInOpts.shape[0], 1))), 1)


    def getPointsfromline(self,vectors,Origin):
        '''
        Devuelve una lista de puntos XYZ de la recta definida por su vector y su origen
        y=mx+n
        '''
        lst = [[] for N in range(vectors.shape[0])]
        alfas = [10,100,10000]
        for idx,Vxyz in enumerate(vectors):
            for alfa in alfas:
                xyz = Origin + Vxyz[:3]*alfa
                lst[idx].append(np.array([xyz[0],xyz[1],xyz[2],1]))
        return lst



    def getLineEpipolar(self,lstPoints3d,cam):
        '''
        Recibe los puntos XYZ1 -> proyecta a coordenadas pixeles
        Calcula m y n (y=mx+n) y el punto inicial y final del eje horizontal
        '''
        projected_lst_grafic = []
        for idx, xyz in enumerate(lstPoints3d):
            if cam == "right":
                projected = self.camRightP.project(xyz)
                projected_grafic = np.floor(self.camRightP.opticalToGrafic(projected)).astype(np.int)
            if cam == "left":
                projected = self.camLeftP.project(xyz)
                projected_grafic = np.floor(self.camLeftP.opticalToGrafic(projected)).astype(np.int)
            projected_lst_grafic.append(np.array([projected_grafic[1], projected_grafic[0], 1]))

        #Get m y n from y = mx+n
        begin = projected_lst_grafic[0][1]
        end = projected_lst_grafic[2][1]
        m = (projected_lst_grafic[0][0]-projected_lst_grafic[1][0]) / (projected_lst_grafic[0][1]-projected_lst_grafic[1][1])
        n = projected_lst_grafic[0][0]-m*projected_lst_grafic[0][1]
        return m,n,begin,end

    def getPointsfromEpipolar(self,m,n,begin,end,size=5):
        '''
        Funcion encargada de crear todos los puntos pertenecientes a la linea epipolar
        Recibe los parametros de la recta y los puntos donde debe iterar y el grosor de la recta
        '''
        horizontal = np.arange(begin, end+1).reshape(-1, 1)
        inc = int(size/2)
        n_inc = np.arange(-inc + n, n + inc + 1)
        if m == 0:
            # Par estereo canonico
            vertical = np.asarray([np.repeat(n,horizontal.shape[0]) for n in n_inc],dtype=int).reshape(-1,1)
            horizontal_inc = np.repeat(horizontal,n_inc.size,axis=1).T.reshape(-1,1)

            return np.concatenate((vertical,horizontal_inc),axis=1)
        else:
            vertical = np.asarray([np.asarray([m*x+n for x in horizontal],dtype=int) for n in n_inc],dtype=int).reshape(-1,1)
            horizontal_inc = np.repeat(horizontal,n_inc.size,axis=1).T.reshape(-1,1)
            # Limit point coordinate
            idx = np.where(vertical<self.height)[0].tolist()
            if len(idx)>0:
                vertical_limit = vertical[idx,:]
                horizontal_limit = horizontal_inc[idx,:]
            else:
                vertical_limit = vertical
                horizontal_limit = horizontal_inc

            return np.concatenate((vertical_limit,horizontal_limit),axis=1)





    def drawPoint(self,image,lstPoints,idx,color):
        '''
        Cambia el nivel de intensidad de una imagen por el color elegido
        Puedes elegir un indice en concreto si deseas
        '''
        out = image.copy()
        if not idx:
            if len(lstPoints.shape)==1:
                out[lstPoints[0], lstPoints[1]] = color
            else:
                out[lstPoints[:,0], lstPoints[:,1],:] = color
        else:
            out[lstPoints[idx,0],lstPoints[idx,1]]= color
        return out

    def drawLastPoint(self,image,lstPoints,idx,color=(0,255,255)):
        '''
        Dibuja un circulo en la posicion
        '''
        out = image.copy()
        if not idx:
            cv2.circle(out, (lstPoints[1], lstPoints[0]), 3,color, 2)

        else:
            cv2.circle(out, (lstPoints[idx, 1], lstPoints[idx, 0]), 3,color, 2)
        return out






    def triangulate(self,pointR,imageRight,vectorL,idx):
        '''
        Triangula los puntos con sus rectas en 3d y obtiene los 2 puntos
        si la distancia entre estos supera un cierto umbral se utiliza
        '''
        vL = vectorL[idx,:3]
        vR = MyAlgorithm.getVectors(self,pointR,self.OR,"right")[:3]
        # intersec = MyAlgorithm.intersection(self, vR, vL, self.OR, self.OL)
        pa,pb = MyAlgorithm.intersectionEcuation(self, vR, vL, self.OR, self.OL)

        if pa.size != 0:
            if LA.norm(pa-pb)<5:
                color = MyAlgorithm.getColor(self, imageRight, pointR)
                return 0.5*(pa+pb),tuple(color)
            else:
                return np.array([]),None

        else:
            return np.array([]),None




    def getColor(self,image,point):
        '''
        Obtiene el color de la imagen de ese punto
        '''
        point = point.astype(np.int)
        color = image[point[0],point[1],:].astype(np.float32).tolist()
        return (color[0]/255,color[1]/255,color[2]/255)


    def calculateSAD(self,patch1,patch2):
        if patch1.shape[1] < patch2.shape[1]:
            patch2 = patch2[:,0:patch1.shape[1]]

        if patch2.shape[1] < patch1.shape[1]:
            patch1 = patch1[:, 0:patch2.shape[1]]

        dif = abs(patch1-patch2)
        sum = np.sum(dif)
        sum /= 255*dif.size
        return sum


    def matchingPoint(self, keyPointL, lst2matchingR, edgesR,outR, outL, size=(25, 25)):
        '''
        Realiza el maching de un punto de la imagen izqda en todos aquellos puntos perteneciente
        a la linea epiplar derecha
        '''

        alfa = 0.5
        beta = 0.5
        incu = int(size[0] / 2)
        incv = int(size[1] / 2)

        patchL = self.hsvL[keyPointL[0] - incu:keyPointL[0] + incu + 1, keyPointL[1] - incv:keyPointL[1] + incv + 1, :]
        maximum = 0.5
        best = np.zeros([3,])



        # eliminamos repetidos
        for jdx, uvR in enumerate(lst2matchingR):
            # Solo buscamos el matching si es un pixel de contorno
            if edgesR[uvR[0], uvR[1]] == 255:
                patchR = self.hsvR[uvR[0] - incu:uvR[0] + incu + 1, uvR[1] - incv:uvR[1] + incv + 1, :]

                corr =1-( alfa * MyAlgorithm.calculateSAD(self, patchL[:, :, 0], patchR[:, :, 0])\
                       + beta * MyAlgorithm.calculateSAD(self, patchR[:, :, 1], patchR[:, :, 1]))


                if corr > maximum:
                    maximum = corr
                    best = uvR

        outL = MyAlgorithm.drawPoint(self, outL, keyPointL.astype(np.int), None, (0, 255, 0))
        outL_last = MyAlgorithm.drawLastPoint(self, outL, keyPointL.astype(np.int), None)


        if tuple(best) == tuple(np.zeros((3,))):
            print "Point not match: ", keyPointL
            return np.zeros((3,)),outR,outL

        pointR = best.astype(np.int)

        # Draw matching
        outR = MyAlgorithm.drawPoint(self, outR, pointR.astype(np.int), None, (255, 0, 0))
        outR_last = MyAlgorithm.drawLastPoint(self, outR, pointR.astype(np.int), None)
        self.setRightImageFiltered(outR_last)
        self.setLeftImageFiltered(outL_last)
        return pointR, outR, outL


    def intersectionEcuation(self,vectorR,vectorL,OR,OL):
        '''
        Calculo de la interseccion de dos rectas en 3D y calculo del punto con menor distancia entre ellos
        '''
        ORL = OR-OL
        d1343 = ORL[0]*vectorL[0]+ORL[1]*vectorL[1]+ORL[2]*vectorL[2]
        d4321 = vectorL[0]*vectorR[0]+vectorL[1]*vectorR[1]+vectorL[2]*vectorR[2]
        d1321 = ORL[0]*vectorR[0]+ORL[1]*vectorR[1]+ORL[2]*vectorR[2]
        d4343 = vectorL[0]*vectorL[0]+vectorL[1]*vectorL[1]+vectorL[2]*vectorL[2]
        d2121 = vectorR[0]*vectorR[0]+vectorR[1]*vectorR[1]+vectorR[2]*vectorR[2]
        denom = d2121 * d4343 - d4321 * d4321
        numer = d1343 * d4321 - d1321 * d4343


        if (abs(denom) < 1e-6):
            return np.array([]),np.array([])
        else:
            mua = numer / denom
            mub = (d1343 + d4321 * (mua)) / d4343

            pa = OR+vectorR*mua
            pb = OL+vectorL*mub

            if abs(pa[0]) > 1e6 or  abs(pa[1]) > 1e6 or  abs(pa[2]) > 1e6:
                return np.array([]), np.array([])
            else:
                return pa,pb


    def execute(self):

        # OBTENCIÓN DE IMÁGENES
        imageLeft = self.sensor.getImageLeft()
        imageRight = self.sensor.getImageRight()
        self.width = imageRight.shape[1]
        self.height = imageRight.shape[0]
        self.time = time.clock()
        self.outEpipol = np.zeros((self.height,self.width*2,3),dtype=np.uint8)

        self.OR = self.camRightP.getCameraPosition()
        self.OL = self.camLeftP.getCameraPosition()
        self.hsvR = cv2.cvtColor(imageRight, cv2.COLOR_RGB2HSV)
        self.hsvL = cv2.cvtColor(imageLeft, cv2.COLOR_RGB2HSV)

        def plot_images(img1, img2, title):
            plt.figure(1, figsize=(18,18))
            plt.title(title)
            plt.subplot(121)
            plt.imshow(img1)
            plt.axis('off')
            plt.subplot(122)
            plt.imshow(img2)
            plt.axis('off')
            plt.show()


        write = False
        visor = True


        # KEYPOINTS
        print "Calculating canny"
        keypointsR, keypointsL,edgesR,edgesL = MyAlgorithm.findKeypoints(self,imageR=imageRight,imageL=imageLeft)
        outR = cv2.cvtColor(edgesR, cv2.COLOR_GRAY2RGB)
        outL = cv2.cvtColor(edgesL, cv2.COLOR_GRAY2RGB)

        print('1a. OutLR images')
        plot_images(outL, outR, 'outLR')
        print('1b. keypointsL', (keypointsL.shape), keypointsL.max(0), keypointsL[1,:])
        # ('1b. keypointsL', (3997, 3), array([135, 319,   1]), array([ 27, 219,   1]))

        print "Keypoints Right: {:d}\t Keypoints Left: {:d}".format(keypointsR.shape[0],keypointsL.shape[0])
        print "Done!"

        print "Calculating all vectors from left camera"
        # Lines
        vectorL = MyAlgorithm.getVectors(self,keypointsL,self.camLeftP.getCameraPosition())
        print('2a. keyPointL', keypointsL.shape, keypointsL[1, :])
        # ('2a. keyPointL', (5526, 3), array([ 15, 290,   1]))
        print('2b. vectorL', vectorL.shape, vectorL[1, :])
        # ('2b. vectorL', (5526, 4), array([ 277., -130.,  104.,    1.]))


        # Get M points from N vectors: NxM
        print "Calculating list of 3D points "
        lstPoints3d_L = MyAlgorithm.getPointsfromline(self,vectorL,self.camLeftP.getCameraPosition())
        print "Dimension",len(lstPoints3d_L),"x",len(lstPoints3d_L[0])

        print('3. lstPoints3d_L', len(lstPoints3d_L), (lstPoints3d_L[0]))
        # ('3. lstPoints3d_L', 5526, [array([ 2.77e+03, -1.16e+03,  1.04e+03,  1.00e+00]),
        #                             array([ 2.770e+04, -1.259e+04,  1.040e+04,  1.000e+00]),
        #                             array([ 2.77000e+06, -1.26989e+06,  1.04000e+06,  1.00000e+00])])


        # Bucle
        for idx,Points3d_L in enumerate(lstPoints3d_L):
            if idx%100==0 and idx>0:
                print idx,'/',len(lstPoints3d_L),"T: ", time.clock() - self.time

            # Linea epipolar
            m, n, begin, end = MyAlgorithm.getLineEpipolar(self,Points3d_L,cam="right")
            projected_R = MyAlgorithm.getPointsfromEpipolar(self,m,n,begin,end)

            print('4a. Points3d_L', (Points3d_L))
            # 4a. Points3d_L', [array([ 2.77e+03, -1.14e+03,  1.04e+03,  1.00e+00]),
            #                   array([ 2.770e+04, -1.239e+04,  1.040e+04,  1.000e+00]),
            #                   array([ 2.77000e+06, -1.24989e+06,  1.04000e+06,  1.00000e+00])])
            print('4b. m, n, begin, end', (m.shape), (n.shape), (begin.shape), (end.shape), m, n, begin, end)
            # ('4b. m, n, begin, end', (), (), (), (), 0, 15, 265, 286)

            print('5. projected_R', (projected_R.shape), projected_R[1, :], projected_R.max(0))
            # ('5. projected_R', (110, 2), array([ 13, 266]))

            # Eliminamos los repetidos
            projected_R_unique = np.array(list(set(tuple(p) for p in np.asarray(projected_R))))
            print('6. projected_R_unique', (projected_R_unique.shape), projected_R_unique[1, :],
                                                            projected_R_unique.max(0))
            # ('6. projected_R_unique', (110, 2), array([ 14, 282]))

            #pintamos los puntos proyectados
            outR_line = MyAlgorithm.drawPoint(self,outR,projected_R_unique,None,(255,255,0))
            self.setRightImageFiltered(outR_line)

            print('7. OutR vs line images')
            plot_images(outR, outR_line, 'outR-line')


            # Matching
            pointR,outR,outL = MyAlgorithm.matchingPoint(self,keypointsL[idx,:],
                                                         projected_R_unique,edgesR,outR,outL, size=(25,25))
            print('8. pointR, outR, outL', (pointR.shape), pointR)
            # ('8. pointR, outR, outL', (2,), array([ 27, 214]))

            if tuple(pointR) != tuple(np.zeros((3,))):

                # Triangulacion
                point3D, color = MyAlgorithm.triangulate(self,pointR,imageRight,vectorL,idx)
                print('9. point3D, color', point3D, (color))
                # ('9. point3D, color', array([ 6771.11111111, -2994.44444444,  2542.22222222]),
                #                      (0.7176470588235294, 0.7176470588235294, 0.6549019607843137))

                # Pintamos
                if point3D.size !=0:
                    if write:
                         MyAlgorithm.writeTxt(self,"pts3DColor",point3D,color,)

                    if visor:
                        self.sensor.drawPoint(point=point3D,color=color)

            break
        print "Tiempo Total: ",time.clock()-self.time
        if self.done:
            return

    def execute(self):
        imageLeft = self.sensor.getImageLeft()
        imageRight = self.sensor.getImageRight()

        if self.done:
            return
        '''
        Method
        1. Obtain points of interest
        2. Search for counterpart point on second image
        3. Triangulation
        4. Representation
        '''
        '''

        def get_descriptors(gray_image, feature_type):
            if feature_type == 'surf':
                feature_extractor = cv2.xfeatures2d.SURF_create()
            elif feature_type == 'sift':
                feature_extractor = cv2.xfeatures2d.SIFT_create()
            else:
                raise TypeError("Invalid feature type; should be either 'surf' or 'sift'")
            keypoints, descriptors = feature_extractor.detectAndCompute(gray_image, None)
            return keypoints, descriptors

        def convert_point(point):
            point = point.reshape((4, 1))
            H = (self.camLeftP.RT)
            T = np.eye(4)
            cam_pos = self.camLeftP.getCameraPosition()
            T[:3, 3] = cam_pos
            conv_point = np.dot(H, np.dot(T, point))
            conv_point = conv_point.T[:, :3].reshape((3,))
            return conv_point*10

        def get_colors_points(image,points):
            colors = []
            out_points = []
            for i in range(points.shape[0]):
                point = points[i,:].astype(np.int)
                color = image[point[1],point[0],:].astype(np.float32).tolist()
                pointIn = np.array([point[1],point[0],1])
                #print(pointIn)
                pointInOpt = self.camLeftP.graficToOptical(pointIn)
                #print(pointInOpt)
                point3d = self.camLeftP.backproject(pointInOpt)
                #print(point3d)
                colors.append((color[0]/255,color[1]/255,color[2]/255))
                #out_points.append(convert_point(point3d))
                out_points.append(point3d)
            return colors, out_points

        img_gray_left = cv2.cvtColor(imageLeft, cv2.COLOR_BGR2GRAY)
        img_gray_right = cv2.cvtColor(imageRight, cv2.COLOR_BGR2GRAY)

        img_left = cv2.Canny(imageLeft, 100, 200)
        img_right = cv2.Canny(imageRight, 100, 200)

        feature_type = 'surf' # 'surf', 'sift'

        kps_left, des_left = get_descriptors(img_left, feature_type)
        kps_right, des_right = get_descriptors(img_right, feature_type)

        # FLANN parameters
        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks=50)

        # Get the matches based on the descriptors
        flann = cv2.FlannBasedMatcher(index_params, search_params)
        matches = flann.knnMatch(des_left, des_right, k=2)

        pts_left_image = []
        pts_right_image = []

        # ratio test to retain only the good matches
        for i,(m,n) in enumerate(matches):
            if m.distance < 0.7*n.distance:
                pts_left_image.append(kps_left[m.queryIdx].pt)
                pts_right_image.append(kps_right[m.trainIdx].pt)

        pts_left_image = np.float32(pts_left_image)
        pts_right_image = np.float32(pts_right_image)

        F, mask = cv2.findFundamentalMat(pts_left_image, pts_right_image, cv2.FM_LMEDS)

        # Selecting only the inliers
        pts_left_image = pts_left_image[mask.ravel()==1]
        pts_right_image = pts_right_image[mask.ravel()==1]

        colors, points = get_colors_points(imageLeft, pts_left_image)

        # Set filtered images on GUI
        self.setLeftImageFiltered(img_left)
        self.setRightImageFiltered(img_right)

        print(len(points))

        for i in range(len(points)):
            print(points[i], colors[i])
            self.sensor.drawPoint(points[i], colors[i])

        '''

        imgL = imageLeft
        imgR = imageRight

        window_size = 3
        min_disp = 16
        num_disp = 112-min_disp
        stereo = cv2.StereoSGBM_create(minDisparity = min_disp,
            numDisparities = num_disp,
            blockSize = 16,
            P1 = 8*3*window_size**2,
            P2 = 32*3*window_size**2,
            disp12MaxDiff = 1,
            uniquenessRatio = 10,
            speckleWindowSize = 100,
            speckleRange = 32
        )

        disp = stereo.compute(imgL, imgR).astype(np.float32) / 16.0

        h, w = imgL.shape[:2]
        f = 0.8*w                          # guess for focal length
        Q = np.float32([[1, 0, 0, 0.5*w],
                        [0,-1, 0,  0.5*h], # turn points 180 deg around x-axis,
                        [0, 0, f,     0], # so that y-axis looks up
                        [0, 0, 0,      1]])
        points = cv2.reprojectImageTo3D(disp, Q)
        #colors = imgL
        colors = cv2.cvtColor(imgL, cv2.COLOR_BGR2RGB)
        mask = disp > disp.min()
        out_points = points[mask]
        out_colors = colors[mask]

        for i in range(len(out_points)):
            print(out_points[i], out_colors[i])
            self.sensor.drawPoint(out_points[i], out_colors[i])
