# -*- coding: utf-8 -*-

"""
Ce fichier gère la patie vision robotique et traitement d'image du rover. Equipé d'une camera
pré-calibré grace au spript cameraCalibration.py, les méthode de tracking on pour rôle de determiner 
la position et l'oriantation d'un object (individu) en mouvement. Pour cela trois type de detection 
sont possible : 
    -Tracking de couleur 
    -Tracking Aruco (QR code)
    -Estimation de la posture d'une personne par un reseau de neorone pré-entrainé
"""

import numpy as np
import cv2.aruco as aruco
import cv2

class Camera(object) :
    def __init__(self,  calib_path = "Calibration Camera\intrinsicParam", cam_nb = 0,  width = 512, height = 304, rate = 30) :
        self.frame = cv2.VideoCapture(cam_nb)   # Instanciantion de la camera
        self.width = width                      # resolution largeur 
        self.height = height                    # resolution hauteur 
        self.rate = rate                        # frequence d'echantillonage en Hz

        # Set the camera size as the one it was calibrated with
        self.frame.set(3, self.width)        # width resolution 
        self.frame.set(4, self.height)       # height resolution
        self.frame.set(5, self.rate)         # frame rate 
        
        # Load intrinsic camera parameter 
        self.matrix =  np.loadtxt(calib_path+'\cameraMatrix.txt', delimiter=',')
        self.distortion = np.loadtxt(calib_path+'\cameraDistortion.txt', delimiter=',')
        
        
        
class Color(object):
    def __init__(self, name = "red", hsv_lower = np.array([161, 155, 84]), hsv_upper = np.array([179, 255, 255])):
        self.name = name
        self.hsv_lower = hsv_lower
        self.hsv_upper = hsv_upper
        
    def detectColorRange(self) : 
        pass

class TrakingColor(object) : 
    def __init__(self, color = Color(), cam = Camera()) :
        self.color = color   #red by default
        self.cam = cam
    
    def display(self, img, mask, cx, cy, rayon):
        cv2.circle(img, (int(cx), int(cy)), int(rayon), (0,255,255), 2)
        cv2.imshow("mask",mask)
        cv2.imshow("cv_immage", img)
        
    def detect(self, img, show = False) : 
        # Change color represtation from BGR to HSV   
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hsv = cv2.blur(hsv, (5,5))    # Foutage de l'image, kernel_size = (5,5) 
        
        # Threshold the HSV image to get only red colors  (filter unwanted color) 
        mask = cv2.inRange(hsv, self.color.hsv_lower, self.color.hsv_upper)
        mask = cv2.erode(mask, None, iterations = 4)      # Suprime les petites formes 4 fois (convolution) 
        mask = cv2.dilate(mask, None, iterations = 4)     # Augmente les grosses forme (convolution inverse)
        
        # Find contours
        forms = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        
        # Si la méthode findContours trouve au moins une forme 
        if len(forms) > 0 :
            
            # On prend la plus grosse forme
            c = max(forms, key = cv2.contourArea)
            
            # On obtient le centre de gravité géometrique et le rayon (ordre de grandeur de l'objet) en pixel
            ((cx,cy),rayon) = cv2.minEnclosingCircle(c)
             
        else :
            cx = self.cam.width/2
            cy = self.cam.height/2
            rayon = 0
        
        if show == True :
            self.display(img, mask, cx, cy, rayon)
        
        return cx
    
    
class TrakingAruco(object) :
    def __init__(self, camera = Camera(), markersSize = 4, totalMarkers = 50, markerID = 20) :
        self.cam = camera 
        self.markerID = markerID
        self.markersSize = markersSize
        key = getattr(aruco, f'DICT_{markersSize}X{markersSize}_{totalMarkers}') # key = aruco.DICT_6X6_250 
        self.arucoDict = aruco.Dictionary_get(key)                               # Déclaration du dictionaire 
        self.arucoParam = aruco.DetectorParameters_create()              

    def display(self, img, corners, px, py, d):
        #-- Draw the detected marker and put a reference frame over it
        aruco.drawDetectedMarkers(img, corners)
        #aruco.drawAxis(img, self.cam.matrix, self.cam.distortion, rvec, tvec, 10)
    
        #-- Print the tag position in camera frame
        str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f"%(px, py, d)
        cv2.putText(img, str_position, (0, 15), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 1, cv2.LINE_AA)
        #--- Display the frame
        cv2.imshow('frame', img)
       
    
    def detect(self, img,  show = False) : 
         #-- Convert in gray scale
        gray    = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red
    
        #-- Find all the aruco markers in the image
        corners, ids, _ = aruco.detectMarkers(image=gray, dictionary=self.arucoDict, parameters=self.arucoParam,
                                                     cameraMatrix=self.cam.matrix, distCoeff=self.cam.distortion)
        
        print(corners)
    
        if ids is not None and ids[0] == self.markerID:
            
            #-- ret = [rvec, tvec, ?]
            #-- array of rotation and position of each marker in camera frame
            #-- rvec = [[rvec_1], [rvec_2]]    attitude of the marker respect to camera frame
            #-- tvec = [[tvec_1], [tvec_2]]    position of the marker in camera frame
            ret = aruco.estimatePoseSingleMarkers(corners, self.markersSize, self.cam.matrix, self.cam.distortion)
    
            #-- Unpack the output, get only the first
            tvec = ret[1][0,0,:]
            
    
            px = np.mean(corners[0][:,0])
            py = np.mean(corners[0][:,1])
            d = tvec[2]           
            
        else : 
            
            px = 0
            py = 0
            d = 0
            
        if show == True :
            self.display(img, corners, px, py, d)
              
        return d, px 

    
    
if __name__ == '__main__':

    cam = Camera()
    trackColor = TrakingColor()
    trackAruco = TrakingAruco(markersSize = 30)
    
    while True:
        # Read frame
        _, img = cam.frame.read()
        x_px = trackColor.detect(img, show = False)
        d, px = trackAruco.detect(img, show = True)
        
    
        key = cv2.waitKey(1)
        if key == 27:  # ESC pressed
            break

cam.frame.release()
cv2.destroyAllWindows() 
       

        
        