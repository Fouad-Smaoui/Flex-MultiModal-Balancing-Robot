# -*- coding: utf-8 -*-
"""
Created on Fri Jan 28 17:00:40 2022

@author: matri
"""

import numpy as np
import cv2.aruco as aruco
import cv2

#-----Camera -------------------------------------------------------------------------------------------------

frame = cv2.VideoCapture(0)    # Instanciantion de la camera
width = 512                    # resolution largeur 
height = 304                   # resolution hauteur 

# Set the camera size as the one it was calibrated with
frame.set(3, width)        # width resolution 
frame.set(4, height)       # height resolution

# Load intrinsic camera parameter 
calib_path = "Calibration Camera\intrinsicParam"
matrix =  np.loadtxt(calib_path+'\cameraMatrix.txt', delimiter=',')
distortion = np.loadtxt(calib_path+'\cameraDistortion.txt', delimiter=',')

#-----Traking color-------------------------------------------------------------------------------------------

# Red color range       
hsv_lower = np.array([161, 155, 84])       
hsv_upper = np.array([179, 255, 255])

    
def displayColor(img, mask, cx, cy, rayon):
    cv2.circle(img, (int(cx), int(cy)), int(rayon), (0,255,255), 2)
    cv2.imshow("mask",mask)
    cv2.imshow("cv_immage", img)
    
def trackColor(img, show = False) : 
    # Change color represtation from BGR to HSV   
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsv = cv2.blur(hsv, (5,5))    # Foutage de l'image, kernel_size = (5,5) 
    
    # Threshold the HSV image to get only red colors  (filter unwanted color) 
    mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
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
        cx = width/2
        cy = height/2
        rayon = 0
    
    if show == True :
        displayColor(img, mask, cx, cy, rayon)
        
    return cx
    
    
#-----Aruco Markers-------------------------------------------------------------------------------------------------

markerID = 9
markersSize = 22
totalMarkers = 50
key = aruco.DICT_4X4_50 
arucoDict = aruco.Dictionary_get(key)                               # Déclaration du dictionaire 
arucoParam = aruco.DetectorParameters_create()              

def displayAruco(img, corners, px, py, d):
    #-- Draw the detected marker and put a reference frame over it
    aruco.drawDetectedMarkers(img, corners)
    #aruco.drawAxis(img, self.cam.matrix, self.cam.distortion, rvec, tvec, 10)

    #-- Print the tag position in camera frame
    str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f"%(px, py, d)
    cv2.putText(img, str_position, (0, 15), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 1, cv2.LINE_AA)
    #--- Display the frame
    cv2.imshow('frame', img)
       
    
def trackAruco(img,  show = False) : 
     #-- Convert in gray scale
    gray    = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red
    
    #-- Find all the aruco markers in the image
    corners, ids, _ = aruco.detectMarkers(image=gray, dictionary=arucoDict, parameters=arucoParam,
                                             cameraMatrix=matrix, distCoeff=distortion)
    
    
    if ids is not None and ids[0] == markerID:
    
        #-- ret = [rvec, tvec, ?]
        #-- array of rotation and position of each marker in camera frame
        #-- rvec = [[rvec_1], [rvec_2]]    attitude of the marker respect to camera frame
        #-- tvec = [[tvec_1], [tvec_2]]    position of the marker in camera frame
        ret = aruco.estimatePoseSingleMarkers(corners, markersSize, matrix, distortion)
        
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
        displayAruco(img, corners, px, py, d)
      
    return d, px 

    
    
if __name__ == '__main__':

   
    while True:
        # Read frame
        _, img = frame.read()
        x_px = trackColor(img, show = False)
        d, px = trackAruco(img, show = True)
        
    
        key = cv2.waitKey(1)
        if key == 27:  # ESC pressed
            break

frame.release()
cv2.destroyAllWindows() 