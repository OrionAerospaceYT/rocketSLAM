import cv2.aruco
import cv2 as cv
import numpy as np
#thanks to https://github.com/tizianofiorenzani/how_do_drones_work/blob/master/scripts/06_precise_landing.py

#all of this will need to get wrapped with serial communications to the teensy. 
dictionary = cv.aruco.Dictionary_get(cv.aruco.DIOCT_6X6_250) #6x6 at 250 mm
parameters = cv.aruco.DetectorParameters_create()
MARKER_IDS = [1,2,3,4] #aruco markers from https://chev.me/arucogen/
MARKER_SIZE = 10 #cm

while True:
	marker_found,x_cm, y_cm, z_cm =  #note that x and y and z will be camera relative
	if marker_found:
		corners,ids,rejects = aruco.detectMarkers(frame,dictionary,parameters=parameters,cameraMatrix=cam_mat,distCoeff=dist_mat)
		angle_x, angle_y    = marker_position_to_angle(x_cm, y_cm, z_cm)

if np.all(ids is not None):
	#there will always be 4 markers
	for i in range(0,3):
		#https://aliyasineser.medium.com/aruco-marker-tracking-with-opencv-8cb844c26628
		rvec,tvec,markerPoints = aruco.estimatePoseSingleMarkers(corners[i],0.02,)

def marker_position_to_angle(x, y, z):
    
    angle_x = math.atan2(x,z)
    angle_y = math.atan2(y,z)
    
    return (angle_x, angle_y)
