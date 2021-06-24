import cv2.aruco as aruco
import cv2 as cv
import numpy as np
#thanks to https://github.com/tizianofiorenzani/how_do_drones_work/blob/master/scripts/06_precise_landing.py

#WEBCAM TESTIN
cap = cv.VideoCapture(0)

#all of this will need to get wrapped with serial communications to the teensy. 
dictionary = cv.aruco.Dictionary_get(cv.aruco.DIOCT_6X6_250) #6x6 at 250 mm
parameters = cv.aruco.DetectorParameters_create()
MARKER_IDS = [1,2,3,4] #aruco markers from https://chev.me/arucogen/
MARKER_SIZE = 10 #cm

while True:
        ret,frame = cap.read()
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        parameters.adaptiveThreshConstat = 10
        corners,ids,rejects = aruco.detectMarkers(frame,dictionary,parameters=parameters,cameraMatrix=cam_mat,distCoeff=dist_mat)
        angle_x, angle_y    = marker_position_to_angle(x_cm, y_cm, z_cm)
        if np.all(ids is not None):
            rvec,tvec,markerPoints = aruco.estimatePoseSingleMarkers(corners[i],0.02,mtx,dist) # where mtx is camera matrix and dist is distortion coeffs
            strg = '' 
            for i in range(0,ids.size):
                #https://aliyasineser.medium.com/aruco-marker-tracking-with-opencv-8cb844c26628
                aruco.drawAxis(frame,mtx,dist,rvec[i],tvec[i],0.1)
                strg += str(ids[i][0]+','
                aruco.drawDetectMarkers(frame,corners)
            cv.putText(frame, "Id: " + strg, (0,64),cv.FONT_HERSHEY_SIMPLEX,1,(0,255,0),2,cv.LINE_AA)
        else:
            cv.putText(frame,"NO ID",(0,64),cv.FONT_HERSHEY_SIMPLEX,1,(0,255,0),2,cv.LINE_AA)
            cv.imshow('cap',frame)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
cap.release()
cv.destroyALLWindows()


def marker_position_to_angle(x, y, z):
    
    angle_x = math.atan2(x,z)
    angle_y = math.atan2(y,z)
    
    return (angle_x, angle_y)
