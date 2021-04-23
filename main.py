import cv2 as cv 
import numpy as np
from matplotlib import pyplot as plt
import time
from features import Features

left = cv.VideoCapture('./Data/left.mp4')
right = cv.VideoCapture('./Data/right.mp4')
stereo_video = cv.VideoCapture('./Data/dual_camera_compressed.mp4')
'''
try adding a check for fiducials, if fioducials just track those and use this to calibrate
if not fiducials run SLAM stuff 
TO DO: 
    A lot of clean up to do... But this serves as some proof that keypoints between previous frame and current frame track and match reasonably well-- This also isnt stereo slam rn-- its kind of just two slams-- need to look into whether this is a good approach or not
'''
extractor = Features()
while(stereo_video.isOpened()):
    ret,frame = stereo_video.read()
    frames_left,frames_right = extractor.processFrame(frame)
    left_feat,right_feat = extractor.findPoints()
    left_m,right_m = extractor.matchFeats()
    if left_feat[1] is not None:
        gray_left = cv.drawMatches(frames_left[0],left_feat[0]["kp"],frames_left[1],left_feat[1]["kp"],left_m[:10],None,flags =cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        gray_right = cv.drawMatches(fr:ames_right[0],right_feat[0]["kp"],frames_right[1],right_feat[1]["kp"],right_m[:10],None,flags =cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        output = cv.vconcat([gray_left,cv.flip(gray_right,-1)])
        output = cv.resize(output,(1280//2,1440//2))
        cv.imshow('out', output)
    if cv.waitKey(1) & 0xFF == ord('p'):
        while not cv.waitKey(1) & 0xFF == ord('u'):
            pass
        
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

left.release()
right.release()
cv.destroyAllWindows()
