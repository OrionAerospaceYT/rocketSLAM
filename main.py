import cv2 as cv 
import numpy as np
from matplotlib import pyplot as plt
import time
from features import Features

left = cv.VideoCapture('./Data/left.mp4')
right = cv.VideoCapture('./Data/right.mp4')
stereo_video = cv.VideoCapture('./Data/dual_camera_compressed.mp4')
match_thresh = 40
'''
try adding a check for fiducials, if fioducials just track those and use this to calibrate
if not fiducials run SLAM stuff 

TO DO:
    -- this isnt really stereo-SLAM more like two SLAMs on both cameras rn (need to look into if this is an okay approach)
    -- clean up main
    -- pretty happy with features.py rn
    -- look into VO and extracting data
    -- look into filtering -- Essential Matrix and other Camera calibration 
'''


def distanceCheck(v,matches,threshold):
    if (((v[1][0] -v[0][0])**2)+((v[1][1]-v[0][1])**2)) < threshold*threshold: #for speed can change the thresh^2 to be computed once and then passed
        matches.append(v)
    return matches

def vectorFlow(matches,feats, ret_vecs):
    for m in matches[:20]:
        p1 = m.trainIdx     
        p2 = m.queryIdx
        l1 = feats[0]["kp"][p2].pt
        l1 = int(l1[0]),int(l1[1])
        l2 = feats[1]["kp"][p1].pt
        l2 = int(l2[0]),int(l2[1])
        ret_vecs = distanceCheck((l1,l2),ret_vecs,match_thresh)
    return ret_vecs

def drawVectors(frame, ret_vecs):
    for v in ret_vecs:    
        frame = cv.line(frame,v[0],v[1],(255,255,255),6)
    return frame

def main():
    extractor = Features()
    ret_vecs_r = []
    ret_vecs_l = []
    while(stereo_video.isOpened()):
        ret,frame = stereo_video.read()
        frames_left,frames_right = extractor.processFrame(frame)
        fl = frames_left[0]
        fr = frames_right[0]
        left_feat,right_feat = extractor.findPoints()
        left_m,right_m = extractor.matchFeats()
        if left_feat[1] is not None:
            ret_vecs_r = vectorFlow(right_m,right_feat,ret_vecs_r)
            ret_vecs_l = vectorFlow(left_m, left_feat,ret_vecs_l)
            fl = drawVectors(fl,ret_vecs_l)
            fr = drawVectors(fr,ret_vecs_r)
            output = cv.vconcat([fl,cv.flip(fr,-1)])
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
if __name__ == "__main__":
    main()

'''
gray_left = cv.drawMatches(frames_left[0],left_feat[0]["kp"],frames_left[1],left_feat[1]["kp"],left_m[:10],None,flags = cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
gray_right = cv.drawMatches(frames_right[0],right_feat[0]["kp"],frames_right[1],right_feat[1]["kp"],right_m[:10],None,flags = cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
'''


