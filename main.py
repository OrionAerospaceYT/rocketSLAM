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
    -- add an array to store all previous "good" matches so that we can draw continuos lines and get better idea of the flow
    -- look into filtering -- Essential Matrix and other Camera calibration 
'''


def distanceCheck(v,matches,threshold):
    if (((v[1][0] -v[0][0])**2)+((v[1][1]-v[0][1])**2)) < threshold*threshold: #for speed can change the thresh^2 to be computed once and then passed
        matches.append(v)
    return matches

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
            for m in left_m[:20]:
                p1 = m.trainIdx     
                p2 = m.queryIdx
                l1 = left_feat[0]["kp"][p2].pt
                l1 = int(l1[0]),int(l1[1])
                l2 = left_feat[1]["kp"][p1].pt
                l2 = int(l2[0]),int(l2[1])
                ret_vecs_l = distanceCheck((l1,l2),ret_vecs_l,match_thresh)
            for v in ret_vecs_l:    
                fl = cv.line(frames_left[0],v[0],v[1],(0,0,0),6)
            for m in right_m[:20]:
                p1 = m.trainIdx
                p2 = m.queryIdx
                l1 = right_feat[0]["kp"][p2].pt
                l1 = int(l1[0]),int(l1[1])
                l2 = right_feat[1]["kp"][p1].pt
                l2 = int(l2[0]),int(l2[1])
                ret_vecs_r = distanceCheck((l1,l2),ret_vecs_r,match_thresh)
            for v in ret_vecs_r:
                fr = cv.line(fr,v[0],v[1],(0,0,0),6)
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


