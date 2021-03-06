import cv2 as cv
import numpy as np


class Features:
    def __init__(self):
        self.last_left = None
        self.last_right = None
        self.orb = cv.ORB_create()
        self.bf = cv.BFMatcher(cv.NORM_HAMMING)
        self.right_feats = None
        self.left_feats = None
        self.last_framel = 0
        self.last_framer = 0
        self.first_frame = True
    def processFrame(self,stereo_frame):
        #trimmed the frame towards the middle a little to save processing
        self.frame_left = stereo_frame[0:1080-200,0:1920//2]
        self.frame_right = stereo_frame[0:1080-200,1920//2:1920]
        #split the frame and convert to one channel
        self.gray_left = cv.cvtColor(self.frame_left,cv.COLOR_BGR2GRAY)
        self.gray_right = cv.cvtColor(self.frame_right,cv.COLOR_BGR2GRAY)
        return (self.gray_left,self.last_framel),(self.gray_right,self.last_framer)
    def findPoints(self):
        kp_left,des_left = self.orb.detectAndCompute(self.gray_left,None)
        kp_right,des_right = self.orb.detectAndCompute(self.gray_right,None)
        self.left_feats = {"kp" : kp_left, "des" : des_left}
        self.right_feats = {"kp" : kp_right, "des" : des_right}
        #extract and return the keypoints for both sides and also return previous kps
        return (self.left_feats,self.last_left),(self.right_feats,self.last_right)
    def matchFeats(self):
        matches_left =[]
        matches_right =[]
        #only check for matches if we have a previous frame to compare to
        if not self.first_frame:
            matches_left = self.bf.match(self.left_feats["des"],self.last_left["des"])
            matches_right = self.bf.match(self.right_feats["des"],self.last_right["des"])
        self.last_right = {"kp" : self.right_feats["kp"], "des" : self.right_feats["des"]}
        self.last_left = {"kp" : self.left_feats["kp"], "des" : self.left_feats["des"]}
        self.last_framer = self.gray_right
        self.last_framel = self.gray_left
        self.first_frame = False
        #sort by shortest distances (using this we can then filter out weird matches)
        matches_left = sorted(matches_left, key = lambda x:x.distance)
        matches_right = sorted(matches_right, key = lambda x:x.distance)
        return matches_left,matches_right
  
