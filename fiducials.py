import numpy as np 
import cv2 as cv

orb = cv2.ORB_create()
#matcher = cv.BFMatcher(cv.NORM_HAMMING2
#Probably a better way to import fiducials for now will hardcode it
#t = top, b = bottom, r = right, l = left
fiducial_tl = cv.imread('./fiducials/fiducial_tl.png')
fiducial_tr = cv.imread('./fiducials/fiducial_trl.png')
fiducial_bl = cv.imread('./fiducials/fiducial_bll.png')
fiducial_br = cv.imread('./fiducials/fiducial_brl.png')
fiducials = [(fiducial_tl, "tl",(fiducial_tr, "tr"),(fiducial_bl, "bl"),(fiducial_br, "br")]
#since each fiducial will be the same resolution etc we can pre compute these intrinsic values here and just add them to the dict for each fid
fiducial_length = .4 #m
fiduciadd_width = .34 #m
#Extract and store information about fiducials and then use this as track. 
def fiducialPreCompute(fiducials):
    for fid in fiducials:
        kps,des = orb.detectAndCompute(fid,None)
        fiducial = {"name" : fid[1], "kps" : kps, "des" : des, pixel_width : "pixel_width", pixel_lenqth : "pixel_length", matches : "match_count"}

