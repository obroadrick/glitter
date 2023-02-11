import numpy as np
import cv2, PIL, os
from cv2 import aruco
import pandas as pd
from dataclasses import dataclass
import sys


def getPoints(arr, loc):
    scores = [np.sqrt((pt[0] - loc[0])**2 + (pt[1] - loc[1])**2) for pt in arr]
    return arr[np.argmin(scores)], np.mean(scores)

def findMarkerCorners(pts):
    BL, TL, TR, BR = Marker(), Marker(), Marker(), Marker()
    _ids = ids.flatten()
    TRidx = np.where(_ids == 9)[0][0]
    TLidx = np.where(_ids == 0)[0][0]
    
    
    TL.BL = getPoints(pts[TLidx], [0, h/2])[0]
    TL.BR = getPoints(pts[TLidx], [w/2, h/2])[0]
    TL.TR = getPoints(pts[TLidx], [w/4, 0])[0]
    TL.TL = getPoints(pts[TLidx], [0, 0])[0]
    
    
    TR.BL, _ = getPoints(pts[TRidx], [w/2, h/2]) #we're assuming that the bottom left is closest to the center of image
    TR.BR = getPoints(pts[TRidx], [w, h/2])[0]
    TR.TR = getPoints(pts[TRidx], [w, 0])[0]
    TR.TL = getPoints(pts[TRidx], [3*w/4, 0])[0]
    
    #cull our array to exclude discovered corners 
    pts = np.delete(pts, TRidx, 0)
    delIdx = TLidx
    if TLidx > TRidx:
        delIdx = TLidx - 1
    pts = np.delete(pts, delIdx, 0)
    #makes the next search easier

    scoresBL = getPoints(pts[0], [0, h]), getPoints(pts[1], [0, h])
    scoresBR = getPoints(pts[0], [w/2, h]), getPoints(pts[1], [w/2, h])
    BLidx = 1
    if scoresBL[0][1] < scoresBL[1][1]:
        BLidx = 0
    BRidx = BLidx - 1
    BL.BL, BR.BL = scoresBL[BLidx][0], scoresBR[BRidx][0]
    
    BL.BR = getPoints(pts[BLidx], [w/2, h])[0]
    BL.TR = getPoints(pts[BLidx], [w/4, 3 * h/4])[0]
    BL.TL = getPoints(pts[BLidx], [0, 3*h/4])[0]
    
    BR.BR = getPoints(pts[BRidx], [w, h])[0]
    BR.TR = getPoints(pts[BRidx], [w, 3*h/4])[0]
    BR.TL = getPoints(pts[BRidx], [3*w/4, h/2])[0]

    return([BL, TL, TR, BR])
    

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

nx = 8
ny = 6
board = aruco.CharucoBoard_create(3, 3, 1, 0.8, aruco_dict)

img_name = str(sys.argv[1])
frameCol = cv2.imread(img_name)
frame = cv2.cvtColor(frameCol, cv2.COLOR_BGR2GRAY)
para = cv2.aruco.DetectorParameters_create()
para.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
bboxs, ids, rejected = cv2.aruco.detectMarkers(frame, aruco_dict, parameters = para)
h, w = frameCol.shape[:2]

refPts = []
@dataclass
class Marker:
    BL: list[float] = list[0.0, 0.0]
    TL: list[float] = list[0.0, 0.0]
    TR: list[float] = list[0.0, 0.0]
    BR: list[float] = list[0.0, 0.0]

if  len(bboxs)==4:  
    for Corner, id in zip(bboxs, ids):               
        corners = Corner.reshape((4, 2))
        (topLeft, topRight, bottomRight, bottomLeft) = corners
        topRight = (int(topRight[0]), int(topRight[1]))
        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
        topLeft = (int(topLeft[0]), int(topLeft[1]))

        # draw lines around the marker and display the marker id   
        corner = np.squeeze(Corner)
        refPts.append(corner)

else:
    raise ValueError('Unable to find all 4 markers')
    
    
fidMarkerPos = findMarkerCorners(refPts)

df = pd.DataFrame(columns = [" ", "Bottom Left Marker", "Top Left Marker", "Top Right Marker", "Bottom Right Marker"])
df[" "] = ["BL Point", "TL Point", "TR Point", "BR Point"]
for (idx, colname) in enumerate(df):
    if idx == 0:
        continue
    df[colname] = [fidMarkerPos[idx-1].BL, fidMarkerPos[idx-1].TL, fidMarkerPos[idx-1].TR, fidMarkerPos[idx-1].BR]

mat_arr = df.drop(" ", 1).to_numpy()

print(mat_arr)