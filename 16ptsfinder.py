import numpy as np
import cv2, PIL, os
from cv2 import aruco
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd
from dataclasses import dataclass

@dataclass
class Marker:
    BL: list[float] = list[0.0, 0.0]
    TL: list[float] = list[0.0, 0.0]
    TR: list[float] = list[0.0, 0.0]
    BR: list[float] = list[0.0, 0.0]

def getPoints(arr, loc):
    scores = [np.sqrt((pt[0] - loc[0])**2 + (pt[1] - loc[1])**2) for pt in arr]
    return arr[np.argmin(scores)], np.mean(scores)

def findMarkerCorners(pts):
    BL, TL, TR, BR = Marker(), Marker(), Marker(), Marker()
    _ids = ids.flatten()
    TRidx = np.where(_ids == 2)[0][0]
    TLidx = np.where(_ids == 1)[0][0]
    
    
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
    
    cv2.circle(frameCol, (int(BL.BL[0]), int(BL.BL[1])), 10, (255, 0, 0), 9)
    cv2.circle(frameCol, (int(BL.TL[0]), int(BL.TL[1])), 10, (255, 0, 0), 9)
    cv2.circle(frameCol, (int(BL.TR[0]), int(BL.TR[1])), 10, (255, 0, 0), 9)
    cv2.circle(frameCol, (int(BL.BR[0]), int(BL.BR[1])), 10, (255, 0, 0), 9)
    
    cv2.circle(frameCol, (int(TR.BL[0]), int(TR.BL[1])), 10, (0, 255, 0), 9)
    cv2.circle(frameCol, (int(TR.TL[0]), int(TR.TL[1])), 10, (0, 255, 0), 9)
    cv2.circle(frameCol, (int(TR.TR[0]), int(TR.TR[1])), 10, (0, 255, 0), 9)
    cv2.circle(frameCol, (int(TR.BR[0]), int(TR.BR[1])), 10, (0, 255, 0), 9)
    
    cv2.circle(frameCol, (int(BR.BL[0]), int(BR.BL[1])), 10, (0, 0, 255), 9)
    cv2.circle(frameCol, (int(BR.TL[0]), int(BR.TL[1])), 10, (0, 0, 255), 9)
    cv2.circle(frameCol, (int(BR.TR[0]), int(BR.TR[1])), 10, (0, 0, 255), 9)
    cv2.circle(frameCol, (int(BR.BR[0]), int(BR.BR[1])), 10, (0, 0, 255), 9)
    
    cv2.circle(frameCol, (int(TL.BL[0]), int(TL.BL[1])), 10, (255, 0, 255), 9)
    cv2.circle(frameCol, (int(TL.TL[0]), int(TL.TL[1])), 10, (255, 0, 255), 9)
    cv2.circle(frameCol, (int(TL.TR[0]), int(TL.TR[1])), 10, (255, 0, 255), 9)
    cv2.circle(frameCol, (int(TL.BR[0]), int(TL.BR[1])), 10, (255, 0, 255), 9)
    
    #cv2.imshow("show", frameCol)
    #cv2.waitKey()  
    #cv2.destroyAllWindows()
    #plt.imshow(frameCol)
    cv2.imwrite("corners.jpg", frameCol)
    #return(Marker(BL, TL, TR, BR))
    #print(scoresBR)
    #print(scoresBL)
    
    return([BL, TL, TR, BR])
    
 
 

import sys

aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
para = cv2.aruco.DetectorParameters_create()
para.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
para.adaptiveThreshWinSizeMax = 200

#frameName = input("Enter relative image directory. Use forward slashes when referencing child directories: ")
frameName = sys.argv[1]
saveName = sys.argv[2]
#print(frameName)
#print(saveName)

frameCol = cv2.imread(frameName)
frame = cv2.cvtColor(frameCol, cv2.COLOR_BGR2GRAY)
#plt.imshow(frame)

bboxs, ids, rejected = cv2.aruco.detectMarkers(frameCol, aruco_dict, parameters = para)
refPts = []
h = 0
w = 0
if  len(bboxs)!=0:  
    for Corner, id in zip(bboxs, ids):               
        corners = Corner.reshape((4, 2))
        (topLeft, topRight, bottomRight, bottomLeft) = corners
        topRight = (int(topRight[0]), int(topRight[1]))
        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
        topLeft = (int(topLeft[0]), int(topLeft[1]))

        # draw lines around the marker and display the marker id
        cv2.line(frameCol, topLeft, topRight, (0, 255, 0), 7)
        cv2.line(frameCol, topRight, bottomRight, (0, 255, 0), 7)
        cv2.line(frameCol, bottomRight, bottomLeft, (0, 255, 0), 7)
        cv2.line(frameCol, bottomLeft, topLeft, (0, 255, 0), 7)                    
        cv2.putText(frame, str(id),(topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,2.5, (255, 0, 0), 2)
        corner = np.squeeze(Corner)
        refPts.append(corner)
        imS = cv2.resize(frameCol, (960, 540))   
        #cv2.imshow("show", imS)
        #cv2.waitKey()  
        #cv2.destroyAllWindows()
        #print(str(id))
    imS = cv2.resize(frameCol, (960, 540))   
    #cv2.imshow("show", imS)
    #cv2.waitKey()  
    #cv2.destroyAllWindows()

    h, w = frameCol.shape[:2]

    fidMarkerPos = findMarkerCorners(refPts)

    df = pd.DataFrame(columns = [" ", "Bottom Left Marker", "Top Left Marker", "Top Right Marker", "Bottom Right Marker"])
    df[" "] = ["BL Point", "TL Point", "TR Point", "BR Point"]
    for (idx, colname) in enumerate(df):
        if idx == 0:
            continue
        df[colname] = [fidMarkerPos[idx-1].BL, fidMarkerPos[idx-1].TL, fidMarkerPos[idx-1].TR, fidMarkerPos[idx-1].BR]

    mat_arr = df.drop(" ", 1).to_numpy()
    np.shape(mat_arr)

    import scipy.io
    scipy.io.savemat(saveName, mdict={'arr': mat_arr})

    print('Success - found ArUco Markers!')
    
    exit()
else:
    print('Markers not detected...')
