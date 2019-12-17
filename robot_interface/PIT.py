
from robolink.robolink import *    # API to communicate with RoboDK
from robodk.robodk import *      # robodk robotics toolbox
import os
import time
import re
import numpy as np

# Any interaction with RoboDK must be done through RDK:
RDK = Robolink()
robot = RDK.ItemUserPick('Select a robot', ITEM_TYPE_ROBOT)
filePoints = open('C:\\Users\\etyur\\Documents\\ROBOTICS_AAU\\5_SEMESTER\\P_5\\RoboDK_P5\\KUKA_KR60_3\\Traject_Flange_1.txt','r')
tempPointArray = filePoints.read()
tempPointArray = re.split(" |\n|,", tempPointArray)
curve_tr = np.zeros((int(len(tempPointArray)/6),6), dtype=float)
for i in range(0, int(len(tempPointArray) / 6)):
    curve_tr[i][0] = float(tempPointArray[i * 6])
    curve_tr[i][1] = float(tempPointArray[i * 6 + 1])
    curve_tr[i][2] = float(tempPointArray[i * 6 + 2])
    curve_tr[i][3] = float(tempPointArray[i * 6 + 3])
    curve_tr[i][4] = float(tempPointArray[i * 6 + 4])*-1
    curve_tr[i][5] = float(tempPointArray[i * 6 + 5])*-1
curve_tr = curve_tr.tolist()
pointsCurve = RDK.AddCurve(curve_tr)#,PROJECTION_NONE) #, temp_target, True, PROJECTION_NONE)
pointsCurve.setName('CurvePoints')# % NUM_POINTS)
path_settings = RDK.AddMachiningProject("AutoPointFollow settings")
progPoints, status = path_settings.setMachiningParameters(part=pointsCurve)

# At this point, we may have to manually adjust the tool object or the reference frame

# Run the create program if success
progPoints.RunProgram()
#robot.setJoints([0,-90,90,0,0,0])

# Done
quit()
time.sleep(5)
RDK.CloseStation()
RDK.CloseRoboDK()





#print(curve_tr)
#pointsPoints = RDK.AddPoints(curve_tr) #, temp_target,True,PROJECTION_ALONG_NORMAL_RECALC )





#visible_curve =

#progPoints = RDK.AddProgram('Add_Follow_Points')
#progPoints.ShowInstructions(True)











#time.sleep(2)
#RDK.CloseStation()
#RDK.CloseRoboDK()



