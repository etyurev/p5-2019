# This macro shows an example to run a program on the robot from the Python API (online programming)
#
# Important: By default, right clicking a program on the RoboDK API and selecting "Run On Robot" has the same effect as running this example.
# Use the Example_OnlineProgramming.py instead if the program is run from the RoboDK Station Tree
#
# This example forces the connection to the robot by using:
# robot.Connect()
# and
# RDK.setRunMode(RUNMODE_RUN_ROBOT)

# In this script, if the variable RUN_ON_ROBOT is set to True, an attempt will be made to connect to the robot
# Alternatively, if the RUN_ON_ROBOT variable is set to False, the program will be simulated (offline programming)
#
# More information about the RoboDK API here:
# https://robodk.com/doc/en/RoboDK-API.html
import sys
import numpy as np
import re
sys.path.append("C:\\Users\\etyur\\Documents\\ROBOTICS_AAU\\5_SEMESTER\\P_5\\RoboDK_P5\\KUKA_KR60_3")
import os
from robolink.robolink import *  # API to communicate with RoboDK
from robodk.robodk import *  # robodk robotics toolbox
import time
import mailbox
from TranformClass.transformation import transformation

####### SETUP of THE  STation ######
RDK = Robolink()
print(RDK._is_connected())
os.system('C:\\Users\\etyur\\Documents\\ROBOTICS_AAU\\5_SEMESTER\\P_5\\RoboDK_P5\\KUKA_KR60_3\\KUKAKR60_301119.rdk')
robot = RDK.ItemUserPick('Select a robot', ITEM_TYPE_ROBOT)
if not robot.Valid():
    raise Exception('No robot selected or available')
scannerPose = [167.8,-52,55,-180,-90,180] #[167.8,-52,57,-180,-90,180]
toolS = KUKA_2_Pose(scannerPose)
robot.AddTool(toolS,'Scanner')
#pitPose = [-175.6,344.02,54.15,180,0,-90]#[-176.04,243.35,54.76,180,0,-90] #[-176.5,245,57,180,0,-90]
pitPose = [-175.6,330,54.15,180,0,-90] #[-175.6,330.02,54.15,180,0,-90]
toolP = KUKA_2_Pose(pitPose)
robot.AddTool(toolP,'PIT')
#pitActPose = [-176.5,232,57,180,0,-90]
#toolPA = KUKA_2_Pose(pitActPose)
#robot.AddTool(toolPA,'PIT_ACTIVATION')
tool1 = RDK.Item('Scanner')
tool2 = RDK.Item('PIT')
#tool3 = RDK.Item('PIT_ACTIVATION')
target1 = RDK.AddTarget('Start Scan Target')
target1.setAsCartesianTarget()
target1.setPose(KUKA_2_Pose([1492.74,-303.42,64.312,-91.87,0.0,-0.02])) # #[1492.74,-303.42,494.43,-91.87,0.0,-0.02] #[1344.917,-298.897,64.312,-90.770,0.015,0.134]
target2 = RDK.AddTarget('End Scan Target')
target2.setAsCartesianTarget()
target2.setPose(KUKA_2_Pose([1561.513,-303.42,64.454,-91.87,0.0,-0.02])) # [1561.513,-303.42,494.43,-91.87,0.0,-0.02]  [1413.690,-300.074,64.454,-91.87,0.0,-0.02]
target3 = RDK.AddTarget('Ready to Work Target')
target3.setAsCartesianTarget()
target3.setPose(KUKA_2_Pose([1484.995,-303.42,460.803,-91.87,0.0,-0.02])) #  [1484.995,-303.42,543.43,-91.87,0.0,-0.02] #[1404.995,-300.074,110.803,-91.87,0.0,-0.02]
path_folder = "C:\\Users\\etyur\\Documents\\ROBOTICS_AAU\\5_SEMESTER\\P_5\\RoboDK_P5\\KUKA_KR60_3\\TranformClass\\"
scanProg = RDK.AddProgram('ScanProgramm', ITEM_TYPE_PROGRAM)
scanProg.ShowInstructions(False)
base_reference_pose = KUKA_2_Pose([0, 0, 0, 0, 0, 0])
scanProg.setPoseFrame(base_reference_pose)
scanProg.setTool(tool1)
scanProg.setSpeed(10)
robot.setPoseTool(tool1)
scanProg.MoveL(target1)
scanProg.setDO('1',1)
scanProg.setDO('1',0)
scanProg.MoveL(target2)
scanProg.MoveL(target3)

#scanProg.RunCode()

while scanProg.Busy() == 1:
    pause(0.1)
print("Scan Programs done")

path_folder = "C:\\Users\\etyur\\Documents\\ROBOTICS_AAU\\5_SEMESTER\\P_5\\RoboDK_P5\\KUKA_KR60_3\\TranformClass\\"
name_kuka_ipo = "WINDOWS-SVF2O6T - R2-D2_19gr563_KRCIpo.r64"
name_kuka_io = "WINDOWS-SVF2O6T - R2-D2_19gr563_KRCIO.r64"
path_kuka_ipo = path_folder + name_kuka_ipo
path_kuka_io = path_folder + name_kuka_io
    #only used for debug.
name_trajectory = "28-11-3-weld-trajectory-0.txt"  # -1.txt right side / -0 left
    #create class and call these 3 functions
trans=transformation()
trans.reading_from_file(path_kuka_ipo,path_kuka_io)
trans.find_start_scan()
trans.find_scan_coordinates()
    #reading trajectories manually only for debug
f=open(path_folder+name_trajectory)
temp=f.read()
temp=re.split(" |\n|;",temp)
local_traject_poses = np.zeros((int(len(temp)/6),3))
for i in range(0,int(len(temp)/6)):
    local_traject_poses[i][0]=-float(temp[i*6])
    local_traject_poses[i][1]=float(temp[i*6+1])
    local_traject_poses[i][2]=-float(temp[i*6+2])
f.close()
print (len(local_traject_poses))
    #returns the curve for robodk as numpy
curve_roboDK = trans.calc_pose_pit_robodk(local_traject_poses,30) # -20 the right side/ 20 left
listCurve = curve_roboDK.tolist()
object_curve = RDK.AddCurve(listCurve)
object_curve.setName('Curve Trajectory for PIT')
robot.setTool(tool2)
path_settings = RDK.AddMachiningProject("CurveFollow settings")
poseStart = [0, 0, 0, 180, 0, 0]
poseStart = KUKA_2_Pose(poseStart)
path_settings.setPose(poseStart)
operationSpeed = 10
path_settings.setSpeed(operationSpeed)
#path_settings.setParam("JoinCurveTol", "0.01")
#path_settings.setValue("JoinCurveTol", "0.01")

#item=RDK.Item("JoinCurveTol")
#item.setParam("JoinCurveTol", "0.01")


prog, status = path_settings.setMillingParameters(part=object_curve,params='NormalApproach=0') # two return values if they are stay
#prog, status = path_settings.setMillingParameters(part=object_curve,params='NormalRetract=78')
prog.ShowInstructions(False)



# prog.InstructionCount()
# prog.InstructionList()

ins_id = 0
ins_count = prog.InstructionCount()
print(ins_count)
active = False
while ins_id < ins_count:
    
    ins_nom, ins_type, move_type, isjointtarget, pose, joints = prog.Instruction(ins_id)
    # print(isjointtarget)
    # print(pose)
    if ins_type == INS_TYPE_CHANGESPEED and ins_id < 10:
        # Changes all speeds to 10 mm/s
        prog.InstructionSelect(ins_id)
        prog.setSpeed(20)
        prog.InstructionDelete(ins_id)
        # prog.RunInstruction(ins_call, INSTRUCTION_CALL_PROGRAM)
        # Advance one additional instruction as we just added another instruction
        ins_id = ins_id + 1
        # ins_count = ins_count + 1
    elif move_type == 1:
        
        target = RDK.AddTarget('Start Target')
        target.setAsCartesianTarget()
        target.setPose(pose)
        prog.setInstruction(ins_id, 'moveL 1', ins_type, 2, 0, pose, joints)
        # print(ins_id)
        ins_id = ins_id + 1
    elif ins_id == 6:
        # calculating target relative to start target.
    
        targetpose = target.Pose()
        pos_i = targetpose.Pos()
        rot_i = targetpose.VZ()
        mmdown = 5
        pos_i[2] = pos_i[2] - mmdown * rot_i[2]
        pos_i[1] = pos_i[1] - mmdown * rot_i[1]
        targetpose.setPos(pos_i)
        # print(targetpose)
        # print(targetpose)
        target = RDK.AddTarget('Target Down')
        target.setAsCartesianTarget()
        target.setPose(targetpose)
        prog.setInstruction(ins_id, 'moveL Down', ins_type, 2, 0, targetpose, joints)
        ins_id = ins_id + 1
    elif ins_type == INS_TYPE_CHANGESPEED and ins_id > 20:
        prog.InstructionSelect(ins_id)
        prog.setSpeed(10)
        prog.InstructionDelete(ins_id)
        ins_id = ins_id + 1
        active = True
    elif active:
        target_end = RDK.AddTarget('End Target')
        target_end.setAsCartesianTarget()
        target_end.setPose(pose)
        target_end = target_end.Pose()
        pos_i = target_end.Pos()
        rot_i = target_end.VZ()
        mmup = 20
        pos_i[2] = pos_i[2] + mmup * rot_i[2]
        pos_i[1] = pos_i[1] + mmup * rot_i[1]
        target_end.setPos(pos_i)
        # print(target_end)
        target_endpose = RDK.AddTarget('Target Up')
        target_endpose.setAsCartesianTarget()
        target_endpose.setPose(target_end)
        prog.setInstruction(ins_id, 'moveL Up', ins_type, 2, 0, target_end, joints)
        ins_id = ins_id + 1
    else:
        ins_id = ins_id + 1

"""""""""
file_path = RDK.getParam('PATH_OPENSTATION') + '\\joints.txt'
fid = open(file_path,'w')
tic()
while (False):
    time = toc()
    print('Current time (s):' + str(time))
    #joints = str(robot.Joints().tolist())
    poses = str(robot.Pose().tolist)
    #fid.write(str(time) + ', ' + joints[1:-1] + ', ' + poses[1:-1] + '\n')
    fid.write(str(time) + ', ' + poses + '\n')
    pause(0.005)
fid.close()


#robot.setPoseTool(transl(167.8, -52, 55)*rotz(-pi)*roty(-pi/2)*rotx(pi))
#connectOnline = RDK.Connect(robot_ip = '192.168.100.147', blocking = True)
#Connect(robot_ip = '192.169.100.147', blocking = True):


## prog.setRunType(PROGRAM_RUN_ON_ROBOT) # Run on robot option
#prog.setRunType(PROGRAM_RUN_ON_SIMULATOR) # Run on simulator (default on startup)

#  Example of the message box  ###  secon = mbox('How many seconds sleep', 'Ok', 'Cancel')
#time.sleep(3)
#robot.setTool(tool2)
#progPIT.RunCode()

#time.sleep(3)
#RDK.CloseStation()
#RDK.CloseRoboDK()

#state = robot.Connect()
#print("The robot connection state is " , state)
#state, msg = robot.ConnectedState()
#print(state)
#print(msg)
#if state != ROBOTCOM_READY:
#    print('Problems connecting: ' + robot.Name() + ': ' + msg)
# quit()
"""""""""""