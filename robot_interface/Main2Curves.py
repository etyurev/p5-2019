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
from jpype import*

#### Run Java script
#startJVM(getDefaultJVMPath(),"-ea")
#JClass('C:\\Users\\etyur\\Documents\\ROBOTICS_AAU\\5_SEMESTER\\P_5\\ConnectScannerJAVA\\ConnectScannerJNI')
#time.sleep(10)
#shutdownJVM()


####### SETUP of THE  STation ######
RDK = Robolink()
print(RDK._is_connected())
os.system('C:\\Users\\etyur\\Documents\\ROBOTICS_AAU\\5_SEMESTER\\P_5\\RoboDK_P5\\KUKA_KR60_3\\KUKAKR60_301119.rdk')
robot = RDK.ItemUserPick('Select a robot', ITEM_TYPE_ROBOT)
if not robot.Valid():
	raise Exception('No robot selected or available')

scanning_speed=10
speed1_left=20
speed1_right=20

speed2_left=3
speed2_right=3

pitAngle_left=10
pitAngle_right=-10

startScanPose=[(1116.24+280),40.339,63.123,-90.576,0.007,0.001]
#[(925.768+490),26.351,57.423,-90.576,0.007,0.001]#[1492.74, -303.42, 64.312, -90.58, 0.01,0.0]
home=[(startScanPose[0]), (startScanPose[1]-20), (startScanPose[2]+50), (startScanPose[3]), (startScanPose[4]),(startScanPose[5])]
scannerPose = [167.8, -52, 55, -180, -90, 180] # [167.8,-52,57,-180,-90,180]
toolS = KUKA_2_Pose(scannerPose)
robot.AddTool(toolS, 'Scanner')
# pitPose = [-175.6,344.02,54.15,180,0,-90]#[-176.04,243.35,54.76,180,0,-90] #[-176.5,245,57,180,0,-90]
# PIT TOOL calibr 11.12.19 #[-174.98,344.67,55.17,180,0,-90]
# PIT TOOL calibr 05.12.19 #[-175.6,344.02,53.86,180,0,-90]
# PIT TOOL used 05.12.19 #[-174.95, 333, 55.17, 180, 0, -90]
pitPose = [-174.95, 333, 57, 180, 0, -90]  #    [-175.6,330.02,54.15,180,0,-90]
toolP = KUKA_2_Pose(pitPose)
robot.AddTool(toolP, 'PIT')
tool1 = RDK.Item('Scanner')
tool2 = RDK.Item('PIT')
target1 = RDK.AddTarget('Start Scan Target')
target1.setAsCartesianTarget()
target1.setPose(KUKA_2_Pose(startScanPose))  # #[1492.74,-303.42,494.43,-91.87,0.0,-0.02] #[1344.917,-298.897,64.312,-90.770,0.015,0.134]
target2 = RDK.AddTarget('End Scan Target')
target2.setAsCartesianTarget()
target2.setPose(KUKA_2_Pose([(startScanPose[0]+70), (startScanPose[1]), (startScanPose[2]), (startScanPose[3]), (startScanPose[4]), (startScanPose[5])]))  # [1561.513,-303.42,494.43,-91.87,0.0,-0.02]  [1413.690,-300.074,64.454,-91.87,0.0,-0.02]
target3 = RDK.AddTarget('Scanner Ready to Work Target')
target3.setAsCartesianTarget()
target3.setPose(KUKA_2_Pose(home))
target4 = RDK.AddTarget('PIT Ready to Work Target')
target4.setAsCartesianTarget()
target4.setPose(KUKA_2_Pose([(home[0]+50),(home[1]+50),(home[2]-98.937),(home[3]+90), (home[4]),(home[5])]))
path_folder = "C:\\Users\\etyur\\Documents\\ROBOTICS_AAU\\5_SEMESTER\\P_5\\RoboDK_P5\\KUKA_KR60_3\\TranformClass\\"
scanProg = RDK.AddProgram('ScanProgramm', ITEM_TYPE_PROGRAM)
scanProg.ShowInstructions(False)
base_reference_pose = KUKA_2_Pose([0, 0, 0, 0, 0, 0])
scanProg.setPoseFrame(base_reference_pose)
scanProg.setTool(tool1)
scanProg.setSpeed(scanning_speed)
robot.setPoseTool(tool1)
scanProg.MoveL(target1)
scanProg.setDO('1', 1)
scanProg.setDO('1', 0)
scanProg.MoveL(target2)
scanProg.MoveL(target3)
# scanProg.RunCode()
while scanProg.Busy() == 1:
	pause(0.1)
print("Scan Programs done")


#Check the availability of the file in the scanner folder

#Load scan to the Program.py


#time.sleep(3)
#os.system("C:\\Users\\etyur\\Documents\\ROBOTICS_AAU\\5_SEMESTER\\P_5\\RoboDK_P5\\KUKA_KR60_3\\P5-master\\program.py")

#Transfer Traces from the MX Automation


#Run Computer Vision


path_folder = "C:\\Users\\etyur\\Documents\\ROBOTICS_AAU\\5_SEMESTER\\P_5\\RoboDK_P5\\KUKA_KR60_3\\TranformClass\\"
name_kuka_ipo = "11-12-19-scan5-Ipo.r64"
name_kuka_io = "11-12-19-scan5-IO.r64"
path_kuka_ipo = path_folder + name_kuka_ipo
path_kuka_io = path_folder + name_kuka_io

# only used for debug.
name_trajectory = "11-12-19-scan5-trajectory-0.txt"  # -1.txt right side / -0.txt left
# create class and call these 3 functions
trans = transformation()
trans.reading_from_file(path_kuka_ipo, path_kuka_io)
trans.find_start_scan()
trans.find_scan_coordinates()
# reading trajectories manually only for debug
f = open(path_folder + name_trajectory)
temp = f.read()
temp = re.split(" |\n|;", temp)
local_traject_poses = np.zeros((int(len(temp) / 6), 3))
for i in range(0, int(len(temp) / 6)):
	local_traject_poses[i][0] = -float(temp[i * 6])
	local_traject_poses[i][1] = float(temp[i * 6 + 1])
	local_traject_poses[i][2] = -float(temp[i * 6 + 2])
f.close()
print(len(local_traject_poses))


# returns the curve for robodk as numpy
curve_roboDK = trans.calc_pose_pit_robodk(local_traject_poses, pitAngle_left)  # -20 the right side/ 20 left
listCurve = curve_roboDK.tolist()
object_curve = RDK.AddCurve(listCurve)
object_curve.setName('Curve Trajectory_1 for PIT')
robot.setTool(tool2)
path_settings = RDK.AddMachiningProject("CurveFollow_1 settings")
poseStart = [0, 0, 0, 180, 0, 0]
poseStart = KUKA_2_Pose(poseStart)
path_settings.setPose(poseStart)
path_settings.setSpeed(speed1_left)
prog, status = path_settings.setMillingParameters(part=object_curve,params='NormalApproach=0')  # two return values if they are stay
prog.ShowInstructions(False)
ins_id = 0
ins_count = prog.InstructionCount()
print(ins_count)

active = False
Flag = True

while ins_id < ins_count:
	ins_nom, ins_type, move_type, isjointtarget, pose, joints = prog.Instruction(ins_id)
	# print(isjointtarget)
	# print(pose)
	if ins_type == INS_TYPE_CHANGESPEED and ins_id < 10 and Flag:
		# Changes all speeds to 10 mm/s
		prog.InstructionSelect(ins_id)
		prog.setSpeed(speed1_left)
		prog.InstructionDelete(ins_id)
		# prog.RunInstruction(ins_call, INSTRUCTION_CALL_PROGRAM)
		# Advance one additional instruction as we just added another instruction
		#print('Item number before')
		#print(ins_id)
		ins_id = ins_id + 1
		Flag=False
		print('Speed')
		print(speed1_left)
		
	# ins_count = ins_count + 1
	elif ins_type == INS_TYPE_CHANGESPEED and ins_id < 10 and Flag==False:
		# Changes all speeds to 10 mm/s
		prog.InstructionSelect(ins_id)
		prog.setSpeed(speed2_left)
		prog.InstructionDelete(ins_id)
		# prog.RunInstruction(ins_call, INSTRUCTION_CALL_PROGRAM)
		# Advance one additional instruction as we just added another instruction
		#print('Item number after')
		#print(ins_id)
		ins_id = ins_id + 1
		Flag=True
		print('Speed')
		print(speed2_left)
		
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
		mmdown = 6
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
		prog.setSpeed(speed1_left)
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
		mmup = 30
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
		
	print(ins_type)
	#print(ins_id)



#Right curve

name_trajectory = "11-12-19-scan5-trajectory-1.txt"  # -1.txt right side / -0 left
# create class and call these 3 functions
trans = transformation()
trans.reading_from_file(path_kuka_ipo, path_kuka_io)
trans.find_start_scan()
trans.find_scan_coordinates()
# reading trajectories manually only for debug
f = open(path_folder + name_trajectory)
temp = f.read()
temp = re.split(" |\n|;", temp)
local_traject_poses = np.zeros((int(len(temp) / 6), 3))
for i in range(0, int(len(temp) / 6)):
	local_traject_poses[i][0] = -float(temp[i * 6])
	local_traject_poses[i][1] = float(temp[i * 6 + 1])
	local_traject_poses[i][2] = -float(temp[i * 6 + 2])
	#print(local_traject_poses[i][2])
f.close()
print(len(local_traject_poses))
# returns the curve for robodk as numpy
curve_roboDK = trans.calc_pose_pit_robodk(local_traject_poses, pitAngle_right)  # -20 the right side/ 20 left
listCurve = curve_roboDK.tolist()
object_curve = RDK.AddCurve(listCurve)
object_curve.setName('Curve Trajectory_2 for PIT')
robot.setTool(tool2)
path_settings = RDK.AddMachiningProject("CurveFollow_2 settings")
poseStart = [0, 0, 0, 180, 0, 0]
poseStart = KUKA_2_Pose(poseStart)
path_settings.setPose(poseStart)
operationSpeed = 10
path_settings.setSpeed(speed1_right)

prog, status = path_settings.setMillingParameters(part=object_curve, params='NormalApproach=0')  # two return values if they are stay
prog.ShowInstructions(False)

ins_id = 0
ins_count = prog.InstructionCount()
print(ins_count)
active = False
while ins_id < ins_count:
	
	ins_nom, ins_type, move_type, isjointtarget, pose, joints = prog.Instruction(ins_id)
	# print(isjointtarget)
	# print(pose)
	if ins_type == INS_TYPE_CHANGESPEED and ins_id < 10 and Flag:
		# Changes all speeds to 10 mm/s
		prog.InstructionSelect(ins_id)
		prog.setSpeed(speed1_right)
		prog.InstructionDelete(ins_id)
		# prog.RunInstruction(ins_call, INSTRUCTION_CALL_PROGRAM)
		# Advance one additional instruction as we just added another instruction
		ins_id = ins_id + 1
		Flag = False
	# ins_count = ins_count + 1
	elif ins_type == INS_TYPE_CHANGESPEED and ins_id < 10 and Flag == False:
		# Changes all speeds to 10 mm/s
		prog.InstructionSelect(ins_id)
		prog.setSpeed(speed2_right)
		prog.InstructionDelete(ins_id)
		# prog.RunInstruction(ins_call, INSTRUCTION_CALL_PROGRAM)
		# Advance one additional instruction as we just added another instruction
		ins_id = ins_id + 1
		Flag = True
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
		mmdown = 6
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
		prog.setSpeed(speed2_right)
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
		mmup = 30
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

pitProg = RDK.AddProgram('PIT_EXECUTE_BOTH_SIDES', ITEM_TYPE_PROGRAM)
pitProg.ShowInstructions(True)
pitProg.RunInstruction('CurveFollow_1', INSTRUCTION_CALL_PROGRAM)
pitProg.setSpeed(speed2_right)
pitProg.MoveL(target4)
pitProg.RunInstruction('CurveFollow_2', INSTRUCTION_CALL_PROGRAM)
#robot.setTool(tool1)
#pitProg.MoveL(target3)
#pitProg.RunInstruction('ScanProgramm',INSTRUCTION_CALL_PROGRAM)
#scanProg.RunCode()