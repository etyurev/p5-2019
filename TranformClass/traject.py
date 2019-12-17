import sys
import numpy as np
import re
sys.path.append("/Users/mathiaspoulsen/Desktop/PythonProjects/")
from transformation import transformation


#example use
path_folder = "/Users/mathiaspoulsen/Desktop/PythonProjects/"
name_kuka_ipo = "WINDOWS-P92OSKB-Wall-E-501514_19gr563_KRCIpo.r64"
name_kuka_io = "WINDOWS-P92OSKB-Wall-E-501514_19gr563_KRCIO.r64"

path_kuka_ipo = path_folder + name_kuka_ipo
path_kuka_io = path_folder + name_kuka_io

#only used for debug.
name_trajectory = "localTrajectory0.txt"

if __name__ == "__main__":
    #create class and call these 3 functions
    trans=transformation()
    trans.reading_from_file(path_kuka_ipo,path_kuka_io)
    trans.find_start_scan()
    trans.find_scan_coordinates()

    #reading trajectories manually only for debug
    f=open(path_folder+name_trajectory)
    temp=f.read()
    temp=re.split(" |\n|;",temp)
    local_traject_poses = np.zeros((int(len(temp)/3),3))
    for i in range(0,int(len(temp)/3)):
        local_traject_poses[i][0]=float(temp[i*3])
        local_traject_poses[i][1]=float(temp[i*3+1])
        local_traject_poses[i][2]=float(temp[i*3+2])
    f.close()

    #returns the curve for robodk as numpy
    curve_roboDK=trans.calc_pose_pit_robodk(local_traject_poses,20)

