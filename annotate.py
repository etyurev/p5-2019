import numpy as np
import sys
sys.path.insert(0, 'include/')
from pointCloud import PointCloud
from pointCloudLine import PointCloudLine
from trajectory import Trajectory
import matplotlib.pyplot as plt
import open3d as o3d
import time
from threading import Thread, Lock
import os
import yaml

def press(event):

    global i, index_1, index_2, indexes, close_window
    if event.key == 'left':
        i += -1
        index_1 = 0
        index_2 = 0
        updateGraph()
        updateCurrentEdges()
    elif event.key == 'right':
        i += 1
        index_1 = 0
        index_2 = 0
        updateGraph()
        updateCurrentEdges()
        #plt.close(fig='all')
    elif event.key == 'x':
        if index_1 != 0 and index_2 != 0:
            print(i, ": Edges ", index_1, " and ", index_2, " saved!")
            indexes[0, i] = index_1
            indexes[1, i] = index_2
        else:
            print("Need two edges to save, or reselect.")


def updateGraph():

    global i

    pcl = PointCloudLine(pcd.pointCloudLine[i], 1)
    npArr = np.arange(len(pcl.x))
    plt.clf()
    plt.scatter(npArr, pcl.z, s=1, color='b')

    if index_1 != 0:
        plt.plot(index_1, pcl.z[index_1], 'ro')
    elif indexes[0, i] != 0:
        plt.plot(indexes[0, i], pcl.z[indexes[0, i]], 'ro')
    if index_2 != 0:
        plt.plot(index_2, pcl.z[index_2], 'ro')
    elif indexes[1, i] != 0:
        plt.plot(indexes[1, i], pcl.z[indexes[1, i]], 'ro')
    plt.ylabel("z coordinate")
    plt.xlabel("x coordinate")
    plt.draw()

def pclVisThread():
    global close_window
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd.pcd)
    if 'trajectory1' in globals():
        vis.add_geometry(trajectory1.pcd)
    if 'trajectory2' in globals():
        vis.add_geometry(trajectory2.pcd)
    if 'currentEdges' in globals():
        vis.add_geometry(currentEdges.pcd)

    while close_window == False:
        vis.update_geometry()
        vis.poll_events()
        vis.update_renderer()
        time.sleep(0.05)

    vis.destroy_window()
    return 1

def updateCurrentEdges():
    pcl = PointCloudLine(pcd.pointCloudLine[i], 1)
    edgesI = []
    if indexes[0, i] != 0:
        if pcl.z[indexes[0, i]] != 0:
            edgesI.append(np.array([pcl.x[indexes[0, i]], pcl.y[i], pcl.z[indexes[0, i]]]))
    else:
        edgesI.append(np.array([1, pcl.y[i], 0]))
    if indexes[1, i] != 0:
        if pcl.z[indexes[1, i]] != 0:
            edgesI.append(np.array([pcl.x[indexes[1, i]], pcl.y[i], pcl.z[indexes[1, i]]]))
    else:
        edgesI.append(np.array([-1, pcl.y[i], 0]))

    edgesI = np.array(edgesI)
    currentEdges.pcd.clear()
    currentEdges.pcd.points = o3d.utility.Vector3dVector(edgesI)
    #currentEdges.setOffset(0, 0, 0.15)
    currentEdges.setColor('r')

def tempCurrentEdges():

    global index_1, index_2
    pcl = PointCloudLine(pcd.pointCloudLine[i], 1)
    edgesI = []
    if index_1 == 0:
        index_1 = indexes[0, i]
    if index_2 == 0:
        index_2 = indexes[1, i]
    edgesI.append(np.array([pcl.x[index_1], pcl.y[i], pcl.z[index_1]]))
    edgesI.append(np.array([pcl.x[index_2], pcl.y[i], pcl.z[index_2]]))


    edgesI = np.array(edgesI)
    currentEdges.pcd.clear()
    currentEdges.pcd.points = o3d.utility.Vector3dVector(edgesI)
    currentEdges.setOffset(0, 0, 0.03)
    currentEdges.setColor('r')

def onclick(event):
    if event.xdata != None and event.ydata != None:
        global index_1, index_2
        if event.button == 1:
            index_1 = int(round(event.xdata,0))
        elif event.button == 3:
            index_2 = int(round(event.xdata,0))
        updateGraph()
        tempCurrentEdges()


#scan_file_path = "scans/before/unprocessed/"
FILE_NAME = "11-6-T-before"
#scan_file_extension = ".pcd"
#scan_file = (scan_file_path + scan_file_name + scan_file_extension)

annotations_file_path = "annotations/validation/"
annotation_file_name = FILE_NAME
annotation_file_ext = ".txt"
annotation_file = (annotations_file_path + annotation_file_name + annotation_file_ext)

with open("config.yaml", 'r') as ymlfile:
    cfg = yaml.load(ymlfile)

def pcdLoader(file_name = ""):
    file_name_npy = (file_name + cfg['pcd_file'].get("ext_npy"))
    file_name_pcd = (file_name + cfg['pcd_file'].get("ext_pcd"))
    file_name_txt = (file_name + cfg['pcd_file'].get("ext_txt"))
    processed_file = (cfg['pcd_file'].get("path_processed") + file_name_npy)
    unprocessed_file_pcd = (cfg['pcd_file'].get("path_unprocessed") + file_name_pcd)
    unprocessed_file_txt = (cfg['pcd_file'].get("path_unprocessed") + file_name_txt)

    if os.path.exists(processed_file) == True:
        print(processed_file)
        npArr = np.load(processed_file)
        pcd = PointCloud(npArr, type = 'NPARR')
        pcd.pcd = o3d.geometry.PointCloud.remove_statistical_outlier(pcd.pcd, 1000, 1)[0]
        return pcd

    elif os.path.exists(unprocessed_file_txt) == True:
        print(unprocessed_file_txt)
        npArr = np.loadtxt(fname = unprocessed_file_txt)
        npArr = np.delete(npArr, 3, 1)
        pcd = PointCloud(npArr, type='NPARR')
        print("Preprocessing point cloud...")
        pcd.deleteZPoints(0)
        pcd.setOffset(0, 0, -120)
        pcd.pcd = o3d.geometry.PointCloud.remove_statistical_outlier(pcd.pcd, 1000, 1)[0]
        pcd.getParameters()
        np.save(processed_file, np.asarray(pcd.pcd.points))
        return pcd

    elif os.path.exists(unprocessed_file_pcd) == True:
        print(unprocessed_file_pcd)
        pcd = PointCloud(unprocessed_file_pcd, type='PCD')
        print("Preprocessing point cloud...")
        pcd.deleteZPoints(0) # Delete all points with z = 0
        pcd.setOffset(0, 0, -120)
        pcd.pcd = o3d.geometry.PointCloud.remove_statistical_outlier(pcd.pcd, 1000, 1)[0]
        pcd.getParameters()
        np.save(processed_file, np.asarray(pcd.pcd.points))
        return pcd

    else:
        print("Couldnt find point cloud.")
        raise
        return False


pcd = pcdLoader(file_name = FILE_NAME)
#pcd.deleteZPoints(0) # Delete all points with z = 0
pcd.setOffset(0, 0,-120)
pcd.flip()

pcd.getPointCloudLines() # Construct arrays of point cloud lines

if os.path.exists(annotation_file) == True:
    indexes = np.loadtxt(annotation_file, dtype=int)
else:
    indexes = np.zeros(len(pcd.pointCloudLine)*2).reshape(2, -1)
    indexes = indexes.astype(int)

traj1, traj2 = [], []
for i in range(len(pcd.pointCloudLine)):
    pcl = PointCloudLine(pcd.pointCloudLine[i], 1)
    try:
        if indexes[0, i] != 0:
            if pcl.z[indexes[0, i]] != 0:
                traj1.append(np.array([pcl.x[indexes[0, i]], pcl.y[0], pcl.z[indexes[0, i]]]))

        if indexes[1, i] != 0:
            if pcl.z[indexes[1, i]] != 0:
                traj2.append(np.array([pcl.x[indexes[1, i]], pcl.y[0], pcl.z[indexes[1, i]]]))
    except Exception as e:
        pass



traj1 = np.array(traj1)
traj2 = np.array(traj2)

i = 0
index_1, index_2 = 0, 0
pcl = PointCloudLine(pcd.pointCloudLine[i], 1)
edgesI = []
if indexes[0, i] != 0:
    if pcl.z[indexes[0, i]] != 0:
        edgesI.append(np.array([pcl.x[indexes[0, i]], pcl.y[i], pcl.z[indexes[0, i]]]))
else:
    edgesI.append(np.array([1, pcl.y[i], 0]))
if indexes[1, i] != 0:
    if pcl.z[indexes[1, i]] != 0:
        edgesI.append(np.array([pcl.x[indexes[1, i]], pcl.y[i], pcl.z[indexes[1, i]]]))
else:
    edgesI.append(np.array([-1, pcl.y[i], 0]))
edgesI = np.array(edgesI)


if len(traj1) != 0:
    trajectory1 = Trajectory(traj1)
    trajectory1.setOffset(0, 0, 0.01)
    trajectory1.setColor('b')
if len(traj2) != 0:
    trajectory2 = Trajectory(traj2)
    trajectory2.setOffset(0, 0, 0.01)
    trajectory2.setColor('b')
if len(edgesI) !=0:
    currentEdges = Trajectory(edgesI)
    #currentEdges.setOffset(0, 0, 0.15)
    currentEdges.setColor('g')


close_window = False

pclThread = Thread(target=pclVisThread)
pclThread.start()

npArr = np.arange(len(pcd.pointCloudLine[i]))
fig, ax = plt.subplots()
ax.scatter(npArr,pcd.pointCloudLine[i][0:len(pcd.pointCloudLine[0]), 2], s = 1, color='r')
ax.set_ylabel("z coordinate")
ax.set_xlabel("x coordinate")
cid = fig.canvas.mpl_connect('button_press_event', onclick)
did = fig.canvas.mpl_connect('key_press_event', press)
plt.tight_layout()
plt.show()
plt.draw()
close_window = True

pclThread.join()

try:
    np.savetxt((annotation_file), indexes, fmt='%d')
    print("Saved edges.")
except Exception as e:
    print("Couldnt save edges")
