import sys
sys.path.insert(0, 'include/')

import numpy as np
from pointCloud import PointCloud
from pointCloudLine import PointCloudLine
from net import Net
from helperFunctions import *
import matplotlib.pyplot as plt
from numpy import fft
from scipy.signal import butter, filtfilt
import open3d as o3d
from trajectory import Trajectory
import heapq
import copy
from tqdm import tqdm
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from skimage import data
from skimage.filters import threshold_otsu
from skimage.segmentation import clear_border
from skimage.measure import label, regionprops
from skimage.morphology import closing, square
from skimage.color import label2rgb
from skimage import filters
import yaml
import os

xOff_l = -2.5
xOff_r = -1.5

FILE_NAME = "11-12-19-scan5"
LOAD_ROI = True
LOAD_PCD = False

with open("config.yaml", 'r') as ymlfile:
    cfg = yaml.load(ymlfile)

net = loadClassifier()

print("Loading point cloud...")

pcd = pcdLoader(file_name = FILE_NAME, afterPIT=False, load=LOAD_PCD)
y_max = pcd.y_max
y_min = pcd.y_min
y_range = pcd.y_range
print(pcd.y_resolution)
print(y_max, y_min, y_range)
o3d.visualization.draw_geometries([pcd.pcd])
#pcd.setOffset(0, 0, -120)
pcd.getPointCloudLines() # Construct arrays of point cloud lines

#npArr = pcd.size_threshold_pointCloudLine(threshold = 0.8)
#pcd.pcd_from_pointCloudLines(npArr)
img = pcd.pcd_to_image()
npArr = pcd.img_to_pcd(img)
pcd = PointCloud(npArr, type='NPARR')
pcd.getPointCloudLines() # Construct arrays of point cloud lines
#print(len(pcd.pointCloudLine))

# Normalize point cloud
pcdNormalized = PointCloud(pcd.normalize(), type ='NPARR')
pcdNormalized.getPointCloudLines()
#print(len(pcdNormalized.pointCloudLine))
img = pcdNormalized.pcd_to_image()


if LOAD_ROI == True:
    if os.path.exists((cfg['pcd_file'].get("ROI") + FILE_NAME + cfg['pcd_file'].get("ext_npy"))) == True:
        ROI = roiLoader(file_name = (cfg['pcd_file'].get("ROI") + FILE_NAME + cfg['pcd_file'].get("ext_npy")))
    else:
        print("Couldn't load regions of interest.")
else:
    ROI = getROI(pcdNormalized, net=net, FILE_NAME=FILE_NAME)

print("ROI to 2D for segmentation...")

roi_img = pcdNormalized.mask_image(img, ROI)

ROI = PointCloud(ROI, type='NPARR')
ROI.setColor('b')

print("Segmenting regions of interests...")
# apply threshold
bw = roi_img > 0

# label image regions
label_image = label(bw, connectivity=2)

regions = ROI.thresArea(label_image, thres = 3000) #4000 for 12-1-T-before
masks = ROI.getMask(regions, pcdNormalized.pointCloudLine)

print("Applying mask to point cloud...")
weld_toe = []

for mask in masks:
    maskLines = ROI.maskToLines(mask)
    npArr = pcdNormalized.mask_pcd(maskLines, border = cfg['feature_vector'].get("border_x"))
    if len(npArr) != 0:
        weld_toe.append(Trajectory(npArr))
        weld_toe[-1].setColor('b')

trajectory = []
print("Finding edges...")
for toe in weld_toe:
    toe.getPointCloudLines()
    coordinates = []
    for points in tqdm(toe.pointCloudLine):
        pcl = PointCloudLine(points, 1)
        predictions = []
        for i in range(len(pcl.x)):
            if i > cfg['feature_vector'].get("border_x") and i < (len(pcl.x)- cfg['feature_vector'].get("border_x")):

                x = pcl.getFeatureVector(i, cfg['feature_vector'].get("border_x"), x=cfg['feature_vector'].get("x"), y=cfg['feature_vector'].get("y"), z=cfg['feature_vector'].get("z"))
                x = torch.from_numpy(x)

                conf= net.predict_confidence(x.float())
                predictions.append(np.array([conf, i]))

        npArray = np.array(predictions)
        if len(npArray) != 0:
            max_confidence = np.amax(npArray[:][:,0])
            result = np.where(npArray[:][:,0] == max_confidence)
            if len(result[0]) is not 0:
                npArr_index = result[0][0]
                coordinates.append(np.array([pcl.x[int(npArray[npArr_index][1])], pcl.y[0], pcl.z[int(npArray[npArr_index][1])]]))
        predictions[:] = []
    coordinates = np.array(coordinates)
    trajectory.append(coordinates)

npArr = pcd.unNormalize()
pcd = PointCloud(npArr, type ='NPARR')
pcd.getPointCloudLines()

for i in range(len(trajectory)):
    trajectory[i] = Trajectory(trajectory[i])

    npArr = trajectory[i].unNormalize(pcd.z_max, pcd.z_min)
    trajectory[i] = Trajectory(npArr)

    """
    x_trajectory = trajectory[i].smooth_outliers(npArr[:][:,0])

    smooth_x = trajectory[i].create_polynomial(npArr[:][:,1],x_trajectory, npArr[:][:,2], pcd.y_resolution, y_range, y_min, POLY_DEGREE = 7)
    y_values = trajectory[i].xx_value
    polynomial = np.array([smooth_x, y_values])
    z_value = []
    traj_npArr = []
    k = 1
    for j in range(y_range+1):
        if j < len(y_values):
            traj_npArr.append(np.array([smooth_x[0][j],y_values[j], smooth_x[1][j]]))
        else:
            traj_npArr.append(np.array([smooth_x[0][len(y_values)-1],(y_values[len(y_values)-1]+pcd.y_resolution*k), smooth_x[1][len(y_values)-1]]))
            k += 1
    traj_npArr = np.array(traj_npArr)
    trajectory[i] = Trajectory(traj_npArr)
    """


    trajectory[i].getPointCloudLines()
    #for line in trajectory[i].pointCloudLine:
    #    print(line[0][1])

    trajectory[i].setColor('g')
#pcd.flip()
trajectory[0].setOffset(xOff_l, 0, 0)
trajectory[0].save(0, FILE_NAME)
trajectory[1].setOffset(xOff_r, 0, 0)
trajectory[1].save(1, FILE_NAME)
#trajectory[i].flip()

o3d.visualization.draw_geometries([pcd.pcd, trajectory[0].pcd, trajectory[1].pcd])
o3d.visualization.draw_geometries([pcd.pcd, trajectory[0].pcd])
o3d.visualization.draw_geometries([pcd.pcd, trajectory[1].pcd])
