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

def findTrajectories(pcdName = "", visualize = False, LOAD_ROI = False, compute_ROI = True):
    net = loadClassifier()

    print("Loading point cloud...")

    pcd = pcdLoader(file_name = pcdName, afterPIT = False, load=False)
    pcd.pcd = o3d.geometry.PointCloud.remove_statistical_outlier(pcd.pcd, 1000, 1)[0]
    #pcd.getParameters()
    y_max = pcd.y_max
    y_min = pcd.y_min
    y_range = pcd.y_range+1
    #pcd.setOffset(0, 0, 120)
    pcd.getPointCloudLines() # Construct arrays of point cloud lines
    npArr = pcd.size_threshold_pointCloudLine(threshold = 0.8)
    pcd.pcd_from_pointCloudLines(npArr)
    img = pcd.pcd_to_image()
    npArr = pcd.img_to_pcd(img)
    pcd = PointCloud(npArr, type='NPARR')

    # Normalize point cloud
    pcdNormalized = PointCloud(pcd.normalize(), type ='NPARR')
    pcdNormalized.getPointCloudLines()
    img = pcdNormalized.pcd_to_image()

    if LOAD_ROI == True:
        if os.path.exists((cfg['pcd_file'].get("ROI") + pcdName + cfg['pcd_file'].get("ext_npy"))) == True:
            ROI = roiLoader(file_name = (cfg['pcd_file'].get("ROI") + pcdName + cfg['pcd_file'].get("ext_npy")))
        else:
            print("Couldn't find previously computed regions of interest.")

    else:
        ROI = getROI(pcdNormalized, net, FILE_NAME=pcdName)

    print("ROI to 2D for segmentation...")

    if visualize == True:
        o3d.visualization.draw_geometries([pcdNormalized.pcd])
    roi_img = pcdNormalized.mask_image(img, ROI)
    roi_img = filters.median(roi_img, np.ones((3, 3)))

    ROI = PointCloud(ROI, type='NPARR')
    ROI.setColor('b')

    if visualize == True:
        o3d.visualization.draw_geometries([pcdNormalized.pcd, ROI.pcd])

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
        npArr = pcdNormalized.mask_pcd(maskLines, border = cfg['model'].get("step_size"))
        if len(npArr) != 0:
            weld_toe.append(Trajectory(npArr))
            weld_toe[-1].setColor('b')

    if visualize == True:
        o3d.visualization.draw_geometries([pcd.pcd, weld_toe[0].pcd, weld_toe[1].pcd])

    trajectory = []
    print("Finding edges...")
    for toe in weld_toe:
        toe.getPointCloudLines()
        coordinates = []
        for points in tqdm(toe.pointCloudLine):
            pcl = PointCloudLine(points, 1)
            predictions = []
            for i in range(len(pcl.x)):
                if i > cfg['model'].get("step_size") and i < (len(pcl.x)-cfg['model'].get("step_size")):

                    x = pcl.getFeatureVectorZOnly(i, cfg['model'].get("step_size"))
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
        #trajectory[i] = Trajectory(trajectory[i])


        trajectory[i].setColor('r')
    #pcd.flip()
    if visualize == True:
        o3d.visualization.draw_geometries([pcd.pcd, trajectory[0].pcd, trajectory[1].pcd])
    return trajectory

with open("config.yaml", 'r') as ymlfile:
    cfg = yaml.load(ymlfile)

annotations_file_path = "annotations/validation/"
log_path = "logs/"
ext_txt = ".txt"
ext_npy = ".npy"

for f in tqdm(os.listdir(annotations_file_path)):
    FILE_NAME = f.replace('.txt', '')
    print(FILE_NAME)
    log_positive = (log_path + FILE_NAME + "_positive" + ext_npy)
    log_negative = (log_path + FILE_NAME + "_negative" + ext_npy)
    trajectory = findTrajectories(pcdName = FILE_NAME, visualize = False, LOAD_ROI = False, compute_ROI = True)

    ind = np.loadtxt((annotations_file_path + FILE_NAME + ext_txt), dtype=int)
    pcdAno = pcdLoader(file_name = FILE_NAME, afterPIT = False, load = False)
    pcdAno.pcd = o3d.geometry.PointCloud.remove_statistical_outlier(pcdAno.pcd, 1000, 1)[0]
    pcdAno.getPointCloudLines()

    points_neg, points_pos = [],[]
    for i in range(len(pcdAno.pointCloudLine)):
        pcl = PointCloudLine(pcdAno.pointCloudLine[i], 1)
        for k in range(len(ind)):
            try:
                if ind[k, i] != 0:

                    index = ind[k,i]
                    if pcl.x[index] > 0:
                        points_pos.append(np.array([pcl.x[index], pcl.y[index], pcl.z[index]]))
                    elif pcl.x[index] < 0:
                        points_neg.append(np.array([pcl.x[index], pcl.y[index], pcl.z[index]]))
            except Exception as e:
                print("something is wrong")
                pass
    points_pos = np.array(points_pos)
    points_neg = np.array(points_neg)
    annotaed_pos = Trajectory(points_pos)
    annotaed_neg = Trajectory(points_neg)
    #o3d.visualization.draw_geometries([trajectory[0].pcd, trajectory[1].pcd, annotaed_neg.pcd, annotaed_pos.pcd])

    trajectory[0].getPointCloudLines()
    diffArr = []
    for line in trajectory[0].pointCloudLine:
        y = round(line[0][1],1)
        index = np.where(points_neg[:][:,1] == y)
        if len(index[0]) is not 0:
            index = index[0][0]
            difference = line[0][0]-points_neg[index][0]
            diffArr.append(difference)
    diffArr = np.array(diffArr)
    np.save(log_negative, diffArr)

    print("Negative --- ", "mean: ", np.mean(diffArr), " minimum: ", np.amin(diffArr), " maximum: ", np.amax(diffArr), " std: ", np.std(diffArr))

    diffArr = []
    trajectory[1].getPointCloudLines()
    for line in trajectory[1].pointCloudLine:
        y = round(line[0][1],1)
        index = np.where(points_pos[:][:,1] == y)
        if len(index[0]) is not 0:
            index = index[0][0]
            difference = line[0][0]-points_pos[index][0]
            diffArr.append(difference)
    diffArr = np.array(diffArr)

    print("Positve --- ", "mean: ", np.mean(diffArr), " minimum: ", np.amin(diffArr), " maximum: ", np.amax(diffArr), " std: ", np.std(diffArr))

    np.save(log_positive, diffArr)
