import sys
sys.path.insert(0, 'include/')

import numpy as np
from pointCloud import PointCloud
from pointCloudLine import PointCloudLine
from quality import Quality
from net import Net
from helperFunctions import *
import matplotlib.pyplot as plt
from numpy import fft
from scipy.signal import butter, filtfilt
import open3d as o3d
from trajectory import Trajectory
import copy
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
import os
import csv
import pandas as pd
import seaborn as sns

#FILE_NAME = "Test-6-Quality"
LOAD_PCD = False
LOAD_ROI = True
DEVIATION_LIMIT = 0.7349

def getPITGrooves(FILE_NAME = "", LOAD_PCD = False, LOAD_ROI = False):
    net = loadClassifier()
    pcd = pcdLoader(file_name = FILE_NAME, afterPIT=True, load=LOAD_PCD)
    # Normalize point cloud
    pcdNormalized = PointCloud(pcd.normalize(), type ='NPARR')
    pcdNormalized.getPointCloudLines() # Construct arrays of point cloud lines

    if LOAD_ROI == True:
        if os.path.exists((cfg['after_file'].get("ROI") + FILE_NAME + cfg['after_file'].get("ext_npy"))) == True:
            ROI = roiLoader(file_name = (cfg['after_file'].get("ROI") + FILE_NAME + cfg['after_file'].get("ext_npy")))
        else:
            print("Couldn't load regions of interest.")
    else:
        ROI = getROI(pcdNormalized, net=net, FILE_NAME=FILE_NAME, afterPIT=True)

    print("ROI to 2D for segmentation...")

    img = pcdNormalized.pcd_to_image()
    roi_img = pcdNormalized.mask_image(img, ROI)
    roi_img = filters.median(roi_img, np.ones((3, 3)))

    ROI = PointCloud(ROI, type='NPARR')
    ROI.setColor('b')

    print("Segmenting regions of interests...")
    # apply threshold
    bw = roi_img > 0

    # label image regions
    label_image = label(bw, connectivity=2)

    regions = ROI.thresArea(label_image, thres = 1500) #4000 for 12-1-T-before
    masks = ROI.getMask(regions, pcdNormalized.pointCloudLine)

    print("Applying mask to point cloud...")
    PIT_groove = []
    pcd.unNormalize()

    for mask in masks:
        maskLines = ROI.maskToLines(mask)
        npArr = pcdNormalized.mask_pcd(maskLines, border = cfg['model_after'].get("step_size"))
        if len(npArr) != 0:
            PIT_groove.append(Trajectory(npArr))

    for i in range(len(PIT_groove)):
        PIT_groove[i].unNormalize(pcd.z_max, pcd.z_min)
        PIT_groove[i].flip()
        PIT_groove[i].setColor('r')

    return (PIT_groove, pcd)

def getQualityReport(pcd, PIT_groove, xOff, FILE_NAME = ""):


    pcd.flip()
    pcd.getPointCloudLines()
    PIT_groove.getPointCloudLines()

    depth, width, deviation, perpline, intercept = [], [], [], [], []

    for i in range(len(PIT_groove.pointCloudLine)):

        try:
            quality = Quality(PIT_groove.pointCloudLine[i], pcd.pointCloudLine[i])
            quality.get_derivative()
            quality.derivative_min_max()
            quality.find_minmax_curvature()
            width.append([quality.get_width_and_midpoint()])
            deviation.append([quality.find_traject(xOff, file = FILE_NAME)])
            perpline.append(quality.pline())
            intercept.append(quality.find_intercept(quality.pline()))
            depth.append([quality.get_depth()])
            #if FILE_NAME == "11-12-19-qual5":
            #print(FILE_NAME, " : ", depth[-1], " ", deviation[-1], " ", width[-1])

        except Exception as e:
            pass
            print(e)

    depth = np.array(depth)
    depth = quality.smooth_outliers(depth[:][:,0],5)
    npArr = []
    for value in depth:
        npArr.append(np.array([value]))
    depth = np.array(npArr)
    width = np.array(width)
    deviation = np.array(deviation)
    deviation = quality.smooth_outliers(deviation[:][:,0],5)
    npArr = []
    for value in deviation:
        npArr.append(np.array([value]))
    deviation = np.array(npArr)

    return (depth, width, deviation)

def plotResults(depth, width, deviation, title = ""):

    fig, axs = plt.subplots(2, 1, constrained_layout=True)
    axs[0].set_title(('Depth ' + title))
    axs[0].plot(depth[:][:,1], depth.smooth_outliers(depth[:][:,0],5))
    axs[0].set_xlabel("Index no.")
    axs[0].set_ylabel("mm.")
    axs[0].hlines(0.1, 0, len(depth[:][:,1]),colors = 'r', linestyles = 'dashed')
    axs[0].hlines(0.6, 0, len(depth[:][:,1]),colors = 'r', linestyles = 'dashed')
    axs[0].grid()
    axs[1].set_title(('Trajectory deviation ' + title))
    axs[1].plot(deviation[:][:,1], deviation[:][:,0])
    axs[1].set_xlabel("Index no.")
    axs[1].set_ylabel("mm.")
    axs[1].hlines(-DEVIATION_LIMIT, 0, len(deviation[:][:,1]),colors = 'r',linestyles = 'dashed')
    axs[1].hlines(DEVIATION_LIMIT, 0, len(deviation[:][:,1]),colors = 'r',linestyles = 'dashed')
    axs[1].grid()

    plt.show()

def loadMetaData(file =""):
    data = pd.read_csv(file)
    return data


with open("config.yaml", 'r') as ymlfile:
    cfg = yaml.load(ymlfile)


data = loadMetaData(file = "test_meta_data_new.csv")
"""
for i in range(int(len(data["File Name"])*0.5)):
    print(data["File Name"][i*2])
    pit_grooves, pcd = getPITGrooves(FILE_NAME = data["File Name"][i*2], LOAD_PCD = False, LOAD_ROI = True)
    for j in range(len(pit_grooves)):
        depth, width, deviation = getQualityReport(pcd, pit_grooves[j], data["Offset"][(i*2)+j], FILE_NAME = data["File Name"][(i*2)+j])
        print(("Test-" + str(data["Test number"][(i*2)+j])))
        #print("Depth mean: ", np.mean(depth[:,0]))
        #print("Width mean: ", np.mean(width[:,0]))
        #print("Devia mean: ", np.mean(deviation[:,0]))
        np.save(("results/Test-"+str(data["Test number"][(i*2)+j]) + "-depth"), depth)
        np.save(("results/Test-"+str(data["Test number"][(i*2)+j]) + "-width"), width)
        np.save(("results/Test-"+str(data["Test number"][(i*2)+j]) + "-deviation"), deviation)
"""

results_path = "results/"
log_path = "logs/"
ext_txt = ".txt"
ext_npy = ".npy"

depth, width, deviation = [None]*4, [], [None]*4
depth_a, depth_b, depth_c, depth_d = np.empty((1)), np.empty(1), np.empty(1), np.empty(1)
width = np.array(width)
dev_a, dev_b, dev_c, dev_d = np.empty(1), np.empty(1), np.empty(1), np.empty(1)
#deviation_a = deviation_a.reshape(-1)
for f in tqdm(os.listdir(results_path)):
    print(f)
    result = np.load((results_path + f))
    #plt.plot(result[:,1], result[:,0])
    #plt.tight_layout()
    #plt.show()
    #print(result)
    result = result[:,0]

    f = f.replace('.npy', '')
    stringList = f.split("-")
    type = stringList[2]
    test = int(stringList[1])
    if type == "depth":
        print(result)
        #3 mm 80 deg
        if test == 13 or test == 19 or test == 20:
            depth_a = np.concatenate((depth_a, result), axis =0)
            depth[0] = depth_a
        #6 mm 70
        elif test == 14 or test == 15 or test == 16:
            depth_b = np.concatenate((depth_b, result), axis =0)
            depth[1] = depth_b
        #3 mm 70
        elif test == 5 or test == 12 or test == 7:
            depth_c = np.concatenate((depth_c, result), axis =0)
            depth[2] = depth_c
        #6 mm 80
        elif test == 17 or test == 18:
            depth_d = np.concatenate((depth_d, result), axis =0)
            depth[3] = depth_d
    if type == "deviation":
        #3 mm 80 deg
        if test == 13 or test == 19 or test == 20:
            dev_a = np.concatenate((dev_a, result), axis =0)
            deviation[0] = dev_a
        #6 mm 70
        elif test == 14 or test == 15 or test == 16:
            dev_b = np.concatenate((dev_b, result), axis =0)
            deviation[1] = dev_b
        #3 mm 70
        elif test == 5 or test == 12 or test == 7:
            dev_c = np.concatenate((dev_c, result), axis =0)
            deviation[2] = dev_c
        #6 mm 80
        elif test == 17 or test == 18:
            dev_d = np.concatenate((dev_d, result), axis =0)
            deviation[3] = dev_d

deviation = np.array(deviation)
depth = np.array(depth)
for i in range(len(depth)):
    depth[i] = np.delete(depth[i], 0, 0)
for i in range(len(deviation)):
    deviation[i] = np.delete(deviation[i], 0, 0)
print(depth[1])
print(deviation.shape)
"""
plt.scatter(depth[:,1], depth[:,0], s= 0.5)
plt.scatter(depth[:,1]+1, depth[:,0], s=0.5)
plt.tight_layout()
plt.show()

plt.scatter(deviation[0][:,1], deviation[0][:,0], s= 0.5)
plt.scatter(deviation[0][:,1]+1, deviation[0][:,0], s=0.5)
plt.tight_layout()
plt.show()
"""
#deviation[0] = dev_a[0:dev_c.shape[0]]
#deviation[1] = dev_b[0:dev_c.shape[0]]
#deviation[2] = dev_c[0:dev_c.shape[0]]
#deviation[3] = dev_d[0:dev_c.shape[0]]
print(deviation[0].shape, deviation[1].shape, deviation[2].shape, deviation[3].shape)

#depth[0] = depth_a[0:dev_c.shape[0]]
#depth[1] = depth_b[0:depth_c.shape[0]]
#depth[2] = depth_c[0:depth_c.shape[0]]
#depth[3] = depth_d[0:depth_c.shape[0]]
print(depth[0].shape, depth[1].shape, depth[2].shape, depth[3].shape)
depth[0] = depth[0][~np.isnan(depth[0])]

print("Deviation:")
print("mean ---> a: ", round(np.mean(deviation[0]),3)," b: ", round(np.mean(deviation[1]),3), " c: ", round(np.mean(deviation[2]),3), " d: ", round(np.mean(deviation[3]),3))
print("median ---> a: ", round(np.median(deviation[0]),3)," b: ", round(np.median(deviation[1]),3), " c: ", round(np.median(deviation[2]),3), " d: ", round(np.median(deviation[3]),3))
print("std ---> a: ", round(np.std(deviation[0]),3)," b: ", round(np.std(deviation[1]),3), " c: ", round(np.std(deviation[2]),3), " d: ", round(np.std(deviation[3]),3))
print("max ---> a: ", round(np.amax(deviation[0]),3)," b: ", round(np.amax(deviation[1]),3), " c: ", round(np.amax(deviation[2]),3), " d: ", round(np.amax(deviation[3]),3))
print("min ---> a: ", round(np.min(deviation[0]),3)," b: ", round(np.min(deviation[1]),3), " c: ", round(np.min(deviation[2]),3), " d: ", round(np.min(deviation[3]),3))
print("99 range: ", round(np.percentile(deviation[0], 0.5),3), ":",round(np.percentile(deviation[0], 99.5),3)," b: ", round(np.percentile(deviation[1], 0.5),3), ":",round(np.percentile(deviation[1], 99.5),3), " c: ", round(np.percentile(deviation[2], 0.5),3), ":",round(np.percentile(deviation[2], 99.5),3), " d: ", round(np.percentile(deviation[3], 0.5),3), ":",round(np.percentile(deviation[3], 99.5),3))

print("Depth:")
print("mean ---> a: ", round(np.mean(depth[0]),3)," b: ", round(np.mean(depth[1]),3), " c: ", round(np.mean(depth[2]),3), " d: ", round(np.mean(depth[3]),3))
print("median ---> a: ", round(np.median(depth[0]),3)," b: ", round(np.median(depth[1]),3), " c: ", round(np.median(depth[2]),3), " d: ", round(np.median(depth[3]),3))
print("std ---> a: ", round(np.std(depth[0]),3)," b: ", round(np.std(depth[1]),3), " c: ", round(np.std(depth[2]),3), " d: ", round(np.std(depth[3]),3))
print("max ---> a: ", round(np.amax(depth[0]),3)," b: ", round(np.amax(depth[1]),3), " c: ", round(np.amax(depth[2]),3), " d: ", round(np.amax(depth[3]),3))
print("min ---> a: ", round(np.min(depth[0]),3)," b: ", round(np.min(depth[1]),3), " c: ", round(np.min(depth[2]),3), " d: ", round(np.min(depth[3]),3))
print("99 range: ", round(np.percentile(depth[0], 0.5),3), ":",round(np.percentile(depth[0], 99.5),3)," b: ", round(np.percentile(depth[1], 0.5),3), ":",round(np.percentile(depth[1], 99.5),3), " c: ", round(np.percentile(depth[2], 0.5),3), ":",round(np.percentile(depth[2], 99.5),3), " d: ", round(np.percentile(depth[3], 0.5),3), ":",round(np.percentile(depth[3], 99.5),3))
print("99 ", round(np.percentile(deviation[0],99),3))
print("98 ", round(np.percentile(deviation[0],98),3))
print("97 ", round(np.percentile(deviation[0],97),3))
print("96 ", round(np.percentile(deviation[0],96),3))
print("95 ", round(np.percentile(deviation[0],95),3))
print("94 ", round(np.percentile(deviation[0],94),3))
print("93 ", round(np.percentile(deviation[0],93),3))
print("92 ", round(np.percentile(deviation[0],92),3))
print("91 ", round(np.percentile(deviation[0],91),3))
print("90 ", round(np.percentile(deviation[0],90),3))
print("89 ", round(np.percentile(deviation[0],89),3))

print("0 ", round(np.percentile(deviation[0],0),3))
print("1 ", round(np.percentile(deviation[0],1),3))
print("2 ", round(np.percentile(deviation[0],2),3))
print("3 ", round(np.percentile(deviation[0],3),3))
print("4 ", round(np.percentile(deviation[0],4),3))
print("5 ", round(np.percentile(deviation[0],5),3))

data0a = {"V = 3 mm/s, A = 80 deg": deviation[0]}
data0b = {"V = 6 mm/s, A = 70 deg": deviation[1]}
data0c = {"V = 3 mm/s, A = 70 deg": deviation[2]}
data0d = {"V = 6 mm/s, A = 80 deg": deviation[3]}

data1a = {"V = 3 mm/s, A = 80 deg": depth[0]}
data1b = {"V = 6 mm/s, A = 70 deg": depth[1]}
data1c = {"V = 3 mm/s, A = 70 deg": depth[2]}
data1d = {"V = 6 mm/s, A = 80 deg": depth[3]}

df0a = pd.DataFrame(data=data0a)
df0b = pd.DataFrame(data=data0b)
df0c = pd.DataFrame(data=data0c)
df0d = pd.DataFrame(data=data0d)

df1a = pd.DataFrame(data=data1a)
df1b = pd.DataFrame(data=data1b)
df1c = pd.DataFrame(data=data1c)
df1d = pd.DataFrame(data=data1d)

fig, ax = plt.subplots(1, 4)
fig.suptitle("Deviation error test")

ax[0] = sns.boxplot(data=df0a, ax=ax[0])#, kind="box")
ax[0].set_ylabel("Error (mm)")
ax[0].hlines(-DEVIATION_LIMIT, -0.5, 0.5, colors = 'r',linestyles = 'dashed')
ax[0].hlines(DEVIATION_LIMIT, -0.5, 0.5,colors = 'r',linestyles = 'dashed')
ax[0].set_ylim(-1.25, 1.25)
#ax[0].set_title("Deviation error test")

ax[1] = sns.boxplot(data=df0b, ax=ax[1], color="orange")#), kind="box")
ax[1].hlines(-DEVIATION_LIMIT, -0.5, 0.5, colors = 'r',linestyles = 'dashed')
ax[1].hlines(DEVIATION_LIMIT, -0.5, 0.5,colors = 'r',linestyles = 'dashed')
ax[1].set_ylim(-1.25, 1.25)

ax[2] = sns.boxplot(data=df0c, ax=ax[2], color="green")#, kind="box")
ax[2].hlines(-DEVIATION_LIMIT, -0.5, 0.5, colors = 'r',linestyles = 'dashed')
ax[2].hlines(DEVIATION_LIMIT, -0.5, 0.5,colors = 'r',linestyles = 'dashed')
ax[2].set_ylim(-1.25, 1.25)

ax[3] = sns.boxplot(data=df0d, ax=ax[3], color="red")#), kind="box")
ax[3].hlines(-DEVIATION_LIMIT, -0.5, 0.5, colors = 'r',linestyles = 'dashed')
ax[3].hlines(DEVIATION_LIMIT, -0.5, 0.5,colors = 'r',linestyles = 'dashed')
ax[3].set_ylim(-1.25, 1.25)

fig, ax = plt.subplots(1, 4)
fig.suptitle("Indeptation test")

ax[0] = sns.boxplot(data=df1a, ax=ax[0])#, kind="box")
ax[0].set_ylabel("Depth (mm)")
ax[0].hlines(0.1, -0.5, 0.5, colors = 'r',linestyles = 'dashed')
ax[0].hlines(0.6, -0.5, 0.5,colors = 'r',linestyles = 'dashed')
ax[0].set_ylim(-0.1, 0.9)
#ax[0].set_title("Deviation error test")

ax[1] = sns.boxplot(data=df1b, ax=ax[1], color="orange")#), kind="box")
ax[1].hlines(0.1, -0.5, 0.5, colors = 'r',linestyles = 'dashed')
ax[1].hlines(0.6, -0.5, 0.5,colors = 'r',linestyles = 'dashed')
ax[1].set_ylim(-0.1, 0.9)

ax[2] = sns.boxplot(data=df1c, ax=ax[2], color="green")#, kind="box")
ax[2].hlines(0.1, -0.5, 0.5, colors = 'r',linestyles = 'dashed')
ax[2].hlines(0.6, -0.5, 0.5,colors = 'r',linestyles = 'dashed')
ax[2].set_ylim(-0.1, 0.9)

ax[3] = sns.boxplot(data=df1d, ax=ax[3], color="red")#), kind="box")
ax[3].hlines(0.1, -0.5, 0.5, colors = 'r',linestyles = 'dashed')
ax[3].hlines(0.6, -0.5, 0.5,colors = 'r',linestyles = 'dashed')
ax[3].set_ylim(-0.1, 0.9)

plt.show()
