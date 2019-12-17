import sys
sys.path.insert(0, 'include/')

import copy
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
import heapq
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
import os


FILE_NAME = "11-12-19-qual5"
LOAD_PCD = False
LOAD_ROI = False
xOff_l = -2.5
xOff_r = -1.5

with open("config.yaml", 'r') as ymlfile:
    cfg = yaml.load(ymlfile)

net = loadClassifier()

print("Loading point cloud...")

pcd = pcdLoader(file_name = FILE_NAME, afterPIT=True, load=LOAD_PCD)

pcd.pcd = o3d.geometry.PointCloud.remove_statistical_outlier(pcd.pcd, 1000, 1)[0]

pcd.getPointCloudLines() # Construct arrays of point cloud lines
npArr = pcd.size_threshold_pointCloudLine(threshold = 0.8)
pcd.pcd_from_pointCloudLines(npArr)
img = pcd.pcd_to_image()
npArr = pcd.img_to_pcd(img)

# Normalize point cloud
pcdNormalized = PointCloud(pcd.normalize(), type ='NPARR')
pcdNormalized.getPointCloudLines() # Construct arrays of point cloud lines

if LOAD_ROI == True:
    if os.path.exists((cfg['after_file'].get("ROI") + FILE_NAME + cfg['after_file'].get("ext_npy"))) == True:
        ROI = roiLoader(file_name = (cfg['after_file'].get("ROI") + FILE_NAME + cfg['after_file'].get("ext_npy")))
    else:
        print("Couldn't load regions of interest.")
else:
    ROI = getROI(pcdNormalized, net=net)

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
weld_toe = []
pcd.unNormalize()

for mask in masks:
    maskLines = ROI.maskToLines(mask)
    npArr = pcdNormalized.mask_pcd(maskLines, border = cfg['model_after'].get("step_size"))
    if len(npArr) != 0:
        weld_toe.append(Trajectory(npArr))

weld_toe[0].unNormalize(pcd.z_max, pcd.z_min)
weld_toe[0].flip()
weld_toe[0].setColor('r')
weld_toe[1].unNormalize(pcd.z_max, pcd.z_min)
weld_toe[1].flip()
weld_toe[1].setColor('g')
pcd.flip()
#
#o3d.visualization.draw_geometries([pcd.pcd, weld_toe[0].pcd, weld_toe[1].pcd])
#o3d.visualization.draw_geometries([pcd.pcd, weld_toe[0].pcd]) #left
#o3d.visualization.draw_geometries([pcd.pcd, weld_toe[1].pcd])

pcd.getPointCloudLines()
weld_toe[0].getPointCloudLines()
weld_toe[1].getPointCloudLines()
raw_weldtoe_left = copy.deepcopy(weld_toe[0].pointCloudLine)
raw_weldtoe_right = copy.deepcopy(weld_toe[1].pointCloudLine)
#o3d.visualization.draw_geometries([pcd.pcd, weld_toe[0].pcd])
#o3d.visualization.draw_geometries([pcd.pcd, weld_toe[1].pcd])

# Karls rocker algorithm from here.

peenDepth_left = []
peenDepth_right = []
peenRadius_left = []
peenRadius_right = []
peenWidth_left = []
peenWidth_right = []
trajectpredict = []
deviation_l, deviation_r = [], []
line=[]
perpline_left = []
perpline_right = []
intercept_left = []
intercept_right = []
for i in range(len(weld_toe[0].pointCloudLine)):

    try:
        print(i)

        quality_left = Quality(weld_toe[0].pointCloudLine[i], pcd.pointCloudLine[i])
        quality_right = Quality(weld_toe[1].pointCloudLine[i], pcd.pointCloudLine[i])
        quality_left.get_derivative()
        quality_left.derivative_min_max()
        quality_left.find_minmax_curvature()
        peenWidth_left.append([quality_left.get_width_and_midpoint(),quality_left.curvature[0][1]])
        deviation_l.append([quality_left.find_traject(xOff_l, file = FILE_NAME), quality_left.curvature[0][1]])
        #line.append(quality_left.widthline()
        perpline_left.append(quality_left.pline())
        intercept_left.append(quality_left.find_intercept(quality_left.pline()))
        peenDepth_left.append([quality_left.get_depth(),quality_left.curvature[0][1]])

        #plt.plot(weld_toe[0].pointCloudLine[i][:,0],weld_toe[0].pointCloudLine[i][:,2])
        #plt.plot(weld_toe[1].pointCloudLine[i][:,0],weld_toe[1].pointCloudLine[i][:,2])
        #plt.show()
        #plt.plot(quality_left.smooth_array[50][0],quality_left.smooth_array[50][2])
        #plt.plot(quality_left.curvature[:,0], quality_left.curvature[:,2])
        #plt.plot(quality_left.midpoint[0], quality_left.midpoint[2], 'ro')



        quality_right.get_derivative()
        quality_right.derivative_min_max()
        quality_right.find_minmax_curvature()
        peenWidth_right.append([quality_right.get_width_and_midpoint(),quality_right.curvature[0][1]])
        deviation_r.append([quality_right.find_traject(xOff_r, file = FILE_NAME), quality_right.curvature[0][1]])
        perpline_right.append(quality_right.pline())
        #intercept_right.append(quality_right.find_intercept(quality_right.pline()))

        circle_fit_left = quality_left.leastsq_circle(quality_left.find_minmax_curvature()[:,0], quality_left.find_minmax_curvature()[:,2])
        peenRadius_left.append([circle_fit_left[2],i])
        #quality_left.plot_data_circle(quality_left.find_minmax_curvature()[:,0], quality_left.find_minmax_curvature()[:,2], stuff_left[0], stuff_left[1], stuff_left[2])
        #quality_left.plot_stuff(weld_toe[0].pointCloudLine[i])
            #trajectpredict.append(quality_left.find_traject()[0])
        intercept_right.append(quality_right.find_intercept(quality_right.pline()))
        peenDepth_right.append([quality_right.get_depth(),quality_right.curvature[0][1]])
        circle_fit_right = quality_right.leastsq_circle(quality_right.find_minmax_curvature()[:,0], quality_right.find_minmax_curvature()[:,2])
        peenRadius_right.append([circle_fit_right[2],i])
        #quality_right.plot_data_circle(quality_right.find_minmax_curvature()[:,0], quality_right.find_minmax_curvature()[:,2], stuff_right[0], stuff_right[1], stuff_right[2])
        #quality_right.plot_stuff(weld_toe[1].pointCloudLine[i])

    except Exception as e:
        pass
        print(e)


peenDepth_left = np.array(peenDepth_left)
peenDepth_right = np.array(peenDepth_right)
peenRadius_left = np.array(peenRadius_left)
peenRadius_right = np.array(peenRadius_right)
peenWidth_left = np.array(peenWidth_left)
peenWidth_right = np.array(peenWidth_right)
trajectpredict = np.array(trajectpredict)
deviation_l = np.array(deviation_l)
deviation_r = np.array(deviation_r)
fig, axs = plt.subplots(3, 1, constrained_layout=True)
axs[0].set_title('Depth Left Side')
axs[0].plot(peenDepth_left[:][:,1], quality_left.moving_Avg(peenDepth_left[:][:,0],5))#quality_right.smooth2(20, len(peenDepth_left[:][:,0]),peenDepth_left[:][:,0]))
axs[0].hlines(0.1, 0, peenDepth_left[-1][1],colors = 'r', linestyles = 'dashed')
axs[0].hlines(0.6, 0, peenDepth_left[-1][1],colors = 'r', linestyles = 'dashed')
axs[0].set(ylabel='Depth (mm)')
axs[0].grid()
#axs[1].set_title('Depth right side')
#axs[1].plot(peenDepth_right[:][:,1], quality_right.smooth_outliers(peenDepth_right[:][:,0],5))
#axs[1].hlines(0.1, 0, len(peenDepth_left[:][:,1]),colors = 'r',linestyles = 'dashed')
#axs[1].hlines(0.6, 0, len(peenDepth_left[:][:,1]),colors = 'r',linestyles = 'dashed')

#axs[1].set_title('Radius left side')
#axs[1].plot(peenRadius_left[:][:,1], quality_left.smooth_outliers(peenRadius_left[:][:,0],5))
#axs[1].grid()

#axs[3].set_title('Radius right side')
#axs[3].plot(peenRadius_right[:][:,1],quality_right.smooth_outliers(peenRadius_right[:][:,0],5))
#axs[4].set_title('Width left side')
axs[1].set_title('Width Left Side')
axs[1].plot(peenWidth_left[:][:,1], quality_right.smooth_outliers(peenWidth_left[:][:,0],5))
#axs[1].hlines(3, 0, peenDepth_left[-1][1],colors = 'r',linestyles = 'dashed')
#axs[1].hlines(6, 0, peenDepth_left[-1][1],colors = 'r',linestyles = 'dashed')
axs[1].set(ylabel='Width (mm)')
axs[1].grid()
#axs[5].set_title('Width right side')
#axs[5].plot(peenWidth_right[:][:,1], peenWidth_right[:][:,0])
#axs[5].hlines(3, 0, len(peenDepth_left[:][:,1]),colors = 'r',linestyles = 'dashed')
#axs[5].hlines(6, 0, len(peenDepth_left[:][:,1]),colors = 'r',linestyles = 'dashed')

axs[2].set_title('Trajectory Deviation Left')
axs[2].plot(deviation_l[:][:,1], quality_right.smooth_outliers(deviation_l[:][:,0],5))#quality_right.smooth2(20, len(deviation_l[:][:,0]),deviation_l[:][:,0]))
axs[2].hlines(-0.7121, 0, peenDepth_left[-1][1],colors = 'r',linestyles = 'dashed')
axs[2].hlines(0.7121, 0, peenDepth_left[-1][1],colors = 'r',linestyles = 'dashed')
axs[2].set(xlabel='y-value (mm)', ylabel='Error (mm)')
axs[2].grid()

fig, axs = plt.subplots(3, 1, constrained_layout=True)
axs[0].set_title('Depth Right Side')
axs[0].plot(peenDepth_right[:][:,1], quality_right.smooth_outliers(peenDepth_right[:][:,0],5))
axs[0].hlines(0.1, 0, peenDepth_left[-1][1],colors = 'r', linestyles = 'dashed')
axs[0].hlines(0.6, 0, peenDepth_left[-1][1],colors = 'r', linestyles = 'dashed')
axs[0].set(ylabel='Depth (mm)')
axs[0].grid()
#axs[1].set_title('Depth right side')
#axs[1].plot(peenDepth_right[:][:,1], quality_right.smooth_outliers(peenDepth_right[:][:,0],5))
#axs[1].hlines(0.1, 0, len(peenDepth_left[:][:,1]),colors = 'r',linestyles = 'dashed')
#axs[1].hlines(0.6, 0, len(peenDepth_left[:][:,1]),colors = 'r',linestyles = 'dashed')

#axs[1].set_title('Radius right side')
#axs[1].plot(peenRadius_right[:][:,1], quality_right.smooth_outliers(peenRadius_right[:][:,0],5))
#axs[1].grid()

#axs[3].set_title('Radius right side')
#axs[3].plot(peenRadius_right[:][:,1],quality_right.smooth_outliers(peenRadius_right[:][:,0],5))
#axs[4].set_title('Width left side')

axs[1].set_title('Width Right Side')
axs[1].plot(peenWidth_right[:][:,1], quality_right.smooth_outliers(peenWidth_right[:][:,0],5))
#axs[1].hlines(3, 0, peenDepth_left[-1][1],colors = 'r',linestyles = 'dashed')
#axs[1].hlines(6, 0, peenDepth_left[-1][1],colors = 'r',linestyles = 'dashed')
axs[1].set(ylabel='Width (mm)')
axs[1].grid()
#axs[5].set_title('Width right side')
#axs[5].plot(peenWidth_right[:][:,1], peenWidth_right[:][:,0])
#axs[5].hlines(3, 0, len(peenDepth_left[:][:,1]),colors = 'r',linestyles = 'dashed')
#axs[5].hlines(6, 0, len(peenDepth_left[:][:,1]),colors = 'r',linestyles = 'dashed')

axs[2].set_title('Trajectory Deviation Right')
axs[2].plot(deviation_r[:][:,1], quality_right.smooth_outliers(deviation_r[:][:,0],5))
axs[2].hlines(-0.7349, 0, peenDepth_left[-1][1],colors = 'r',linestyles = 'dashed')
axs[2].hlines(0.7349, 0, peenDepth_left[-1][1],colors = 'r',linestyles = 'dashed')
axs[2].set(xlabel='y-value (mm)',ylabel='Error (mm)')
axs[2].grid()


#axs[7].set_title('Trajectory deviation right')
#axs[7].plot(deviation_r[:][:,1], deviation_r[:][:,0])
#axs[7].hlines(-0.75, 0, len(deviation_r[:][:,1]),colors = 'r',linestyles = 'dashed')
#axs[7].hlines(0.75, 0, len(deviation_r[:][:,1]),colors = 'r',linestyles = 'dashed')

plt.show()


for i in range(len(weld_toe[1].pointCloudLine)):
    print(i)
    #print(pcd.pointCloudLine[i][quality_left.find_traject(),0])
    #print(pcd.pointCloudLine[i][qualit-0.75y_left.find_traject(),2])
    #plt.plot(weld_toe[0].pointCloudLine[0],weld_toe[0].pointCloudLine[2])
    #plt.plot(pcd.pointCloudLine[i][:,0],pcd.pointCloudLine[i][:,2])
    #plt.plot(pcd.pointCloudLine[i][:,0], intercept[i][1], 'ro')
    #plt.plot(quality_left.widthline()[0],quality_left.widthline()[1])
    #plt.plot(pcd.pointCloudLine[i][0],pcd.pointCloudLine[i][2])
    #plt.plot(weld_toe[1].pointCloudLine[i][:,0],weld_toe[1].pointCloudLine[i][:,2])
    try:
        #plt.plot(quality_left.smooth_array[i][:,0],quality_left.smooth_array[i][:,2])

        #plt.plot(trajectzvalue[i][2],trajectzvalue[i][1], "ro")
        #plt.plot(trajectzvalue[i][3],trajectzvalue[i][4], "ro")
        #plt.plot(trajectzvalue[i][5],trajectzvalue[i][6], "ro")
        plt.plot(line[i][0],line[i][1])
        #plt.plot(perpline_left[i][0],perpline_left[i][1])
        #plt.plot(intercept_left[i][0],intercept_left[i][1], 'ro')
        print("x:", intercept_left[i][0], "z:", intercept_left[i][1])
        #plt.plot(perpline_right[i][0],perpline_right[i][1])
        #plt.plot(intercept_right[i][0],intercept_right[i][1], 'ro')
    except Exception as e:
        print(e)
        pass
    print("Peen depth left:", peenDepth_left[i][0])
    print("Peen depth right:", peenDepth_right[i][0])
    print("Radius left:", peenRadius_left[i][0])
    print("Radius right:", peenRadius_right[i][0])
    print("Width left:", peenWidth_left[i][0])
    print("Width right:", peenWidth_right[i][0])
    print("  ")
    plt.show()


#pdf = pdfprint()
