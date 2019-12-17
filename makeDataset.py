import numpy as np
import sys
sys.path.insert(0, 'include/')
from pointCloud import PointCloud
from pointCloudLine import PointCloudLine
from trajectory import Trajectory
import open3d as o3d
import os
from tqdm import tqdm
import random

step_size = 50

annotations_file_path = "annotations/train/"

dataset_path = "dataset/"
positive_path = (dataset_path + "positive/")
negative_path = (dataset_path + "negative/")

for f in tqdm(os.listdir(annotations_file_path)):
    try:
        ind = np.loadtxt(os.path.join(annotations_file_path, f), dtype=int)
    except Exception as e:
        print("Failed to load annotations for: ", f)
    file = f.replace('.txt', '')
    try:
        pcd = PointCloud(np.load(("scans/before/processed/" + file +".npy")), type='NPARR')
    except Exception as e:
        print("Failed to load processed pcd numpy for: ", file)

    try:
        pcd = PointCloud(pcd.normalize(), type='NPARR')
        pcd.getPointCloudLines()
    except Exception as e:
        print("Failed to process pcd for: ", file)

    data_pos, data_neg = [], []
    for i in range(len(pcd.pointCloudLine)):
        pcl = PointCloudLine(pcd.pointCloudLine[i], 1)
        for k in range(len(ind)):

            edge, not_edge = [], []
            try:
                if ind[k, i] != 0:
                    for j in range(2*step_size+1):
                        index = (ind[k,i]+j)-step_size
                        edge.append([pcl.z[index]])
                        #edge.append([pcl.x[index]])
                    data_pos.append([np.array(edge), np.eye(2)[1]])

                    edge = []
                    for j in range(2*step_size+1):
                        index = (ind[k,i]-j)+step_size
                        edge.append([pcl.z[index]])
                        #edge.append([pcl.x[index]])
                    data_pos.append([np.array(edge), np.eye(2)[1]])
            except Exception as e:
                pass
            random_number = 0
            bool = True
            o = 0
            try:
                while bool == True:
                    random_number = random.randint(0, len(pcl.x))
                    difference = abs(random_number-ind[0, i])
                    if difference > abs(random_number-ind[1, i]):
                        difference = abs(random_number-ind[1, i])
                    if difference > (step_size*0.35):
                        bool = False
                    o += 1
                    if o > 50:
                        bool = False
            except Exception as e:
                print("Trouble with while loop.")



            for j in range(2*step_size+1):
                try:
                    index = (random_number+j)-step_size
                    not_edge.append([pcl.z[index]])
                except:
                    pass
            data_neg.append([np.array(not_edge), np.eye(2)[0]])
            not_edge = []
            for j in range(2*step_size+1):
                try:
                    index = (random_number-j)+step_size
                    not_edge.append([pcl.z[index]])
                except:
                    pass
            data_neg.append([np.array(not_edge), np.eye(2)[0]])
    data_pos = np.array(data_pos)
    data_neg = np.array(data_neg)
    try:
        np.save((positive_path + file + ".npy"), data_pos)
    except Exception as e:
        print(file, " positive.")
    try:
        np.save((negative_path + file + ".npy"), data_neg)
    except Exception as e:
        print(file, " negative.")

    print("No. of positve samples: ", len(data_pos))
    print("No. of negative samples: ", len(data_neg))
