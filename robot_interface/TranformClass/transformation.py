import sys
import numpy as np
import struct
from scipy.interpolate import interp1d
from math import cos, sin, radians, pi
import re

# for debug
np.set_printoptions(threshold=sys.maxsize)


class transformation:
    # Specifying paths for the different files.
    def __init__(self):
        interval = 12  # ms
        self.PIT_Config = [-176.5, 335, 54, 180, 0, -90]
        self.scanner_Config = [167.80, -52, 55, -180, -90, 180]
        self.PIT_Config_KUKA60 = [-176.5, 335, 54, 180, 0, -90]
        self.scanner_Config_KUKA60 = [167.80, -52, 55, -180, -90, 180]
        self.n_points_ipo = 14
        self.n_points_io = 2
        self.scanning_speed = 10  # mm/s
        self.scanning_length = 50  # mm
        self.scanner_frequency = 10 / 0.3  # microseconds between  line
        self.data_frequency = 12  # ms logged data
        self.scan_offset = 0  # mm scanning offset, since we start scanning before we are moving.
        self.scan_end_offset = 30  # num of points to not include at the end of scan

    def reading_from_file(self, path_ipo, path_io):
        # opening the files
        file_kuka_ipo = open(path_ipo, "rb+")
        file_kuka_io = open(path_io, "rb+")
        raw_data_ipo = file_kuka_ipo.read()
        raw_data_io = file_kuka_io.read()

        # Parsing the raw data
        length_data_ipo = int(len(raw_data_ipo) / 8)
        length_data_io = int(len(raw_data_io) / 8)
        value_data_ipo = []
        value_data_io = []
        for i in range(0, length_data_ipo):
            unpack = struct.unpack('d', raw_data_ipo[i * 8:i * 8 + 8])[0]
            value_data_ipo.append(unpack)
        for i in range(0, length_data_io):
            unpack = struct.unpack('d', raw_data_io[i * 8:i * 8 + 8])[0]
            value_data_io.append(unpack)
        # print(len(value_data_ipo))
        # putting the data into matrices.

        self.data_value_ipo = np.array([[0] * self.n_points_ipo] * int(length_data_ipo / self.n_points_ipo),
                                       dtype=np.double)
        self.data_value_io = np.array([[0] * self.n_points_io] * int(length_data_io / self.n_points_io), dtype=np.int8)

        # for i in range(0,int(length_data_ipo/self.n_points_ipo)):
        for i in range(0, int(length_data_ipo / self.n_points_ipo)):
            for j in range(0, self.n_points_ipo):
                self.data_value_ipo[i][j] = value_data_ipo[i * self.n_points_ipo + j]

        for i in range(0, int(length_data_io / self.n_points_io)):
            for j in range(0, self.n_points_io):
                self.data_value_io[i][j] = value_data_io[i * self.n_points_io + j]

        file_kuka_ipo.close()
        file_kuka_io.close()
        return self.data_value_ipo, self.data_value_io

    def find_start_scan(self):
        self.scan_start = []
        for i in range(0, len(self.data_value_io)):
            if self.data_value_io[i][1] == 1:
                self.scan_start.append(i)
        self.scan_start_point = self.scan_start[0]

    def find_scan_coordinates(self):  # ,speed,length):
        # only taking the actual positions of the robot.
        self.trajectory = self.data_value_ipo[
                          self.scan_start_point:self.scan_start_point + self.scanning_speed * self.scanning_length - self.scan_end_offset,
                          6:12]
        return self.trajectory

    def interpolating_coordinates(self, scan_trajectory_len):
        num_points = len(self.trajectory)
        linspace_rob_traj = np.linspace(0, num_points * self.data_frequency, num_points)
        interpolate_functions0 = interp1d(linspace_rob_traj, self.trajectory[:, 0])
        interpolate_functions1 = interp1d(linspace_rob_traj, self.trajectory[:, 1])
        interpolate_functions2 = interp1d(linspace_rob_traj, self.trajectory[:, 2])
        interpolate_functions3 = interp1d(linspace_rob_traj, self.trajectory[:, 3])
        interpolate_functions4 = interp1d(linspace_rob_traj, self.trajectory[:, 4])
        interpolate_functions5 = interp1d(linspace_rob_traj, self.trajectory[:, 5])
        # interpolated_function = interp1d(linspace_rob_traj, self.trajectory[:,0])
        interpolated_points = np.zeros((scan_trajectory_len, len(self.trajectory[0])))
        print(scan_trajectory_len)
        for i in range(0, scan_trajectory_len):
            interpolated_points[i][0] = interpolate_functions0(i * self.scanner_frequency)
            interpolated_points[i][1] = interpolate_functions1(i * self.scanner_frequency)
            interpolated_points[i][2] = interpolate_functions2(i * self.scanner_frequency)
            interpolated_points[i][3] = interpolate_functions3(i * self.scanner_frequency)
            interpolated_points[i][4] = interpolate_functions4(i * self.scanner_frequency)
            interpolated_points[i][5] = interpolate_functions5(i * self.scanner_frequency)
        return interpolated_points

    # Function is currectly suited to poses logged directly from scanner(workvisual stuff)
    def calc_pose_global(self, local_poses, inter_points):
        matrix = []
        for i in range(0, len(local_poses)):
            matrix1 = self.matrix(
                [local_poses[i][0], local_poses[0][1] - self.scan_offset, local_poses[i][2], 0, 20, 0])
            matrix2 = self.matrix(
                [inter_points[i][0], inter_points[i][1], inter_points[i][2], inter_points[i][3] * 180 / pi,
                 inter_points[i][4] * 180 / pi, inter_points[i][5] * 180 / pi])
            # print(inter_points[i])
            matrix.append(np.dot(matrix2, matrix1))
            # print(np.dot(matrix2,matrix1))
        np_matrix_array = np.array(matrix)
        # print(len(np_matrix_array[0][1][1]))
        return np_matrix_array

    def calc_pose_pit_robodk(self, local_poses, angle):
        inter_p = self.interpolating_coordinates(len(local_poses))
        global_poses_matrix = self.calc_pose_global(local_poses, inter_p)
        curve_rob = np.zeros((len(global_poses_matrix), 6))
        for i in range(0, len(global_poses_matrix)):
            for j in range(0, 3):
                curve_rob[i][j] = global_poses_matrix[i][j][3]
            curve_rob[i][3] = 0
            cosa, sina = self.ang(90 - angle)
            curve_rob[i][4] = cosa * 100
            curve_rob[i][5] = -sina * 100
        return curve_rob

    def ang(self, angle):
        r = radians(angle)
        return cos(r), sin(r)

    def matrix(self, cart):
        dX = cart[0]
        dY = cart[1]
        dZ = cart[2]
        zC, zS = self.ang(cart[3])
        yC, yS = self.ang(cart[4])
        xC, xS = self.ang(cart[5])
        Translate_matrix = np.array([[1, 0, 0, dX],
                                     [0, 1, 0, dY],
                                     [0, 0, 1, dZ],
                                     [0, 0, 0, 1]])
        Rotate_X_matrix = np.array([[1, 0, 0, 0],
                                    [0, xC, -xS, 0],
                                    [0, xS, xC, 0],
                                    [0, 0, 0, 1]])
        Rotate_Y_matrix = np.array([[yC, 0, yS, 0],
                                    [0, 1, 0, 0],
                                    [-yS, 0, yC, 0],
                                    [0, 0, 0, 1]])
        Rotate_Z_matrix = np.array([[zC, -zS, 0, 0],
                                    [zS, zC, 0, 0],
                                    [0, 0, 1, 0],
                                    [0, 0, 0, 1]])
        return np.dot(Translate_matrix, np.dot(Rotate_Z_matrix, np.dot(Rotate_Y_matrix, Rotate_X_matrix)))

