from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from pytransform3d import rotations, transformations
import pytransform3d.rotations as pr
import pytransform3d.visualizer as pv
import sys
import argparse

def read_planes_from_file(file_path):
    planes = []
    with open(file_path, 'r') as file:
        for line in file:
            # data = [float(x) for x in line.strip().split(",")]
            # planes.append((data[:4], data[4:]))
            data0 = line.strip().split(',')[0]
            data1 = line.strip().split(',')[1]
            planes.append((np.array([float(x) for x in data0.strip().split()]), np.array([float(x) for x in data1.strip().split()])))
    return planes

def plot_planes(planes):
    fig = pv.figure()
    fig.plot_transform(np.eye(4))
    random_matrix = np.random.rand(3, len(planes))
    index = 0
    for plane in planes:
        normal, distance = plane[0][:3], plane[0][3] + 5
        point_in_plane = normal * distance
        fig.plot_plane(normal,distance, c = random_matrix[:,index])
        fig.plot_vector(start=point_in_plane,direction=normal)
        # fig.plot_plane(normal,distance, c = (1,0,0)) 
        index +=1
    index = 0
    for plane in planes:
        normal, distance = plane[1][:3], plane[1][3] + 5
        point_in_plane = normal * distance
        fig.plot_plane(normal,distance, c = random_matrix[:,index])
        fig.plot_vector(start=point_in_plane,direction=normal)
        # fig.plot_plane(normal,distance, c = (0,0,1)) 
        index +=1
    fig.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Visualize measurements obtained from calibrate_lidar_camera node")

    parser.add_argument(
        '--input_file',
        type=str,
        required=True)

    args = parser.parse_args()
    planes = read_planes_from_file(args.input_file)
    plot_planes(planes)
