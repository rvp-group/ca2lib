import argparse
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os


def draw_plane(ax, plane_nd: np.float32, size: (int, int) = (2, 2), color: str = "b") -> None:
    print(f"Drawing plane {plane_nd}")
    xx, yy = np.meshgrid(
        range(size[0]), range(size[1]))
    n = plane_nd[:3]
    d = plane_nd[-1]
    z = (-n[0] * xx - n[1] * yy - d) * 1. / n[2]

    ax.plot_surface(xx, yy, z, color=color)
    return


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="Visualize measurements obtained from calibrate_lidar_camera node")

    parser.add_argument(
        '--input_file',
        type=str,
        required=True)

    args = parser.parse_args()

    with open(args.input_file, 'r') as f:
        # <nx ny nz d, nx ny nz d>
        planes_cloud, planes_camera = zip(
            *[l.split(",") for l in f.readlines()])

        planes_cloud = np.asarray(
            list(map(lambda x: np.fromstring(x.strip(), sep=" "), planes_cloud)))

        planes_camera = np.asarray(
            list(map(lambda x: np.fromstring(x.strip(), sep=" "), planes_camera)))

    fig = plt.figure(figsize=(5, 5))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")

    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_zlim(-10, 10)

    for i in range(planes_cloud.shape[0]):
        draw_plane(ax, planes_cloud[i, :], size=(2, 2), color="b")
        draw_plane(ax, planes_camera[i, :], size=(2, 2), color='g')

    plt.show()

    exit(0)
