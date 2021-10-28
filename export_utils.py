"""
This file contains all the methods responsible for saving the generated data in the correct output format.

"""

import numpy as np
from PIL import Image
import os
import logging
import math


def save_ref_files(OUTPUT_FOLDER, id):
    """ Appends the id of the given record to the files """
    for name in ['train.txt', 'val.txt', 'trainval.txt']:
        path = os.path.join(OUTPUT_FOLDER, name)
        with open(path, 'a') as f:
            f.write("{0:06}".format(id) + '\n')
        logging.info("Wrote reference files to %s", path)


def save_image_data(filename, image):
    logging.info("Wrote image data to %s", filename)
    image.save_to_disk(filename)

def save_bbox_image_data(filename, image):
    im = Image.fromarray(image)
    im.save(filename)

def save_lidar_data(filename, point_cloud, format="bin"):
    """ Saves lidar data to given filename, according to the lidar data format.
        bin is used for KITTI-data format, while .ply is the regular point cloud format
        In Unreal, the coordinate system of the engine is defined as, which is the same as the lidar points
        z
        ^   ^ x
        |  /
        | /
        |/____> y
              z
              ^   ^ x
              |  /
              | /
        y<____|/
        Which is a right handed coordinate sylstem
        Therefore, we need to flip the y axis of the lidar in order to get the correct lidar format for kitti.
        This corresponds to the following changes from Carla to Kitti
            Carla: X   Y   Z
            KITTI: X  -Y   Z
        NOTE: We do not flip the coordinate system when saving to .ply.
    """
    logging.info("Wrote lidar data to %s", filename)

    if format == "bin":
        point_cloud = np.copy(np.frombuffer(point_cloud.raw_data, dtype=np.dtype('f4')))
        point_cloud = np.reshape(point_cloud, (int(point_cloud.shape[0] / 4), 4))
        point_cloud = point_cloud[:, :-1]

        lidar_array = [[point[0], -point[1], point[2], 1.0]
                       for point in point_cloud]
        lidar_array = np.array(lidar_array).astype(np.float32)
        logging.debug("Lidar min/max of x: {} {}".format(
                      lidar_array[:, 0].min(), lidar_array[:, 0].max()))
        logging.debug("Lidar min/max of y: {} {}".format(
                      lidar_array[:, 1].min(), lidar_array[:, 0].max()))
        logging.debug("Lidar min/max of z: {} {}".format(
                      lidar_array[:, 2].min(), lidar_array[:, 0].max()))
        lidar_array.tofile(filename)


def save_label_data(filename, datapoints):
    with open(filename, 'w') as f:
        out_str = "\n".join([str(point) for point in datapoints if point])
        f.write(out_str)
    logging.info("Wrote kitti data to %s", filename)


def save_calibration_matrices(transform, filename, intrinsic_mat):
    """ Saves the calibration matrices to a file.
        AVOD (and KITTI) refers to P as P=K*[R;t], so we will just store P.
        The resulting file will contain:
        3x4    p0-p3      Camera P matrix. Contains extrinsic
                          and intrinsic parameters. (P=K*[R;t])
        3x3    r0_rect    Rectification matrix, required to transform points
                          from velodyne to camera coordinate frame.
        3x4    tr_velodyne_to_cam    Used to transform from velodyne to cam
                                     coordinate frame according to:
                                     Point_Camera = P_cam * R0_rect *
                                                    Tr_velo_to_cam *
                                                    Point_Velodyne.
        3x4    tr_imu_to_velo        Used to transform from imu to velodyne coordinate frame. This is not needed since we do not export
                                     imu data.
    """
    # KITTI format demands that we flatten in row-major order
    ravel_mode = 'C'
    P0 = intrinsic_mat
    P0 = np.column_stack((P0, np.array([0, 0, 0])))
    P0 = np.ravel(P0, order=ravel_mode)

    camera_transform = transform[0]
    lidar_transform = transform[1]
    # pitch yaw rool
    b = math.radians(lidar_transform.rotation.pitch-camera_transform.rotation.pitch)
    x = math.radians(lidar_transform.rotation.yaw-camera_transform.rotation.yaw)
    a = math.radians(lidar_transform.rotation.roll-lidar_transform.rotation.roll)
    R0 = np.identity(3)

    TR = np.array([[math.cos(b) * math.cos(x), math.cos(b) * math.sin(x), -math.sin(b)],
                    [-math.cos(a) * math.sin(x) + math.sin(a) * math.sin(b) * math.cos(x),
                     math.cos(a) * math.cos(x) + math.sin(a) * math.sin(b) * math.sin(x), math.sin(a) * math.cos(b)],
                    [math.sin(a) * math.sin(x) + math.cos(a) * math.sin(b) * math.cos(x),
                     -math.sin(a) * math.cos(x) + math.cos(a) * math.sin(b) * math.sin(x), math.cos(a) * math.cos(b)]])
    TR_velodyne = np.dot(TR, np.array([[1, 0, 0], [0, -1, 0], [0, 0, 1]]))

    TR_velodyne = np.dot(np.array([[0, 1, 0], [0, 0, -1], [1, 0, 0]]), TR_velodyne)

    '''
    TR_velodyne = np.array([[0, -1, 0],
                            [0, 0, -1],
                            [1, 0, 0]])
    '''
    # Add translation vector from velo to camera. This is 0 because the position of camera and lidar is equal in our configuration.
    TR_velodyne = np.column_stack((TR_velodyne, np.array([0, 0, 0])))
    TR_imu_to_velo = np.identity(3)
    TR_imu_to_velo = np.column_stack((TR_imu_to_velo, np.array([0, 0, 0])))

    def write_flat(f, name, arr):
        f.write("{}: {}\n".format(name, ' '.join(
            map(str, arr.flatten(ravel_mode).squeeze()))))

    # All matrices are written on a line with spacing
    with open(filename, 'w') as f:
        for i in range(4):  # Avod expects all 4 P-matrices even though we only use the first
            write_flat(f, "P" + str(i), P0)
        write_flat(f, "R0_rect", R0)
        write_flat(f, "Tr_velo_to_cam", TR_velodyne)
        write_flat(f, "TR_imu_to_velo", TR_imu_to_velo)
    logging.info("Wrote all calibration matrices to %s", filename)

def save_rgb_image(filename, image):
    im = Image.fromarray(image)
    im.save(filename)