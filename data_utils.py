import sys

import numpy as np
from numpy.linalg import inv

from config import cfg_from_yaml_file
from data_descriptor import KittiDescriptor, CarlaDescriptor
from image_converter import depth_to_array, to_rgb_array
import math
from visual_utils import draw_3d_bounding_box

sys.path.append("/opt/carla-simulator/PythonAPI/carla/dist/carla-0.9.12-py3.7-linux-x86_64.egg")

import carla

cfg = cfg_from_yaml_file("configs.yaml")

MAX_RENDER_DEPTH_IN_METERS = cfg["FILTER_CONFIG"]["MAX_RENDER_DEPTH_IN_METERS"]
MIN_VISIBLE_VERTICES_FOR_RENDER = cfg["FILTER_CONFIG"]["MIN_VISIBLE_VERTICES_FOR_RENDER"]
MAX_OUT_VERTICES_FOR_RENDER = cfg["FILTER_CONFIG"]["MAX_OUT_VERTICES_FOR_RENDER"]
WINDOW_WIDTH = cfg["SENSOR_CONFIG"]["DEPTH_RGB"]["ATTRIBUTE"]["image_size_x"]
WINDOW_HEIGHT = cfg["SENSOR_CONFIG"]["DEPTH_RGB"]["ATTRIBUTE"]["image_size_y"]


def objects_filter(data):
    environment_objects = data["environment_objects"]
    agents_data = data["agents_data"]
    actors = data["actors"]
    actors = [x for x in actors if x.type_id.find("vehicle") != -1 or x.type_id.find("walker") != -1]
    for agent, dataDict in agents_data.items():
        intrinsic = dataDict["intrinsic"]
        extrinsic = dataDict["extrinsic"]
        sensors_data = dataDict["sensor_data"]
        kitti_datapoints = []
        carla_datapoints = []
        rgb_image = to_rgb_array(sensors_data[0])
        image = rgb_image.copy()
        depth_data = sensors_data[1]

        data["agents_data"][agent]["visible_environment_objects"] = []
        for obj in environment_objects:
            kitti_datapoint, carla_datapoint = is_visible_by_bbox(agent, obj, image, depth_data, intrinsic, extrinsic)
            if kitti_datapoint is not None:
                data["agents_data"][agent]["visible_environment_objects"].append(obj)
                kitti_datapoints.append(kitti_datapoint)
                carla_datapoints.append(carla_datapoint)

        data["agents_data"][agent]["visible_actors"] = []

        for act in actors:
            kitti_datapoint, carla_datapoint = is_visible_by_bbox(agent, act, image, depth_data, intrinsic, extrinsic)
            if kitti_datapoint is not None:
                data["agents_data"][agent]["visible_actors"].append(act)
                kitti_datapoints.append(kitti_datapoint)
                carla_datapoints.append(carla_datapoint)

        data["agents_data"][agent]["rgb_image"] = image
        data["agents_data"][agent]["kitti_datapoints"] = kitti_datapoints
        data["agents_data"][agent]["carla_datapoints"] = carla_datapoints
    return data


def is_visible_by_bbox(agent, obj, rgb_image, depth_data, intrinsic, extrinsic):
    obj_transform = obj.transform if isinstance(obj, carla.EnvironmentObject) else obj.get_transform()
    obj_bbox = obj.bounding_box
    if isinstance(obj, carla.EnvironmentObject):
        vertices_pos2d = bbox_2d_from_agent(intrinsic, extrinsic, obj_bbox, obj_transform, 0)
    else:
        vertices_pos2d = bbox_2d_from_agent(intrinsic, extrinsic, obj_bbox, obj_transform, 1)
    depth_image = depth_to_array(depth_data)
    num_visible_vertices, num_vertices_outside_camera = calculate_occlusion_stats(vertices_pos2d, depth_image)
    if num_visible_vertices >= MIN_VISIBLE_VERTICES_FOR_RENDER and num_vertices_outside_camera < MAX_OUT_VERTICES_FOR_RENDER:
        obj_tp = obj_type(obj)
        midpoint = midpoint_from_agent_location(obj_transform.location, extrinsic)
        bbox_2d = calc_projected_2d_bbox(vertices_pos2d)
        rotation_y = get_relative_rotation_y(agent.get_transform().rotation, obj_transform.rotation) % math.pi
        ext = obj.bounding_box.extent
        truncated = num_vertices_outside_camera / 8
        if num_visible_vertices >= 6:
            occluded = 0
        elif num_visible_vertices >= 4:
            occluded = 1
        else:
            occluded = 2

        velocity = "0 0 0" if isinstance(obj, carla.EnvironmentObject) else\
            "{} {} {}".format(obj.get_velocity().x, obj.get_velocity().y, obj.get_velocity().z)
        acceleration = "0 0 0" if isinstance(obj, carla.EnvironmentObject) else \
            "{} {} {}".format(obj.get_acceleration().x, obj.get_acceleration().y, obj.get_acceleration().z)
        angular_velocity = "0 0 0" if isinstance(obj, carla.EnvironmentObject) else\
            "{} {} {}".format(obj.get_angular_velocity().x, obj.get_angular_velocity().y, obj.get_angular_velocity().z)
        # draw_3d_bounding_box(rgb_image, vertices_pos2d)

        kitti_data = KittiDescriptor()
        kitti_data.set_truncated(truncated)
        kitti_data.set_occlusion(occluded)
        kitti_data.set_bbox(bbox_2d)
        kitti_data.set_3d_object_dimensions(ext)
        kitti_data.set_type(obj_tp)
        kitti_data.set_3d_object_location(midpoint)
        kitti_data.set_rotation_y(rotation_y)

        carla_data = CarlaDescriptor()
        carla_data.set_type(obj_tp)
        carla_data.set_velocity(velocity)
        carla_data.set_acceleration(acceleration)
        carla_data.set_angular_velocity(angular_velocity)
        return kitti_data, carla_data
    return None, None

def obj_type(obj):
    if isinstance(obj, carla.EnvironmentObject):
        return obj.type
    else:
        if obj.type_id.find('walker') is not -1:
            return 'Pedestrian'
        if obj.type_id.find('vehicle') is not -1:
            return 'Car'
        return None

def get_relative_rotation_y(agent_rotation, obj_rotation):
    """ 返回actor和camera在rotation yaw的相对角度 """

    rot_agent = agent_rotation.yaw
    rot_car = obj_rotation.yaw
    return degrees_to_radians(rot_agent - rot_car)


def bbox_2d_from_agent(intrinsic_mat, extrinsic_mat, obj_bbox, obj_transform, obj_tp):
    bbox = vertices_from_extension(obj_bbox.extent)
    if obj_tp == 1:
        bbox_transform = carla.Transform(obj_bbox.location, obj_bbox.rotation)
        bbox = transform_points(bbox_transform, bbox)
    else:
        box_location = carla.Location(obj_bbox.location.x-obj_transform.location.x,
                                      obj_bbox.location.y-obj_transform.location.y,
                                      obj_bbox.location.z-obj_transform.location.z)
        box_rotation = obj_bbox.rotation
        bbox_transform = carla.Transform(box_location, box_rotation)
        bbox = transform_points(bbox_transform, bbox)
    # 获取bbox在世界坐标系下的点的坐标
    bbox = transform_points(obj_transform, bbox)
    # 将世界坐标系下的bbox八个点转换到二维图片中
    vertices_pos2d = vertices_to_2d_coords(bbox, intrinsic_mat, extrinsic_mat)
    return vertices_pos2d


def vertices_from_extension(ext):
    """ 以自身为原点的八个点的坐标 """
    return np.array([
        [ext.x, ext.y, ext.z],  # Top left front
        [- ext.x, ext.y, ext.z],  # Top left back
        [ext.x, - ext.y, ext.z],  # Top right front
        [- ext.x, - ext.y, ext.z],  # Top right back
        [ext.x, ext.y, - ext.z],  # Bottom left front
        [- ext.x, ext.y, - ext.z],  # Bottom left back
        [ext.x, - ext.y, - ext.z],  # Bottom right front
        [- ext.x, - ext.y, - ext.z]  # Bottom right back
    ])


def transform_points(transform, points):
    """ 作用：将三维点坐标转换到指定坐标系下 """
    # 转置
    points = points.transpose()
    # [[X0..,Xn],[Y0..,Yn],[Z0..,Zn],[1,..1]]  (4,8)
    points = np.append(points, np.ones((1, points.shape[1])), axis=0)
    # transform.get_matrix() 获取当前坐标系向相对坐标系的旋转矩阵
    points = np.mat(transform.get_matrix()) * points
    # 返回前三行
    return points[0:3].transpose()


def vertices_to_2d_coords(bbox, intrinsic_mat, extrinsic_mat):
    """将bbox在世界坐标系中的点投影到该相机获取二维图片的坐标和点的深度"""
    vertices_pos2d = []
    for vertex in bbox:
        # 获取点在world坐标系中的向量
        pos_vector = vertex_to_world_vector(vertex)
        # 将点的world坐标转换到相机坐标系中
        transformed_3d_pos = proj_to_camera(pos_vector, extrinsic_mat)
        # 将点的相机坐标转换为二维图片的坐标
        pos2d = proj_to_2d(transformed_3d_pos, intrinsic_mat)
        # 点实际的深度
        vertex_depth = pos2d[2]
        # 点在图片中的坐标
        x_2d, y_2d = pos2d[0], pos2d[1]
        vertices_pos2d.append((y_2d, x_2d, vertex_depth))
    return vertices_pos2d


def vertex_to_world_vector(vertex):
    """ 以carla世界向量（X，Y，Z，1）返回顶点的坐标 （4,1）"""
    return np.array([
        [vertex[0, 0]],  # [[X,
        [vertex[0, 1]],  # Y,
        [vertex[0, 2]],  # Z,
        [1.0]  # 1.0]]
    ])


def calculate_occlusion_stats(vertices_pos2d, depth_image):
    """ 作用：筛选bbox八个顶点中实际可见的点 """
    num_visible_vertices = 0
    num_vertices_outside_camera = 0

    for y_2d, x_2d, vertex_depth in vertices_pos2d:
        # 点在可见范围中，并且没有超出图片范围
        if MAX_RENDER_DEPTH_IN_METERS > vertex_depth > 0 and point_in_canvas((y_2d, x_2d)):
            is_occluded = point_is_occluded(
                (y_2d, x_2d), vertex_depth, depth_image)
            if not is_occluded:
                num_visible_vertices += 1
        else:
            num_vertices_outside_camera += 1
    return num_visible_vertices, num_vertices_outside_camera


def point_in_canvas(pos):
    if (pos[0] >= 0) and (pos[0] < WINDOW_HEIGHT) and (pos[1] >= 0) and (pos[1] < WINDOW_WIDTH):
        return True
    return False


def point_is_occluded(point, vertex_depth, depth_image):
    y, x = map(int, point)
    from itertools import product
    neigbours = product((1, -1), repeat=2)
    is_occluded = []
    for dy, dx in neigbours:
        if point_in_canvas((dy + y, dx + x)):
            # 判断点到图像的距离是否大于深对应深度图像的深度值
            if depth_image[y + dy, x + dx] < vertex_depth:
                is_occluded.append(True)
            else:
                is_occluded.append(False)
    # 当四个邻居点都大于深度图像值时，点被遮挡。返回true
    return all(is_occluded)


def midpoint_from_agent_location(location, extrinsic_mat):
    """ 将agent在世界坐标系中的中心点转换到相机坐标系下 """
    midpoint_vector = np.array([
        [location.x],  # [[X,
        [location.y],  # Y,
        [location.z],  # Z,
        [1.0]  # 1.0]]
    ])
    transformed_3d_midpoint = proj_to_camera(midpoint_vector, extrinsic_mat)
    return transformed_3d_midpoint


def camera_intrinsic(width, height):
    k = np.identity(3)
    k[0, 2] = width / 2.0
    k[1, 2] = height / 2.0
    f = width / (2.0 * math.tan(90.0 * math.pi / 360.0))
    k[0, 0] = k[1, 1] = f
    return k


def proj_to_camera(pos_vector, extrinsic_mat):
    """ 作用：将点的world坐标转换到相机坐标系中 """
    # inv求逆矩阵
    transformed_3d_pos = np.dot(inv(extrinsic_mat), pos_vector)
    return transformed_3d_pos


def proj_to_2d(camera_pos_vector, intrinsic_mat):
    """将相机坐标系下的点的3d坐标投影到图片上"""
    cords_x_y_z = camera_pos_vector[:3, :]
    cords_y_minus_z_x = np.concatenate([cords_x_y_z[1, :], -cords_x_y_z[2, :], cords_x_y_z[0, :]])
    pos2d = np.dot(intrinsic_mat, cords_y_minus_z_x)
    # normalize the 2D points
    pos2d = np.array([
        pos2d[0] / pos2d[2],
        pos2d[1] / pos2d[2],
        pos2d[2]
    ])
    return pos2d


def filter_by_distance(data_dict, dis):
    environment_objects = data_dict["environment_objects"]
    actors = data_dict["actors"]
    for agent,_ in data_dict["agents_data"].items():
        data_dict["environment_objects"] = [obj for obj in environment_objects if
                                            distance_between_locations(obj.transform.location, agent.get_location())
                                            <dis]
        data_dict["actors"] = [act for act in actors if
                                            distance_between_locations(act.get_location(), agent.get_location())<dis]


def distance_between_locations(location1, location2):
    return math.sqrt(pow(location1.x-location2.x, 2)+pow(location1.y-location2.y, 2))

def calc_projected_2d_bbox(vertices_pos2d):
    """ 根据八个顶点的图片坐标，计算二维bbox的左上和右下的坐标值 """
    legal_pos2d = list(filter(lambda x: x is not None, vertices_pos2d))
    y_coords, x_coords = [int(x[0][0]) for x in legal_pos2d], [
        int(x[1][0]) for x in legal_pos2d]
    min_x, max_x = min(x_coords), max(x_coords)
    min_y, max_y = min(y_coords), max(y_coords)
    return [min_x, min_y, max_x, max_y]

def degrees_to_radians(degrees):
    return degrees * math.pi / 180