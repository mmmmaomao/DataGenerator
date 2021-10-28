## DateGenerator

**简介**

基于CARLA Simulator生成KITTI 2D/3D目标检测数据集格式的仿真数据集。除了对车辆和行人生成Label外，还对仿真环境中的20余种物体生成Label(树木、交通信号灯等）。

![image](https://user-images.githubusercontent.com/55339200/138204888-18958f52-ab1a-454a-8eef-23b7d4987f37.png)

**数据收集流程**

![image](https://user-images.githubusercontent.com/55339200/138204862-d4863c85-418b-4e4a-8db8-9efc7029635c.png)



**数据集格式**

training

|__   calib/    # 相机、雷达等传感器的矫正数据

|__   image/   # 相机产生的RGB图像

|__   label/   #  object 的标签

|__   velodyne/  #  激光雷达的测量数据

|__   train.txt

|__   trainval.txt

|__   val.txt

```
label：
#Values    Name      Description
----------------------------------------------------------------------------
   1    type         Describes the type of object: 'Car','Pedestrian',
   					 'TrafficSigns', etc.
   1    truncated    Float from 0 (non-truncated) to 1 (truncated), where
                     truncated refers to the object leaving image boundaries
   1    occluded     Integer (0,1,2,3) indicating occlusion state:
                     0 = fully visible, 1 = partly occluded
                     2 = largely occluded
   1    alpha        Observation angle of object, ranging [-pi..pi]
   4    bbox         2D bounding box of object in the image (0-based index):
                     contains left, top, right, bottom pixel coordinates
   3    dimensions   3D object dimensions: height, width, length (in meters)
   3    location     3D object location x,y,z in camera coordinates (in meters)
   1    rotation_y   Rotation ry around Y-axis in camera coordinates [-pi..pi]
   1    score        Only for results: Float, indicating confidence in
                     detection, needed for p/r curves, higher is better.
```

**label种类**

label标定的目标主要分为两类，第一类是我们自己生成的actors(Car 和 Pedestrian)；第二类是地图中存在的环境目标(None，Buildings，Fences，Other，Pedestrians，Poles，RoadLines，Roads，Sidewalks，TrafficSigns，Vegetation，Vehicles，Walls，Sky，Ground，Bridge，RailTrack，GuardRail，TrafficLight，Static，Dynamic，Water，Terrain)

**使用**

Carla版本：carla 0.9.12

```
python generator.py
```

SynchronyModel.py，场景类，负责建立client，设置server，生成actors，驱动server计算并获取数据

data_utils.py，包含点坐标转换、生成label等工具函数

data_descriptor.py, KITTI格式的描述类

DataSave.py，数据保存类，生成保存数据路径，保存数据

export_utils，保存数据的工具函数

image_converter.py, 图片格式转换函数

visual_utils，可视化工具函数

