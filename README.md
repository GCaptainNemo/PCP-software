# publish-and-record-multi-sensors

## 一、Motivation

本仓库不面向截取整段数据流任务(比如SLAM)，而面向多源传感器数据流中，截取部分数据的任务。比如非重复扫描固态激光雷达结合可见光图像的三维重建，一帧数据包括固定位姿下10s的积分点云以及一帧可见光图像。整个处理过程可分为以下三步：



<p align="center"><img src="./resources/flow_chart.png" width=50%></p>

<h6 align="center">流程图</h6>





## 使用方法

### 1. 发布数据

```
roslaunch collect_data publish_data.launch
```

### 2. 记录一帧数据

```
python3 modify_name.py
roslaunch collect_data collect_data.launch
```

or

