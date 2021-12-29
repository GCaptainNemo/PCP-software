# publish-and-record-multi-sensors

## 一、Motivation

本仓库不面向截取整段数据流任务(比如SLAM)，而面向多源传感器数据流中，截取部分数据的任务。比如非重复扫描固态激光雷达结合可见光图像的三维重建，一帧数据包括固定位姿下10s的积分点云以及一帧可见光图像。整个处理过程可分为以下三步：



<p align="center"><img src="./resources/flow_chart.png" width=50%></p>

<h6 align="center">流程图</h6>





## 二、使用方法

### 1. 发布数据

```
roslaunch collect_data publish_data.launch
```

### 2. 记录数据

##### 2.1 更改launch文件

**备注:**更改collect_data.launch中的文件保存路径，collect_calib.launch中的文件前缀

```
python3 modify_name.py
```

##### 2.2 抓取激光雷达、可见光、热红外数据

```
roslaunch collect_data collect_data.launch
```

##### 2.3 抓取可见光、热红外数据(用来标定热红外、可见光相机外参)

```
roslaunch collect_data collect_calib.launch
python3 modify_calib_name.py
```

or

```
sh ./collect_calib.sh
```

