# publish-and-record-multi-sensors

## 一、Motivation

本仓库不面向截取整段数据流任务(比如SLAM)，而面向多源传感器数据流中，截取部分数据的任务。比如非重复扫描固态激光雷达结合可见光图像的三维重建，一帧数据包括固定位姿下10s的积分点云以及一帧可见光图像。整个处理过程可分为以下三步：



<p align="center"><img src="./resources/flow_chart.png" width=50%></p>

<h6 align="center">流程图</h6>





## 二、命令行使用方法

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



## 三、客户端使用方法

````
pip install catkin-tools
pip install rospkg defusedxml
````



## 四、参考资料

[1] [python中调用launch文件](https://blog.csdn.net/bluewhalerobot/article/details/80952751)

[2] [python3运行ROS节点](https://blog.csdn.net/heroacool/article/details/118497979)

[3] [PyQT5订阅图像节点](https://blog.csdn.net/huoxingrenhdh/article/details/116753581)

[4] [Ros melodic python3环境解决cv_bridge问题](https://blog.csdn.net/weixin_42675603/article/details/107785376)

**备注:**`cv_bridge`是将image_msg转opencv image的库，但在python3中存在问题，需要重新编译(还是不行。。。)。

[5] [用自定义数据类型解决cv_bridge问题](https://blog.csdn.net/lizhiyuanbest/article/details/108022588)

[6] [发布和订阅自定义msg数组](https://blog.csdn.net/weixin_30847939/article/details/99386250?utm_medium=distribute.pc_relevant.none-task-blog-2~default~baidujs_baidulandingword~default-0.no_search_link&spm=1001.2101.3001.4242.1&utm_relevant_index=3)

