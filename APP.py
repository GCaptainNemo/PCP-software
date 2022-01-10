#!/usr/bin/env python3
# coding=UTF-8
import os

import sys
import numpy as np
import xml.etree.ElementTree as ET
import re
import roslaunch
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from PyQt5.QtCore import QThread, pyqtSignal,pyqtSlot
from PyQt5 import QtGui, QtCore, QtWidgets
import qdarkstyle

from collect_data.msg import self_image

# bridge = CvBridge()

COLLECT_DATA_LAUNCH_ADDR = "/home/why/ROS_self/collect_data/src/collect_data/launch/collect_data.launch"
COLLECT_CALIB_LAUNCH_ADDR = "/home/why/ROS_self/collect_data/src/collect_data/launch/collect_calib.launch"
PUBLISH_LAUNCH_ADDR = "/home/why/ROS_self/collect_data/src/collect_data/launch/publish_raw_data.launch"
START_PUBLISH_FLAG = 0

        

class CameraRosNode(QThread):
    rgb_image_signal = pyqtSignal() 
    ir_image_signal = pyqtSignal() 
    def __init__(self):
        QThread.__init__(self)  #创建线程
        rospy.init_node('camera_ros_node') #创建ros节点
        rospy.Subscriber('/rgb_remap', self_image, self.callback_rgb_img)
        rospy.Subscriber('/ir_remap', self_image, self.callback_ir_img)
        self.ir_img = None
        self.rgb_img = None

    def callback_ir_img(self, data):
        self.ir_img = np.ndarray(shape=(data.height, data.width, data.channels), dtype=np.uint8, buffer=data.data) # 将自定义图像消息转化为图像
        self.ir_image_signal.emit()

    def callback_rgb_img(self, data):
        self.rgb_img = np.ndarray(shape=(data.height, data.width, data.channels), dtype=np.uint8, buffer=data.data) # 将自定义图像消息转化为图像
        self.rgb_image_signal.emit()
        # cv2.imshow("rgb", image)
        # cv2.waitKey(0)
        # self.raw_image_signal.emit()

    def run(self):
        #  spin()后才能进入callback函数，spin后的程序不能执行，除非跳出
        rospy.spin()    

class ThreadStartScribe(QtCore.QThread):
    def __init__(self):
        super(ThreadStartScribe, self).__init__()

    def rgb_callback(self, data):
        cv_image = bridge.imgmsg_to_cv2(data,"bgr8")
        cv2.imshow("lala",cv_image)
        cv2.waitKey(0)

    def run(self):
        rospy.init_node('showImages',anonymous = True)
        rospy.Subscriber('/hik_cam_node/hik_camera', Image, self.rgb_callback)
        rospy.spin()
        


class MyWinPicture(QtWidgets.QWidget):
    def __init__(self):
        """
        Rewrite Qwidget to display pictures.
        param flag_:
        Two mode: 1. flag_ = 1, display through pictures address(self.dir)
                  2. flag_ = 0 display through pictures (self.picture_matrix)
        """
        super(MyWinPicture, self).__init__()
        self.pixmap = None

    def paintEvent(self, event):
        try:
            if self.pixmap is not None:
                painter = QtGui.QPainter(self)
                painter.drawPixmap(self.rect(), self.pixmap)
        except Exception as e:
            print(e)


class MainWindow(QtWidgets.QWidget):
    def __init__(self) -> None:
        super(MainWindow, self).__init__()
        VSplitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        # VSplitter = QtWidgets.QSplitter(QtCore.Qt.Horizon)

        self.setWindowTitle("PAC")   # publish and collect data
        self.rgb_widget = MyWinPicture()
        self.ir_widget = MyWinPicture()
        VSplitter.addWidget(self.rgb_widget)
        VSplitter.addWidget(self.ir_widget)
        # ###############################################################3
        Hlayout_1 = QtWidgets.QHBoxLayout()
        self.btn_start_publish = QtWidgets.QPushButton('Start Publish')
        self.btn_stop_publish = QtWidgets.QPushButton('Stop Publish')
        self.btn_start_collect = QtWidgets.QPushButton('Collect')
        self.btn_start_calib = QtWidgets.QPushButton('Calib')
        Hlayout_1.addWidget(self.btn_start_publish)
        Hlayout_1.addWidget(self.btn_stop_publish)

        Hlayout_1.addWidget(self.btn_start_collect)
        Hlayout_1.addWidget(self.btn_start_calib)
        self.publish_launch = None
        self.btn_start_publish.clicked.connect(self.start_publish)
        self.btn_stop_publish.clicked.connect(self.stop_publish)

        self.btn_start_collect.clicked.connect(self.start_collect_data)
        self.btn_start_calib.clicked.connect(self.start_collect_calib)

        # ################################################################
        Hlayout_2 = QtWidgets.QHBoxLayout()
        self.label_current_prefix = QtWidgets.QLabel("prefix: ")
        self.linedit_current_prefix = QtWidgets.QLineEdit()
        self.btn_get_current_prefix = QtWidgets.QPushButton("Current Prefix")        
        self.btn_next_prefix = QtWidgets.QPushButton("Next Prefix")
        self.btn_set_prefix = QtWidgets.QPushButton("Set Prefix")
        self.btn_get_current_prefix.clicked.connect(self.get_current_prefix)
        self.btn_next_prefix.clicked.connect(self.get_next_prefix)
        self.btn_set_prefix.clicked.connect(self.set_prefix)

        
        Hlayout_2.addWidget(self.label_current_prefix)
        Hlayout_2.addWidget(self.linedit_current_prefix)
        Hlayout_2.addWidget(self.btn_get_current_prefix)
        Hlayout_2.addWidget(self.btn_next_prefix)
        Hlayout_2.addWidget(self.btn_set_prefix)
        # ################################################################
        Hlayout_3 = QtWidgets.QHBoxLayout()
        self.label_current_savedir = QtWidgets.QLabel("save dir: ")
        self.linedit_current_savedir = QtWidgets.QLineEdit()
        self.btn_get_current_savedir = QtWidgets.QPushButton("Current Savedir")  
        self.btn_browse = QtWidgets.QPushButton("Browse")  
        self.btn_set_savedir = QtWidgets.QPushButton("Set Savedir")
        Hlayout_3.addWidget(self.label_current_savedir)
        Hlayout_3.addWidget(self.linedit_current_savedir)
        Hlayout_3.addWidget(self.btn_get_current_savedir)
        Hlayout_3.addWidget(self.btn_browse)
        Hlayout_3.addWidget(self.btn_set_savedir)

        self.btn_get_current_savedir.clicked.connect(self.get_current_savedir)
        self.btn_browse.clicked.connect(self.browse)
        self.btn_set_savedir.clicked.connect(self.set_savedir)
        # ################################################################
        Hlayout_4 = QtWidgets.QHBoxLayout()
        self.label_current_record_time = QtWidgets.QLabel("Record time(s): ")
        self.linedit_current_record_time = QtWidgets.QLineEdit()
        self.btn_get_current_record_time = QtWidgets.QPushButton("Current record time")
        self.btn_set_record_time = QtWidgets.QPushButton("Set record time")
        Hlayout_4.addWidget(self.label_current_record_time)
        Hlayout_4.addWidget(self.linedit_current_record_time)
        Hlayout_4.addWidget(self.btn_get_current_record_time)
        Hlayout_4.addWidget(self.btn_set_record_time)
        self.btn_get_current_record_time.clicked.connect(self.get_current_recordtime)
        self.btn_set_record_time.clicked.connect(self.set_recordtime)

   
        # ################################################################        
        vlayout = QtWidgets.QVBoxLayout(self)
        vlayout.addWidget(VSplitter)
        vlayout.addLayout(Hlayout_1)
        vlayout.addLayout(Hlayout_2)
        vlayout.addLayout(Hlayout_3)
        vlayout.addLayout(Hlayout_4)


        # ##############################################################
        # ros node subscribe
        # ##############################################################
        self.subscribe_thread = CameraRosNode()
        self.subscribe_thread.rgb_image_signal.connect(self.update_rgb_img)
        self.subscribe_thread.ir_image_signal.connect(self.update_ir_img)

     
    def update_ir_img(self):
        """
        slot function
        """
        new_img = self.subscribe_thread.ir_img.copy()
        dst_img = QtGui.QImage(new_img, new_img.shape[1],
                                 new_img.shape[0], QtGui.QImage.Format_Grayscale)
        self.ir_widget.pixmap = QtGui.QPixmap.fromImage(dst_img)
        self.ir_widget.repaint()

    def update_rgb_img(self):
        """
        slot function
        """
        print("in update rgb img")
        new_img = self.subscribe_thread.rgb_img.copy()
        new_img = cv2.cvtColor(new_img, cv2.COLOR_BGR2RGB)
        # new_img = new_img[:, :, ::-1]
        dst_img = QtGui.QImage(new_img, new_img.shape[1],
                                 new_img.shape[0], QtGui.QImage.Format_RGB888)
        self.rgb_widget.pixmap = QtGui.QPixmap.fromImage(dst_img)
        self.rgb_widget.repaint()
       

    def rgb_callback(self, data):
        cv_image = bridge.imgmsg_to_cv2(data,"bgr8")
        cv2.imshow("lala",cv_image)
        cv2.waitKey(0)
    
    def start_publish(self):
        global START_PUBLISH_FLAG
        START_PUBLISH_FLAG = 1
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.publish_launch = roslaunch.parent.ROSLaunchParent(
            uuid, [PUBLISH_LAUNCH_ADDR])
        self.publish_launch.start()
        self.subscribe_thread.start()
        
     
        QtWidgets.QMessageBox.information(self, "[INFO]", "finish start publishing", QtWidgets.QMessageBox.Ok)

    def stop_publish(self):
        global START_PUBLISH_FLAG
        START_PUBLISH_FLAG = 0
        if self.publish_launch:
            self.publish_launch.shutdown()
            self.subscribe_thread.stop()
            QtWidgets.QMessageBox.information(self, "[INFO]", "finish stop publishing", QtWidgets.QMessageBox.Ok)

    def start_collect_data(self):
        global START_PUBLISH_FLAG
        if START_PUBLISH_FLAG:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            self.collect_data_launch = roslaunch.parent.ROSLaunchParent(
                uuid, [COLLECT_DATA_LAUNCH_ADDR])
            self.collect_data_launch.start()
        else:
            QtWidgets.QMessageBox.information(self, "ERROR", "Please start publishing first!", QtWidgets.QMessageBox.Ok)

    def start_collect_calib(self):
        global START_PUBLISH_FLAG
        if START_PUBLISH_FLAG:
            rgb_img = self.subscribe_thread.rgb_img.copy()
            ir_img = self.subscribe_thread.ir_img.copy()
            cv2.imwrite()
            # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            # roslaunch.configure_logging(uuid)
            # self.collect_calib_launch = roslaunch.parent.ROSLaunchParent(
            #     uuid, [COLLECT_CALIB_LAUNCH_ADDR])
            # self.collect_calib_launch.start()
        else:
            QtWidgets.QMessageBox.information(self, "ERROR", "Please start publishing first!", QtWidgets.QMessageBox.Ok)

    def get_current_prefix(self):
        """
        get prefix from launch file
        """  
        dom = ET.parse(COLLECT_DATA_LAUNCH_ADDR)
        root = dom.getroot()
        for node in root.findall('param'):
            if node.get("name") == "prefix":
                prefix = node.get("value")
                break
        self.linedit_current_prefix.setText(prefix)


    def get_next_prefix(self):
        """
        calculate next prefix
        """
        dom = ET.parse(COLLECT_DATA_LAUNCH_ADDR)
        root = dom.getroot()
        for node in root.findall('param'):
            if node.get("name") == "save_dir":
                save_dir = node.get("value")
                break    
        if not os.path.exists(save_dir):
            QtWidgets.QMessageBox.information(self, "INFO", "current savedir doesn't exist \n", QtWidgets.QMessageBox.Ok) 
        index = 500
        while True:
            prefix = str(index).zfill(4)
            rgb_dir = save_dir + prefix + "_rgb.jpg"
            ir_dir = save_dir + prefix + "_ir.jpg"
            lidar_dir = save_dir + prefix + "_lidar.bag"
            if os.path.exists(rgb_dir) or os.path.exists(ir_dir) or os.path.exists(lidar_dir):
                index += 1
                prefix = str(index).zfill(4)
                break
            elif index > 1:
                index -= 1
            else:
                prefix = str(index).zfill(4)
                break
        self.linedit_current_prefix.setText(prefix)

        
    def set_prefix(self):
        """
        set new prefix to COLLECT_DATA_LAUNCH_ADDR and COLLECT_CALIB_LAUNCH_ADDR launch files
        """
        # ####################################################################
        # modify collect calib launch
        # ####################################################################
        dom = ET.parse(COLLECT_CALIB_LAUNCH_ADDR)
        prefix = self.linedit_current_prefix.text()
        root = dom.getroot()
        for node in root.findall('param'):
            if node.get("name") == "prefix":
                node.set("value", prefix)
                break
        dom.write(COLLECT_CALIB_LAUNCH_ADDR) 

        # #####################################################################
        # collect data launch
        # #####################################################################
        dom = ET.parse(COLLECT_DATA_LAUNCH_ADDR)
        root = dom.getroot()
        for node in root.findall('param'):
            if node.get("name") == "prefix":
                node.set("value", prefix)
                break

        # get save dir
        for node in root.findall('param'):
            if node.get("name") == "save_dir":
                save_dir = node.get("value")
                break
        # change lidar rosbag command 
        for node in root.findall('node'):
            if node.get("pkg") == "rosbag":
                args = node.get("args")
                break
        lst = args.split("-O ")
        new_args = lst[0] + "-O " + save_dir + prefix + "_lidar"
        node.set("args", new_args)
        # save
        dom.write(COLLECT_DATA_LAUNCH_ADDR)  
        QtWidgets.QMessageBox.information(self, "INFO", "finish set prefix to \n" + COLLECT_DATA_LAUNCH_ADDR + 
                                                " and \n" + COLLECT_CALIB_LAUNCH_ADDR, QtWidgets.QMessageBox.Ok)

    def get_current_savedir(self):
        """
        get COLLECT_DATA_LAUNCH_ADDR launch file savedir
        """
        dom = ET.parse(COLLECT_DATA_LAUNCH_ADDR)
        root = dom.getroot()
        for node in root.findall('param'):
            if node.get("name") == "save_dir":
                save_dir = node.get("value")
                break
        self.linedit_current_savedir.setText(save_dir)
        if not os.path.exists(save_dir):
            QtWidgets.QMessageBox.information(self, "INFO", "current savedir doesn't exist \n", QtWidgets.QMessageBox.Ok)
        
    def browse(self):
        download_path = QtWidgets.QFileDialog.getExistingDirectory(self,  
                                    "Browse",  
                                    "/home/why")   
        self.linedit_current_savedir.setText(download_path + "/")  

    def set_savedir(self):
        """
        set new save dir to COLLECT_DATA_LAUNCH_ADDR and COLLECT_CALIB_LAUNCH_ADDR launch files
        """
        # #########################################################
        # COLLECT_DATA_LAUNCH_ADDR
        # #########################################################
        dom = ET.parse(COLLECT_DATA_LAUNCH_ADDR)
        root = dom.getroot()
        for node in root.findall('param'):
            if node.get("name") == "save_dir":
                old_save_dir = node.get("value")
                break
        
        new_save_dir = self.linedit_current_savedir.text()
        node.set("value", new_save_dir)
                
        # change lidar output dir
        for node in root.findall('node'):
            if node.get("pkg") == "rosbag":
                args = node.get("args")
                break
        new_args = args.replace(old_save_dir, new_save_dir)  
        node.set("args", new_args)
        dom.write(COLLECT_DATA_LAUNCH_ADDR)  
        # #########################################################
        # COLLECT_CALIB_LAUNCH_ADDR
        # #########################################################
        dom = ET.parse(COLLECT_CALIB_LAUNCH_ADDR)
        root = dom.getroot()
        for node in root.findall('param'):
            if node.get("name") == "save_dir":
                break
        node.set("value", new_save_dir)
        dom.write(COLLECT_CALIB_LAUNCH_ADDR)  
        QtWidgets.QMessageBox.information(self, "INFO", "finish set savedir to \n" + COLLECT_DATA_LAUNCH_ADDR + 
                                                " and \n" + COLLECT_CALIB_LAUNCH_ADDR, QtWidgets.QMessageBox.Ok)

    def get_current_recordtime(self):
        """
        get COLLECT_DATA_LAUNCH_ADDR duration time
        """
        # #########################################################
        # COLLECT_DATA_LAUNCH_ADDR
        # #########################################################
        dom = ET.parse(COLLECT_DATA_LAUNCH_ADDR)
        root = dom.getroot()
        fam = re.compile("--duration=[0-9]+")
        for node in root.findall('node'):  # for every <node></node> 
            if (node.get("pkg") == "rosbag"):
                break        
        value = node.get("args") # without return None
        res = fam.findall(value)
        if not res:
            QtWidgets.QMessageBox.information(self, "ERROR", COLLECT_DATA_LAUNCH_ADDR  + "dont have record time!", QtWidgets.QMessageBox.Ok)
            return
        self.linedit_current_record_time.setText(res[0][11:])

    def set_recordtime(self):
        """
        set COLLECT_DATA_LAUNCH_ADDR duration time
        """
        # #########################################################
        # COLLECT_DATA_LAUNCH_ADDR
        # #########################################################
        dom = ET.parse(COLLECT_DATA_LAUNCH_ADDR)
        root = dom.getroot()
        for node in root.findall('node'):  # for every <node></node> 
            if (node.get("pkg") == "rosbag"):
                break     
        value = node.get("args") # without return None   
        fam = re.compile("--duration=[0-9]*")
        old_args = fam.findall(value)[0]
        try:
            new_record_time = int(self.linedit_current_record_time.text())
        except Exception as e:
            QtWidgets.QMessageBox.information(self, "INFO", "input record time error!", QtWidgets.QMessageBox.Ok)
            return
        new_args = "--duration={}".format(new_record_time)
        print(new_args)
        new_value = value.replace(old_args, new_args)
        node.set("args", new_value)
        dom.write(COLLECT_DATA_LAUNCH_ADDR)  
        QtWidgets.QMessageBox.information(self, "INFO", "finish set recordtime to \n" + COLLECT_DATA_LAUNCH_ADDR, QtWidgets.QMessageBox.Ok)




if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    app.setStyleSheet(qdarkstyle.load_stylesheet_pyqt5())
    PAC = MainWindow()
    PAC.showMaximized()
    
    # PAC.showNormal()
    sys.exit(app.exec_())

# os.system("python3 modify_name.py")





