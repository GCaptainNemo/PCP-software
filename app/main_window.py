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

import queue
import threading
import time

from utils import *
from main_subwindow import *

# bridge = CvBridge()
# publish
PUBLISH_LAUNCH_ADDR = "/home/why/ROS_self/publish_collect_review_data/src/collect_data/launch/publish_raw_data.launch"  # raw data/without visualize
PUBLISH_LIDAR_ONLY_ADDR = "/home/why/driver/ws_livox/src/livox_ros_driver/launch/livox_lidar_msg.launch"
# collect
COLLECT_DATA_LAUNCH_ADDR = "/home/why/ROS_self/publish_collect_review_data/src/collect_data/launch/collect_data.launch"
COLLECT_CALIB_LAUNCH_ADDR = "/home/why/ROS_self/publish_collect_review_data/src/collect_data/launch/collect_calib.launch"
# review
# ....
START_PUBLISH_FLAG = 0

IR_DEVICE = 4
RGB_DEVICE = 'rtsp://admin:a12345678@192.168.1.64/1'

class MainWindow(QtWidgets.QWidget):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.set_ui()
        self.setWindowTitle("ir/rgb/lidar PCR system@WHY")
        # ##############################################################
        # ros node subscribe
        # ##############################################################
       
        self.thread_start_rosbag = ThreadStartRosbag(self)
        self.thread_start_rosbag.signal_finished.connect(self.finished_rosbag_record)
        self.thread_start_rosbag.signal_started.connect(self.started_rosbag_record)
        

        # #############################################################
        try:
            self.get_current_prefix()
            self.get_current_recordtime()
            save_dir = self.get_current_savedir()
            if(os.path.exists(save_dir)):
                self.review_Widget.tree_view.set_path(save_dir)

            self.get_info()
        except Exception as e:
            pass
    

    def set_ui(self):
        VSplitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        HSplitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)

        self.setWindowTitle("PAC")   # publish and collect data
        self.ir_widget = SingleThreadCaptureWin(IR_DEVICE)
        self.rgb_widget = MultiThreadCaptureWin(RGB_DEVICE)
        VSplitter.addWidget(self.rgb_widget)
        VSplitter.addWidget(self.ir_widget)
        # ###############################################################3
        Hlayout_1 = QtWidgets.QHBoxLayout()
        self.btn_start_publish = QtWidgets.QPushButton('Start Publish')
        self.btn_check_lidar = QtWidgets.QPushButton('Check lidar connection')
        self.btn_check_lidar.clicked.connect(self.check_lidar)
        self.btn_stop_publish = QtWidgets.QPushButton('Stop Publish')
        self.btn_stop_publish.setEnabled(False)
        self.btn_start_collect_triplet = QtWidgets.QPushButton('Collect ir/rgb/lidar triplet')
        self.rosbag_record_progressbar = QtWidgets.QProgressBar()
        self.rosbag_record_progressbar.setValue(0)
        self.rosbag_timer = QtCore.QTimer() # timer to collect rosbag
        self.rosbag_timer.timeout.connect(self.update_progressbar)
        self.rosbag_recorded_time = 0

        self.btn_start_collect_pair = QtWidgets.QPushButton('Collect ir/rgb pair')
        Hlayout_1.addWidget(self.btn_start_publish)
        Hlayout_1.addWidget(self.btn_check_lidar)
        Hlayout_1.addWidget(self.btn_stop_publish)

        Hlayout_1.addWidget(self.btn_start_collect_triplet)
        Hlayout_1.addWidget(self.rosbag_record_progressbar)
        Hlayout_1.addWidget(self.btn_start_collect_pair)
        self.publish_launch = None
        self.btn_start_publish.clicked.connect(self.start_publish)

        self.btn_stop_publish.clicked.connect(self.stop_publish)

        self.btn_start_collect_triplet.clicked.connect(self.start_collect_triplet)
        self.btn_start_collect_pair.clicked.connect(self.start_collect_pair)

        # ################################################################
        Hlayout_2 = QtWidgets.QHBoxLayout()
        self.label_current_prefix = QtWidgets.QLabel("prefix: ")
        self.linedit_current_prefix = QtWidgets.QLineEdit()
        self.btn_get_current_prefix = QtWidgets.QPushButton("Current Prefix")        
        self.btn_next_prefix = QtWidgets.QPushButton("Next Prefix")
        self.btn_set_prefix = QtWidgets.QPushButton("Set Prefix")
        self.btn_next_and_set_prefix = QtWidgets.QPushButton("Next & Set Prefix")

        self.btn_get_current_prefix.clicked.connect(self.get_current_prefix)
        self.btn_next_prefix.clicked.connect(self.get_next_prefix)
        self.btn_set_prefix.clicked.connect(self.set_prefix)
        self.btn_next_and_set_prefix.clicked.connect(self.next_and_set_prefix)

        Hlayout_2.addWidget(self.label_current_prefix)
        Hlayout_2.addWidget(self.linedit_current_prefix)
        Hlayout_2.addWidget(self.btn_get_current_prefix)
        Hlayout_2.addWidget(self.btn_next_prefix)
        Hlayout_2.addWidget(self.btn_set_prefix)
        Hlayout_2.addWidget(self.btn_next_and_set_prefix)

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
        Hlayout_5 = QtWidgets.QHBoxLayout()
        self.label_current_rgb_info = QtWidgets.QLabel("RGB: ")
        self.linedit_current_rgb_info = QtWidgets.QLineEdit()
        self.label_current_ir_info = QtWidgets.QLabel("IR: ")
        self.linedit_current_ir_info = QtWidgets.QLineEdit()
        self.label_current_lidar_info = QtWidgets.QLabel("LIDAR: ")
        self.linedit_current_lidar_info = QtWidgets.QLineEdit()
        self.label_current_calib_ir_info = QtWidgets.QLabel("calib IR: ")
        self.lineedit_current_calib_ir_info = QtWidgets.QLineEdit()
        self.label_current_calib_rgb_info = QtWidgets.QLabel("calib RGB: ")
        self.lineedit_current_calib_rgb_info = QtWidgets.QLineEdit()
        self.btn_get_info = QtWidgets.QPushButton("prefix info")
        Hlayout_5.addWidget(self.label_current_rgb_info)
        Hlayout_5.addWidget(self.linedit_current_rgb_info)
        Hlayout_5.addWidget(self.label_current_ir_info)
        Hlayout_5.addWidget(self.linedit_current_ir_info)
        Hlayout_5.addWidget(self.label_current_lidar_info)
        Hlayout_5.addWidget(self.linedit_current_lidar_info)
        Hlayout_5.addWidget(self.label_current_calib_ir_info)
        Hlayout_5.addWidget(self.lineedit_current_calib_ir_info)
        Hlayout_5.addWidget(self.label_current_calib_rgb_info)
        Hlayout_5.addWidget(self.lineedit_current_calib_rgb_info)
        Hlayout_5.addWidget(self.btn_get_info)
        self.btn_get_info.clicked.connect(self.get_info)

        # ################################################################        
        vlayout = QtWidgets.QVBoxLayout(self)
        HSplitter.addWidget(VSplitter)
        self.review_Widget = ReviewWidget()
        HSplitter.addWidget(self.review_Widget)
        vlayout.addWidget(HSplitter)
        vlayout.addLayout(Hlayout_1)
        vlayout.addLayout(Hlayout_2)
        vlayout.addLayout(Hlayout_3)
        vlayout.addLayout(Hlayout_4)
        vlayout.addLayout(Hlayout_5)
    
    def start_publish(self):
        global START_PUBLISH_FLAG
        START_PUBLISH_FLAG = 1
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.publish_launch = roslaunch.parent.ROSLaunchParent(
            uuid, [PUBLISH_LIDAR_ONLY_ADDR])
        self.publish_launch.start()
        self.ir_widget.open_camera()
        self.rgb_widget.open_camera()
        res = os.system("rostopic list |grep /livox/lidar")
        if res == 0:
            QtWidgets.QMessageBox.information(self, "[INFO]", "finish start publishing, /livox/lidar detected", QtWidgets.QMessageBox.Ok)
        else:
            QtWidgets.QMessageBox.information(self, "[WARNING]", "finish start publishing, /livox/lidar not detected!", QtWidgets.QMessageBox.Ok)
        self.btn_start_publish.setEnabled(False)
        self.btn_stop_publish.setEnabled(True)

    def check_lidar(self):
        res = os.system("rostopic list |grep /livox/lidar")
        if res == 0:
            QtWidgets.QMessageBox.information(self, "[INFO]", "/livox/lidar detected", QtWidgets.QMessageBox.Ok)
        else:
            QtWidgets.QMessageBox.information(self, "[INFO]", "/livox/lidar not detected!", QtWidgets.QMessageBox.Ok)
        
        
    def stop_publish(self):
        global START_PUBLISH_FLAG
        START_PUBLISH_FLAG = 0
        if self.publish_launch:
            self.publish_launch.shutdown()
        self.rgb_widget.close_camera()
        self.ir_widget.close_camera()
        self.btn_start_publish.setEnabled(True)
        self.btn_stop_publish.setEnabled(False)


    def start_collect_triplet(self):
        global START_PUBLISH_FLAG
        if START_PUBLISH_FLAG:
            ir_file_name, rgb_file_name, lidar_file_name = self.get_data_save_dir()
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
            record_time = int(res[0][11:])
            rgb_img = self.rgb_widget.image.copy()
            ir_img = self.ir_widget.image.copy()
            cv2.imwrite(ir_file_name, ir_img)
            cv2.imwrite(rgb_file_name, rgb_img)
            self.thread_start_rosbag.record_time = record_time
            self.thread_start_rosbag.lidar_file_name = lidar_file_name

            self.thread_start_rosbag.start()
            QtWidgets.QMessageBox.information(self, "INFO", "rosbag record {} seconds".format(record_time), QtWidgets.QMessageBox.Ok)
            # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            # roslaunch.configure_logging(uuid)
            # self.collect_data_launch = roslaunch.parent.ROSLaunchParent(
            #     uuid, [COLLECT_DATA_LAUNCH_ADDR])
            # self.collect_data_launch.start()
            # QtWidgets.QMessageBox.information(self, "INFO", "finish collecting ir/rgb/lidar frame", QtWidgets.QMessageBox.Ok)
            # self.collect_data_launch.shutdown()
        else:
            QtWidgets.QMessageBox.information(self, "ERROR", "Please start publishing first!", QtWidgets.QMessageBox.Ok)

    def started_rosbag_record(self):
        self.rosbag_timer.start(1000)

    def update_progressbar(self):
        self.rosbag_recorded_time += 1 
        self.rosbag_record_progressbar.setValue(self.rosbag_recorded_time / self.thread_start_rosbag.record_time * 100)
        

    def finished_rosbag_record(self, success):
        if(success == 0):
            QtWidgets.QMessageBox.information(self, "INFO", "finish collecting ir/rgb/lidar triplet", 
                QtWidgets.QMessageBox.Ok)
        elif(success == 1):
            QtWidgets.QMessageBox.information(self, "[ERROR]", "record time or save dir undefined", QtWidgets.QMessageBox.Ok)
        self.rosbag_timer.stop()
        self.rosbag_recorded_time = 0
        self.rosbag_record_progressbar.setValue(0)
        print("thread_start_rosbag status = ", str(self.thread_start_rosbag.isFinished()))
        # self.thread_start_rosbag.stop()

    def get_info(self):
        prefix = self.linedit_current_prefix.text()
        dom = ET.parse(COLLECT_DATA_LAUNCH_ADDR)
        root = dom.getroot()
        for node in root.findall('param'):
            if node.get("name") == "save_dir":
                save_dir = node.get("value")
                break
        lidar_count = 0
        rgb_count = 0
        ir_count = 0
        calib_ir_count = 0
        calib_rgb_count = 0
        rgb_fam = re.compile(prefix + "_[0-9]+__rgb")
        ir_fam = re.compile(prefix + "_[0-9]+__ir")
        
        for file in os.listdir(save_dir):
            if file == prefix + "_lidar.bag":
                lidar_count += 1
                continue
            elif file == prefix + "_rgb.jpg":
                rgb_count += 1
                continue
            elif file == prefix + "_ir.jpg":
                ir_count += 1
                continue
            res_lst = rgb_fam.findall(file)
            if res_lst:
                calib_rgb_count += 1
                continue
            res_lst = ir_fam.findall(file)
            if res_lst:
                calib_ir_count += 1
                continue
        self.linedit_current_ir_info.setText(str(ir_count))
        self.linedit_current_rgb_info.setText(str(rgb_count))
        self.linedit_current_lidar_info.setText(str(lidar_count))
        self.lineedit_current_calib_ir_info.setText(str(calib_ir_count))
        self.lineedit_current_calib_rgb_info.setText(str(calib_rgb_count))

    def get_data_save_dir(self):
        dom = ET.parse(COLLECT_DATA_LAUNCH_ADDR)
        root = dom.getroot()
        for node in root.findall('param'):
            if node.get("name") == "save_dir":
                save_dir = node.get("value")
            if node.get("name") == "prefix":
                prefix = node.get("value")
        ir_file_name = save_dir + prefix + "_ir.jpg" 
        rgb_file_name = save_dir + prefix + "_rgb.jpg"
        lidar_file_name = save_dir + prefix + "_lidar.bag"
        return ir_file_name, rgb_file_name, lidar_file_name
    

    def get_calib_save_dir(self):
        dom = ET.parse(COLLECT_DATA_LAUNCH_ADDR)
        root = dom.getroot()
        for node in root.findall('param'):
            if node.get("name") == "save_dir":
                save_dir = node.get("value")
            if node.get("name") == "prefix":
                prefix = node.get("value")
        index = 100
        while True:
            ir_file_name = save_dir + prefix + "_{}__ir.jpg".format(index) 
            rgb_file_name = save_dir + prefix + "_{}__rgb.jpg".format(index) 
            if os.path.exists(ir_file_name) or os.path.exists(rgb_file_name):
                index += 1
                ir_file_name = save_dir + prefix + "_{}__ir.jpg".format(index) 
                rgb_file_name = save_dir + prefix + "_{}__rgb.jpg".format(index) 
                break
            elif index > 1:
                index -= 1
            else:
                ir_file_name = save_dir + prefix + "_{}__ir.jpg".format(index) 
                rgb_file_name = save_dir + prefix + "_{}__rgb.jpg".format(index) 
                break
        return ir_file_name, rgb_file_name



        
    def start_collect_pair(self):
        global START_PUBLISH_FLAG
        if START_PUBLISH_FLAG:
            ir_file_name, rgb_file_name = self.get_calib_save_dir()
            rgb_img = self.rgb_widget.image.copy()
            ir_img = self.ir_widget.image.copy()
            # rgb_img = self.subscribe_rgb_thread.rgb_img.copy()
            # ir_img = self.subscribe_rgb_thread.ir_img.copy()
            cv2.imwrite(ir_file_name, ir_img)
            cv2.imwrite(rgb_file_name, rgb_img)
            QtWidgets.QMessageBox.information(self, "INFO", "finish collecting ir/rgb pair to " + ir_file_name + "\n" + rgb_file_name, 
                QtWidgets.QMessageBox.Ok)
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
        return prefix

    def get_next_prefix(self):
        """
        calculate next prefix and set to lineedit
        """
        dom = ET.parse(COLLECT_DATA_LAUNCH_ADDR)
        root = dom.getroot()
        for node in root.findall('param'):
            if node.get("name") == "save_dir":
                save_dir = node.get("value")
                break    
        if not os.path.exists(save_dir):
            QtWidgets.QMessageBox.information(self, "INFO", "current savedir doesn't exist \n", QtWidgets.QMessageBox.Ok) 
            return -1
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

    def next_and_set_prefix(self):
        """
        get next prefix and set
        """
        if(self.get_next_prefix() != -1):  # save dir not exist
            self.set_prefix()

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
        return save_dir

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
        return res[0][11:]

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





