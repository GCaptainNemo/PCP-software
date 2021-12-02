from PyQt5 import QtGui, QtWidgets, QtCore
from sys import argv, exit
import cv2
import os

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
# import pydensecrf.densecrf as dcrf
# from pydensecrf.utils import unary_from_softmax, create_pairwise_bilateral
# import torch
# import torch.nn.functional as F
# import torch.nn as nn
# from torch.utils.data import DataLoader
# import cv2
# from PIL import Image
# from utils.metrics import compute_iou_batch
# from models.net import EncoderDecoderNet, SPPNet
# from dataset.cityscapes import CityscapesDataset
# from dataset.mydata_temp import MyDataset
# from utils.preprocess import minmax_normalize
# import yaml




class MyWinPicture(QtWidgets.QWidget):
    """
    Rewrite Qwidget to display pictures.
    """
    def __init__(self):
        super(MyWinPicture, self).__init__()
        self.dir = None

    def paintEvent(self, event):
        try:
            painter = QtGui.QPainter(self)
            pixmap = QtGui.QPixmap(self.dir)
            painter.drawPixmap(self.rect(), pixmap)
        except Exception as e:
            print(e)

class MyWidgetRegistration(MyWinPicture):
    """
        Rewrite MyWinPicture to registrate manually.
    """
    def __init__(self):
        super(MyWidgetRegistration, self).__init__()
        self.Feature_Point_lst = []
        self.flag = 0

    def paintEvent(self, event):
        super().paintEvent(event)
        rect = QtCore.QRect(self.x0, self.y0, abs(self.x1 - self.x0), abs(self.y1 - self.y0))
        painter = QtGui.QPainter(self)
        painter.setPen(QtGui.QPen(QtCore.Qt.red, 2, QtCore.Qt.SolidLine))
        painter.drawRect(rect)

    def mousePressEvent(self, event):
        self.x0 = event.x()
        self.y0 = event.y()
        self.Feature_Point_lst.append([event.x(), event.y()])
        self.flag = 1

    def mouseMoveEvent(self, event):
        if self.flag:
            self.x1 = event.x()
            self.y1 = event.y()
            self.update()


    def mouseReleaseEvent(self, event, points):
        try:
            # for p in self.lastClicked:
            #     p.resetPen()
            # for p in points:
            #     p.setPen('r', width=2)
            #     # self.signal_judge_point1.emit(mousePoint1.x(), mousePoint1.y())
            # self.lastClicked = points
            self.flag = 0
        except Exception as e:
            print(e)





class MyWindow(QtWidgets.QWidget):
    """
        __init__(): setup GUI and function connection
    """
    def __init__(self):
        super(MyWindow, self).__init__()
        self.show()
        self.resize(2000, 1500)
        font = QtGui.QFont()
        font.setPointSize(9)
        self.setFont(font)
        self.setWindowTitle("Main Window")
        Tab = QtWidgets.QTabWidget()
        Tab.setFont(QtGui.QFont('Times New Roman', 10))
        self.window_1 = QtWidgets.QWidget()
        Tab.addTab(self.window_1, "数据集展示")
        # window2
        self.window_2 = QtWidgets.QWidget()
        Tab.addTab(self.window_2, "无人机数据评估")
        Vlayout = QtWidgets.QVBoxLayout(self.window_2)
        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.Movie_display_widget = MyLoadingVideo()
        self.textwidget_2_point = QtWidgets.QTextEdit()
        splitter.addWidget(self.Movie_display_widget)
        splitter.addWidget(self.textwidget_2_point)
        Vlayout.addWidget(splitter)
        Hlayout = QtWidgets.QHBoxLayout()
        self.button2_browse = QtWidgets.QPushButton("浏览")
        self.button2_browse.clicked.connect(self.browse_2)
        self.button2_stop = QtWidgets.QPushButton("停止")
        self.button2_stop.clicked.connect(self.stop_2)
        self.button2_point = QtWidgets.QPushButton("评分")
        self.button2_point.clicked.connect(self.point_2)
        Hlayout.addWidget(self.button2_browse)
        Hlayout.addWidget(self.button2_stop)
        Hlayout.addWidget(self.button2_point)
        Vlayout.addLayout(Hlayout)
        # Window3
        self.window_3 = QtWidgets.QWidget()
        Tab.addTab(self.window_3, "样例级/语义解析")
        Vlayout_3 = QtWidgets.QVBoxLayout(self.window_3)
        self.show_image_widget_3 = MyWinPicture()
        self.show_text_3 = QtWidgets.QTextEdit()
        self.show_text_3.setReadOnly(True)
        splitter_image_text_3 = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        splitter_image_text_3.addWidget(self.show_image_widget_3)
        splitter_image_text_3.addWidget(self.show_text_3)
        Vlayout_3.addWidget(splitter_image_text_3)
        Hlayout_3_button = QtWidgets.QHBoxLayout()
        self.button3_1 = QtWidgets.QPushButton("选择")
        self.button3_1.clicked.connect(self.browse_3)
        self.button3_2 = QtWidgets.QPushButton("Run")
        self.button3_2.clicked.connect(self.run_3)
        self.button3_3 = QtWidgets.QPushButton("准确率")
        self.button3_3.clicked.connect(self.calculate_3)
        Hlayout_3_button.addWidget(self.button3_1)
        Hlayout_3_button.addWidget(self.button3_2)
        Hlayout_3_button.addWidget(self.button3_3)
        Vlayout_3.addLayout(Hlayout_3_button)
        # Window4
        self.window_4 = QtWidgets.QWidget()
        Tab.addTab(self.window_4, "样例级/语义解析")
        Vlayout_4 = QtWidgets.QVBoxLayout(self.window_4)
        self.show_image_widget_4_IR = MyWinPicture()
        self.show_image_widget_4_RGB = MyWinPicture()
        Vsplitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        self.show_text_4 = QtWidgets.QTextEdit()
        self.show_text_4.setReadOnly(True)
        splitter_image_text_4 = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        Vsplitter.addWidget(self.show_image_widget_4_IR)
        Vsplitter.addWidget(self.show_image_widget_4_RGB)

        splitter_image_text_4.addWidget(Vsplitter)
        splitter_image_text_4.addWidget(self.show_text_4)
        Vlayout_4.addWidget(splitter_image_text_4)
        Hlayout_4_button = QtWidgets.QHBoxLayout()
        self.button4_1 = QtWidgets.QPushButton("选择(IR)")
        self.button4_1.clicked.connect(self.browse_4_IR)
        self.button4_2 = QtWidgets.QPushButton("Run(IR)")
        self.button4_2.clicked.connect(self.run_4_IR)
        self.button4_3 = QtWidgets.QPushButton("准确率(IR)")
        self.button4_3.clicked.connect(self.calculate_4_IR)
        self.button4_4 = QtWidgets.QPushButton("选择(RGB)")
        self.button4_4.clicked.connect(self.browse_4_RGB)
        self.button4_5 = QtWidgets.QPushButton("Run(RGB)")
        self.button4_5.clicked.connect(self.run_4_RGB)
        self.button4_6 = QtWidgets.QPushButton("准确率(RGB)")
        self.button4_6.clicked.connect(self.calculate_4_RGB)

        self.accir = 0; self.accrgb = 0

        Hlayout_4_button.addWidget(self.button4_1)
        Hlayout_4_button.addWidget(self.button4_2)
        Hlayout_4_button.addWidget(self.button4_3)
        Hlayout_4_button.addWidget(self.button4_4)
        Hlayout_4_button.addWidget(self.button4_5)
        Hlayout_4_button.addWidget(self.button4_6)

        Vlayout_4.addLayout(Hlayout_4_button)
        # Window5
        self.window_5 = QtWidgets.QWidget()
        VSplitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        Hsplitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        vlayout = QtWidgets.QVBoxLayout(self.window_5)
        # self.Infrared_widget_5 = MyWinPicture()
        # self.RGB_widget_5 = MyWinPicture()
        # self.Output_5 = MyWinPicture()
        self.Infrared_widget_5 = MyWidgetRegistration()
        self.RGB_widget_5 = MyWidgetRegistration()
        self.Output_5 = MyWidgetRegistration()
        VSplitter.addWidget(self.Infrared_widget_5)
        VSplitter.addWidget(self.RGB_widget_5)
        Hsplitter.addWidget(VSplitter)
        Hsplitter.addWidget(self.Output_5)
        Hlayout = QtWidgets.QHBoxLayout()
        self.button5_1 = QtWidgets.QPushButton('Browse(IR)')
        self.button5_1.clicked.connect(self.browse5_IR)
        self.button5_2 = QtWidgets.QPushButton('Browse(RGB)')
        self.button5_2.clicked.connect(self.browse5_RGB)
        self.button5_3 = QtWidgets.QPushButton('Registration')
        self.button5_3.clicked.connect(self.registration_5)

        Hlayout.addWidget(self.button5_1)
        Hlayout.addWidget(self.button5_2)
        Hlayout.addWidget(self.button5_3)
        vlayout.addWidget(Hsplitter)
        vlayout.addLayout(Hlayout)
        Tab.addTab(self.window_5, "配准")
        # Window6
        self.window_6 = QtWidgets.QWidget()
        Tab.addTab(self.window_6, "三维模型展示")
        Vlayout = QtWidgets.QVBoxLayout(self.window_6)
        self.text_widget6 = QtWidgets.QTextEdit()
        Hlayout6_3 = QtWidgets.QHBoxLayout()
        self.button6_1 = QtWidgets.QPushButton("显示原始模型")
        self.button6_1.clicked.connect(self.show_orijinal_model6)
        self.button6_2 = QtWidgets.QPushButton("展示语义模型")
        self.button6_2.clicked.connect(self.show_semantic_model6)
        self.button6_3 = QtWidgets.QPushButton("展示实例模型")
        self.button6_3.clicked.connect(self.show_example_model6)
        self.button6_4 = QtWidgets.QPushButton("展示材质模型")
        self.button6_4.clicked.connect(self.show_material_model6)

        Hlayout6_3.addWidget(self.button6_1)
        Hlayout6_3.addWidget(self.button6_2)
        Hlayout6_3.addWidget(self.button6_3)
        Hlayout6_3.addWidget(self.button6_4)
        Vlayout.addWidget(self.text_widget6)
        Vlayout.addLayout(Hlayout6_3)
        # Window7
        self.window_7 = QtWidgets.QWidget()
        Tab.addTab(self.window_7, "仿真结果展示")
        Vlayout = QtWidgets.QVBoxLayout(self.window_7)
        self.text_widget7 = QtWidgets.QTextEdit()
        self.button7_openexe = QtWidgets.QPushButton("打开软件")
        self.button7_openexe.clicked.connect(self.Open_exe7)
        Vlayout.addWidget(self.text_widget7)
        Vlayout.addWidget(self.button7_openexe)
        Hlayout = QtWidgets.QHBoxLayout(self)
        Hlayout.addWidget(Tab)

    def browse_2(self):
        try:
            self.file_dir2, filetype = QtWidgets.QFileDialog.getOpenFileName(self, '选择文件', '',
                                                                                   'files(*.gif , *.avi, *.mp4)')
            print(self.file_dir2, filetype)
            self.Movie_display_widget.movie_dir = self.file_dir2
            self.Movie_display_widget.setMovie()
        except Exception as e:
            print(e)

    def stop_2(self):
        """ Stop the video. """
        try:
            self.Movie_display_widget.stop()
        except:
            pass

    def point_2(self):
        """ Show points. """
        try:
            ...
            # self.textwidget_2_point.setText("")
        except Exception as e:
            print(e)

    def browse_3(self):
        try:
            self.file_dir3, filetype = QtWidgets.QFileDialog.getOpenFileName(self, '选择文件', '',
                                                                                   'files(*.jpg , *.png)')
            print(self.file_dir3, filetype)
            self.show_image_widget_3.dir = self.file_dir3
            self.show_image_widget_3.update()
        except Exception as e:
            print(e)

    def run_3(self):
        """ Window3 'Run' button. Use self.file_dir3 as an interface."""
        try:
            ...
        except Exception as e:
            print(e)

    def calculate_3(self):
        """ Show results."""
        ...
        # self.show_text_3.setText(...)

    def browse_4_IR(self):
        """ Window4 browse Infra-Red picture"""
        try:
            self.file_dir4_IR, filetype = QtWidgets.QFileDialog.getOpenFileName(self, '选择文件', '',
                                                                            'files(*.jpg , *.png)')
            print(self.file_dir4_IR, filetype)
            self.show_image_widget_4_IR.dir = self.file_dir4_IR
            self.show_image_widget_4_IR.update()
        except Exception as e:
            print(e)

    def browse_4_RGB(self):
        """ Window4 browse RGB picture"""
        try:
            self.file_dir4_RGB, filetype = QtWidgets.QFileDialog.getOpenFileName(self, '选择文件', '',
                                                                            'files(*.jpg , *.png)')
            print(self.file_dir4_RGB, filetype)
            self.show_image_widget_4_RGB.dir = self.file_dir4_RGB
            self.show_image_widget_4_RGB.update()
        except Exception as e:
            print(e)

    def run_4_IR(self):
        """ Window4 'Run(IR)' button. Use self.file_dir4 as an interface."""
        try:
            ...            
        except Exception as e:
            print(e)

    def run_4_RGB(self):
        """ Window4 'Run(RGB)' button. Use self.file_dir4 as an interface."""
        try:
            ...
        except Exception as e:
            print(e)

    def calculate_4_IR(self):
        self.show_text_3.setText(self.accir)

    def calculate_4_RGB(self):
        self.show_text_3.setText(self.accrgb)

    def browse5_IR(self):
        try:
            self.file_dir5_IR, filetype = QtWidgets.QFileDialog.getOpenFileName(self, '选择红外图片', '',
                                                                            'files(*.jpg , *.png)')
            print(self.file_dir5_IR, filetype)
            self.Infrared_widget_5.dir = self.file_dir5_IR
            self.Infrared_widget_5.update()
        except Exception as e:
            print(e)

    def browse5_RGB(self):
        try:
            self.file_dir5_RGB, filetype = QtWidgets.QFileDialog.getOpenFileName(self, '选择可见光图片', '',
                                                                            'files(*.jpg , *.png)')
            print(self.file_dir5_RGB, filetype)
            self.RGB_widget_5.dir = self.file_dir5_RGB
            self.RGB_widget_5.update()
        except Exception as e:
            print(e)

    def registration_5(self):
        ...

    def show_orijinal_model6(self):
        ...

    def show_semantic_model6(self):
        ...

    def show_example_model6(self):
        ...

    def show_material_model6(self):
        ...

    def Open_exe7(self):
        """ Function used to open .exe"""
        os.chdir("D:\\ENVI4.5\\") #子目录
        path_01 = "envi45winx86_32.exe"  #调用的exe
        os.system(path_01)
        print(1)


if __name__ == '__main__':
    try:
        app = QtWidgets.QApplication(argv)
        window = MyWindow()
        exit(app.exec_())
    except Exception as e:
        print(e)
