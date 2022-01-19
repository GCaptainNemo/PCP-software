from PyQt5 import QtGui, QtCore, QtWidgets
import cv2
import threading
import queue
import time
from utils import ThreadOperator
from mono_calib import MonoIrCalib
from stereo_calib import StereoCalib
import os
import re



class MultiThreadCaptureWin(QtWidgets.QWidget):
    def __init__(self, device_info):
        """
        Rewrite Qwidget to display pictures in label widget.
        """
        super(MultiThreadCaptureWin, self).__init__()
        self.device_info = device_info
        self.cap = None
        self.set_ui()
        self.img_queue = queue.Queue()
        self.img_get_thread = None
        self.img_put_thread = None
    
    
    def image_put(self):
        while True:
            self.img_queue.put(self.cap.read()[1])
            self.img_queue.get() if self.img_queue.qsize() > 1 else time.sleep(0.01)

    def image_get(self):
        while True:
            try:
                self.image = self.img_queue.get()
                try:
                    if self.image.data.shape[0]:
                        img = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
                except Exception as e:
                    continue
                # cv2.imshow("123", frame)
                # cv2.waitKey(1)
                # if self.image is None or self.image.size == 0:
                #     continue
                show_image = QtGui.QImage(img.data, img.shape[1],img.shape[0], QtGui.QImage.Format_RGB888)
                self.camera_label.setPixmap(QtGui.QPixmap.fromImage(show_image))
                time.sleep(0.1)  # if update label too frequently will crash
            except Exception as e:
                print(e)

    def set_ui(self):
        self.camera_label = QtWidgets.QLabel("")
        self.camera_label.setScaledContents(False)
        vlayout = QtWidgets.QVBoxLayout()
        vlayout.addWidget(self.camera_label)
        self.setLayout(vlayout)

  
    def open_camera(self):
        self.cap = cv2.VideoCapture()
        flag = self.cap.open(self.device_info)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

        if flag == False:
            msg = QtWidgets.QMessageBox.information(self, "[error]", 
            str(self.device_info) + " camera can not connect!", QtWidgets.QMessageBox.Ok)
        else:
            print("[info] thread num: ", len(threading.enumerate()))
            try:
                if self.img_put_thread:
                    ThreadOperator.stop_thread(self.img_put_thread)
                if self.img_get_thread:
                    ThreadOperator.stop_thread(self.img_get_thread)
            except Exception as e:
                print(e)
            self.img_put_thread = threading.Thread(target=self.image_put)
            self.img_get_thread = threading.Thread(target=self.image_get)
            self.img_put_thread.setDaemon(True)
            self.img_get_thread.setDaemon(True)
            self.img_put_thread.start()
            self.img_get_thread.start()

    def close_camera(self):
        try:
            if self.img_put_thread:
                ThreadOperator.stop_thread(self.img_put_thread)
            if self.img_get_thread:
                ThreadOperator.stop_thread(self.img_get_thread)
            self.camera_label.clear()
        except Exception as e:
            print(e)

class SingleThreadCaptureWin(QtWidgets.QWidget):
    def __init__(self, device_info):
        """
        use timer to trigger update label pixmap. cannot display real time
        Rewrite Qwidget to display pictures in label widget.
        """
        super(SingleThreadCaptureWin, self).__init__()
        self.device_info = device_info
        self.cap = cv2.VideoCapture()
        self.timer = QtCore.QTimer() #初始化定时器
        self.set_ui()
        self.init_slot()

    def set_ui(self):
        self.camera_label = QtWidgets.QLabel("")
        self.camera_label.setScaledContents(True)
        vlayout = QtWidgets.QVBoxLayout()
        vlayout.addWidget(self.camera_label)
        self.setLayout(vlayout)

    def init_slot(self):
        self.timer.timeout.connect(self.show_camera)
    
    def show_camera(self):
        flag, self.image = self.cap.read()
        
        self.image = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
        if flag:
            show_image = QtGui.QImage(self.image.data, self.image.shape[1],self.image.shape[0], QtGui.QImage.Format_RGB888)
            self.camera_label.setPixmap(QtGui.QPixmap.fromImage(show_image))

    def open_camera(self):
        flag = self.cap.open(self.device_info)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        if flag == False:
            msg = QtWidgets.QMessageBox.information(self, "[error]", 
            str(self.device_info) + " camera can not connect!", QtWidgets.QMessageBox.Ok)
        else:
            self.timer.start(30)


    def close_camera(self):
        self.timer.stop()
        self.cap.release()
        self.camera_label.clear()


class ReviewWidget(QtWidgets.QWidget):
    def __init__(self):
        super(ReviewWidget, self).__init__()
        self.set_ui()
        self.mono_calib_obj = MonoIrCalib()
        self.stereo_calib_obj = StereoCalib()

    def set_ui(self):
        self.label_mono_prefix = QtWidgets.QLabel("Mono prefix: ")
        self.linedit_mono_prefix = QtWidgets.QLineEdit()
        self.linedit_mono_prefix.setPlaceholderText("0000 or all")

        self.label_mono_suffix = QtWidgets.QLabel("Mono suffix: ")
        self.linedit_mono_suffix = QtWidgets.QLineEdit()
        self.linedit_mono_suffix.setPlaceholderText("ir or rgb")
        self.btn_mono_calib_start = QtWidgets.QPushButton("Mono Calib")    
        self.btn_mono_calib_start.clicked.connect(self.start_mono_calib)
        Hlayout_1 = QtWidgets.QHBoxLayout()
        Hlayout_1.addWidget(self.label_mono_prefix)
        Hlayout_1.addWidget(self.linedit_mono_prefix)
        Hlayout_1.addWidget(self.label_mono_suffix)
        Hlayout_1.addWidget(self.linedit_mono_suffix)
        Hlayout_1.addWidget(self.btn_mono_calib_start)
        # #######################################################
        self.label_stereo_prefix = QtWidgets.QLabel("Stereo prefix: ")
        self.linedit_stereo_prefix = QtWidgets.QLineEdit()
        self.linedit_stereo_prefix.setPlaceholderText("0000 or all")
        self.btn_stereo_calib_start = QtWidgets.QPushButton("Stereo Calib")  
        self.btn_stereo_calib_start.clicked.connect(self.start_stereo_calib)

        Hlayout_2 = QtWidgets.QHBoxLayout()
        Hlayout_2.addWidget(self.label_stereo_prefix)
        Hlayout_2.addWidget(self.linedit_stereo_prefix)
        Hlayout_2.addWidget(self.btn_stereo_calib_start)

        vlayout = QtWidgets.QVBoxLayout(self)
        vlayout.addLayout(Hlayout_1)
        vlayout.addLayout(Hlayout_2)
        # ########################################################
        # self.text_widget = QtWidgets.QTextEdit()
        # vlayout.addWidget(self.text_widget)
        vsplitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        self.tree_view = TreeView()
        self.tree_view.doubleClicked.connect(self.review_ir_rgb)    

        vsplitter.addWidget(self.tree_view)

        # ########################################################
        hsplitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.ir_display_widget = LoadingVideoWin(self)
        self.rgb_display_widget = LoadingVideoWin(self)
        hsplitter.addWidget(self.ir_display_widget)
        hsplitter.addWidget(self.rgb_display_widget)
        vsplitter.addWidget(hsplitter)

        vlayout.addWidget(vsplitter)        

        self.setLayout(vlayout)    

    def review_ir_rgb(self):
        index = self.tree_view.currentIndex()
        model = index.model()
        self.tree_view.__current_select_path = model.filePath(index)
        addr = self.tree_view.__current_select_path
        if(os.path.isfile(addr)):
            print(addr)
            reg_ = re.compile("(?:ir|rgb)[.]jpg")
            res_lst = reg_.findall(addr)
            if not res_lst:
                QtWidgets.QMessageBox.information(self, "[error]", "[error] only review ir, rgb img!", QtWidgets.QMessageBox.Ok) 
            else:
                self.show_data(addr)
    
    def show_data(self, addr):
        reg_ = re.compile("ir[.]jpg")
        res_lst = reg_.findall(addr)
        if res_lst:
            # ir img
            self.ir_display_widget.stop()
            self.ir_display_widget.img_dir = addr
            self.ir_display_widget.show_img()
            return 
                
        reg_ = re.compile("rgb[.]jpg")
        res_lst = reg_.findall(addr)
        if res_lst:
            # rgb img
            self.rgb_display_widget.stop()
            self.rgb_display_widget.img_dir = addr
            self.rgb_display_widget.show_img()
            return
            


    def start_mono_calib(self):
        try:
            fam = re.compile("[0-9]{4}|all")
            PREFIX = self.linedit_mono_prefix.text()
            res = fam.findall(PREFIX)
            if res[0] != PREFIX:
                raise ValueError("[error] Prefix must be of size 4(e.g., 0000) or all")
        except Exception as e:
            QtWidgets.QMessageBox.information(self, "[error]", str(e), QtWidgets.QMessageBox.Ok)
            return
        try:
            SUFFIX = self.linedit_mono_suffix.text()
            if (SUFFIX != "ir" and SUFFIX != "rgb" and SUFFIX != "ir_resize"):
                raise ValueError("[error] Suffix must be ir, rgb or ir_resize!")
        except Exception as e:
            QtWidgets.QMessageBox.information(self, "[error]", str(e), QtWidgets.QMessageBox.Ok) 
            return
        try:
            success_lst, fail_lst = self.mono_calib_obj.start(PREFIX, SUFFIX)
            QtWidgets.QMessageBox.information(self, "[info]", "[info] success: {}, fail: {}!".format(success_lst, fail_lst), QtWidgets.QMessageBox.Ok) 
        except Exception as e:
            QtWidgets.QMessageBox.information(self, "[error]", str(e), QtWidgets.QMessageBox.Ok) 


    def start_stereo_calib(self):
        try:
            fam = re.compile("[0-9]{4}|all")
            option = self.linedit_stereo_prefix.text()
            res = fam.findall(option)
            if res[0] != option:
                raise ValueError("[error] Prefix must be of size 4(e.g., 0000) or all")
        except Exception as e:
            QtWidgets.QMessageBox.information(self, "[error]", str(e), QtWidgets.QMessageBox.Ok) 
            return
        try:
            success_lst, fail_lst = self.stereo_calib_obj.start(option)
            QtWidgets.QMessageBox.information(self, "[info]", "[info] success: {}, fail: {}!".format(success_lst, fail_lst), QtWidgets.QMessageBox.Ok) 
        except Exception as e:
            QtWidgets.QMessageBox.information(self, "[error]", str(e), QtWidgets.QMessageBox.Ok) 


class TreeView(QtWidgets.QTreeView):
    def __init__(self, parent=None):
        super(TreeView, self).__init__(parent)
        self.__model = QtWidgets.QFileSystemModel()
        self.__model.setRootPath(QtCore.QDir.rootPath())
        self.setModel(self.__model)
        self.__current_select_path = None

    def set_path(self, path):
        self.setRootIndex(self.__model.index(path))

    def get_cur_path(self):
        return self.__current_select_path


class LoadingVideoWin(QtWidgets.QWidget):
    """
    Rewrite Qwidget to display movies.
    """
    def __init__(self, parent=None):
        super(LoadingVideoWin, self).__init__(parent)
        Vlayout = QtWidgets.QVBoxLayout()
        self.movie_dir = ""
        self.img_dir = ""
        self.label = QtWidgets.QLabel('', self)
        self.label.setGeometry(QtCore.QRect(0, 0, 984, 783))
        self.label.setMinimumSize(640,400)
        self.label.setScaledContents(True)
        self.videoflag = -1
        self.stop_flag = 1
        Vlayout.addWidget(self.label)
        self.setLayout(Vlayout)

    def show_img(self):
        try:
            frame = cv2.imread(self.img_dir, -1)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            # frame = cv2.resize(frame, (self.label.width(), self.label.height()))
            img = QtGui.QImage(frame.data, frame.shape[1], frame.shape[0], QtGui.QImage.Format_RGB888)

            pixmap = QtGui.QPixmap.fromImage(img)
            self.label.setPixmap(pixmap)
        except Exception as e:
            print(e)

    def set_movie(self):
        try:
            self.cap = cv2.VideoCapture(self.movie_dir)
          #  self.cap = cv2.VideoCapture(filepath)
            self.frameRate = self.cap.get(cv2.CAP_PROP_FPS)
            self.videoflag = 1
            thu = threading.Thread(target=self.Display)
            thu.start()
           # self.Display()
            #print("movie_dir = ", self.movie_dir)
        except Exception as e:
            print(e)

    def Display(self):
        while self.videoflag == 1:
            if self.cap.isOpened():
                self.stop_flag = 0
              #  print('1')
                success, frame = self.cap.read()
              #  print(2)
                self.picture = frame
                # RGB转BGR
                if success:
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    frame = cv2.resize(frame, (self.label.width(),self.label.height()))
                    img = QtGui.QImage(frame.data, frame.shape[1], frame.shape[0], QtGui.QImage.Format_RGB888)
                  #  print(4)
                    pixmap = QtGui.QPixmap.fromImage(img)
                   # print(5)
                    self.label.setPixmap(pixmap)
               #     print(6)
                   # self.label.setScaledContents(True)
                   # time.sleep(int(10000/self.frameRate)/1000)
                   # print(self.frameRate)
                    time.sleep(1/self.frameRate)
                else:
                    #print("read failed, no frame data")
                     pass
            else:
                print("open file or capturing device error, init again")
                self.reset()
        self.stop_flag = 1
        self.cap.release()


    def stop(self):
        """ Slot function to stop the movie. """
        self.videoflag = -1

