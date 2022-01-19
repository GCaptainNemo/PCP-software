#!/usr/bin/env python3

from PyQt5.QtCore import QThread, pyqtSignal,pyqtSlot
import os
import ctypes
import inspect
# sonWidget and others


class ThreadStartRosbag(QThread):
    signal_finished = pyqtSignal(int)
    signal_started = pyqtSignal()
    def __init__(self, parent=None):
        super(ThreadStartRosbag, self).__init__(parent)
        self.record_time = None
        self.lidar_file_name = None

    def run(self):
        if self.record_time and self.lidar_file_name:
            self.signal_started.emit()
            os.system("rosbag record /livox/lidar/ --duration={} -O ".format(self.record_time) + self.lidar_file_name)
            self.signal_finished.emit(0)
        else:
            self.signal_finished.emit(1)



class ThreadOperator:
    @staticmethod
    def _async_raise(tid, exctype):
        """raises the exception, performs cleanup if needed"""
        tid = ctypes.c_long(tid)
        if not inspect.isclass(exctype):
            exctype = type(exctype)
        res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exctype))
        if res == 0:
            raise ValueError("[stop thread error] invalid thread id")
        elif res != 1:
            # """if it returns a number greater than one, you're in trouble,
            # and you should call it again with exc=NULL to revert the effect"""
            ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)
            raise SystemError("[stop thread error] PyThreadState_SetAsyncExc failed")

    @staticmethod
    def stop_thread(thread):
        """
        kill thread
        """
        ThreadOperator._async_raise(thread.ident, SystemExit)
