#!/usr/bin/env python3
#coding=utf-8

import os
import xml.etree.ElementTree as ET
COLLECT_DATA_LAUNCH_ADDR = "/home/why/ROS_self/publish_collect_review_data/src/collect_data/launch/collect_data.launch"
COLLECT_CALIB_LAUNCH_ADDR = "/home/why/ROS_self/publish_collect_review_data/src/collect_data/launch/collect_calib.launch"


def getFileName(save_dir):
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
        else:
            index -= 1
    return prefix


def modify_collect_calib_launch(file_prefix):
    dom = ET.parse(COLLECT_CALIB_LAUNCH_ADDR)
    root = dom.getroot()
    for node in root.findall('param'):
        if node.get("name") == "prefix":
            node.set("value", file_prefix)
            break
    dom.write(COLLECT_CALIB_LAUNCH_ADDR) 
    


def main():
    dom = ET.parse(COLLECT_DATA_LAUNCH_ADDR)
    root = dom.getroot()
    for node in root.findall('param'):
        if node.get("name") == "save_dir":
            save_dir = node.get("value")
            break
    # get save dir
    if not os.path.exists(save_dir):
        print("[error]", save_dir, "not exists!")
        return -1
    
    # get file name
    file_prefix = getFileName(save_dir)
    print("file_prefix = ", file_prefix)
    for node in root.findall('param'):
        if node.get("name") == "prefix":
            break
    node.set("value", file_prefix)
    modify_collect_calib_launch(file_prefix) 

    # ######################################################
    for node in root.findall('node'):
        if node.get("pkg") == "rosbag":
            break
    args = node.get("args")
    lst = args.split("-O ")
    new_args = lst[0] + "-O " + save_dir + file_prefix + "_lidar"
    node.set("args", new_args)
    # save
    dom.write(COLLECT_DATA_LAUNCH_ADDR)  
    

if __name__ == "__main__":
    res = main()

