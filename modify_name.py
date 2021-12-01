# #coding=utf-8

import os
import xml.etree.ElementTree as ET


def getFileName(save_dir):
    index = 1
    while True:
        prefix = str(index).zfill(4)
        rgb_dir = save_dir + prefix + "_rgb.jpg"
        ir_dir = save_dir + prefix + "_ir.jpg"
        lidar_dir = save_dir + prefix + "_lidar.bag"
        if os.path.exists(rgb_dir) or os.path.exists(ir_dir) or os.path.exists(lidar_dir):
            index += 1
        else:
            break
    return prefix


def main():
    dom = ET.parse('./src/collect_data/launch/collect_data.launch')
    #得到文档元素对象
    print(dom)
    root = dom.getroot()
    print(root)

    itemlist = root.findall('param')
    print(itemlist)
    
    # get save dir
    tag_val = itemlist[0].items()
    for tag, val in tag_val:
        if tag == "value":
            save_dir = val
    print("save_dir = ", save_dir)
    if not os.path.exists(save_dir):
        return -1
    
    # get file name
    file_prefix = getFileName(save_dir)
    print("file_prefix = ", file_prefix)
    itemlist[1].set("value", file_prefix)

    # ######################################################
    item_lst = root.findall('node')
    tag_val = item_lst[2].items()
    for tag, val in tag_val:
        if tag == "args":
            args = val
    lst = args.split("-O ")
    new_args = lst[0] + "-O " + save_dir + file_prefix + "_lidar"
    item_lst[2].set("args", new_args)
    # save
    dom.write("./src/collect_data/launch/collect_data.launch")   

if __name__ == "__main__":
    res = main()

