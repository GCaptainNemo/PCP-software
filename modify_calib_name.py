# #coding=utf-8

import os
import xml.etree.ElementTree as ET




def modify_collect_calib_launch():
    dom = ET.parse('./src/collect_data/launch/collect_calib.launch')
    root = dom.getroot()
    itemlist = root.findall('param')
    print(itemlist[1])
    tag_val = itemlist[1].items()
    for tag, val in tag_val:
        if tag == "value":
            args = val
    lst = args.split("_")
    calib_num = int(lst[1])
    calib_num += 1
    file_prefix = lst[0]
    itemlist[1].set("value", file_prefix + "_{}_".format(calib_num))
    dom.write("./src/collect_data/launch/collect_calib.launch") 
    



if __name__ == "__main__":
    modify_collect_calib_launch()
