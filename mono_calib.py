#!/usr/bin/env python3
# coding=UTF-8
import os
import xml.etree.ElementTree as ET
import sys
import re
COLLECT_DATA_LAUNCH_ADDR = "/home/why/ROS_self/publish_collect_review_data/src/collect_data/launch/collect_data.launch"
IMGS_XML_CONFIG_ADDR = "/home/why/pg_cpp/Camera-Calib-OpenCV/config/imgs.xml"
IR_CHESSBOARD_XML_ADDR = "/home/why/pg_cpp/Camera-Calib-OpenCV/config/IR_chessboard.xml"


def get_save_dir():
    dom = ET.parse(COLLECT_DATA_LAUNCH_ADDR)
    root = dom.getroot()
    for node in root.findall('param'):
        if node.get("name") == "save_dir":
            save_dir = node.get("value")
            break
    return save_dir

def get_calib_ir_address_lst(save_dir):
	try:
		prefix = sys.argv[1]
	except Exception as e:
		print("[error] please input prefix!")
		exit(0)
	ir_fam = re.compile(prefix + "_[0-9]+__ir")
	address_lst = []
	for file in os.listdir(save_dir):
		res_lst = ir_fam.findall(file)
		if res_lst:
			address_lst.append(save_dir + file)
			continue
	return address_lst

def write_imgs_xml(address_lst):
	dom = ET.parse(IMGS_XML_CONFIG_ADDR)
	root = dom.getroot()
	
	images_node = root.find('images')
		# print(node.text)
	images_node.text = "\n" + "\n".join(address_lst) + "\n"
	print(images_node.text)
	dom.write(IMGS_XML_CONFIG_ADDR) 
	add_xml_declaration(IMGS_XML_CONFIG_ADDR)

def add_xml_declaration(addr):
	with open(addr, "r+") as f:
		lines = f.readlines()	
	lines.insert(0, '<?xml version="1.0"?>\n') 
	s = ''.join(lines) 
	print(s)               
	with open(addr, "w+") as f:
		f.write(s)

def write_IR_chessboard_xml(save_dir, prefix):
	dom = ET.parse(IR_CHESSBOARD_XML_ADDR)
	root = dom.getroot()
	setting_node = root.find('Settings')
	output_node = setting_node.find("Write_outputFileName")
	
	print(output_node.text)
	output_node.text = '"' + save_dir + prefix + '_monocalib.yaml"'
	# dom.write(IR_CHESSBOARD_XML_ADDR, encoding="utf-8", xml_declaration=True) 
	dom.write(IR_CHESSBOARD_XML_ADDR) 
	add_xml_declaration(IR_CHESSBOARD_XML_ADDR)

	

def main():
	save_dir = get_save_dir()
	address_lst = get_calib_ir_address_lst(save_dir)
	if len(address_lst) < 3:
		print("[error] less calib data")
		exit(0)
	write_imgs_xml(address_lst)
	write_IR_chessboard_xml(save_dir, sys.argv[1])


if __name__ == "__main__":
	main()

