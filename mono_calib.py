#!/usr/bin/env python3
# coding=UTF-8
import os
import xml.etree.ElementTree as ET
import sys
import re
COLLECT_DATA_LAUNCH_ADDR = "/home/why/ROS_self/publish_collect_review_data/src/collect_data/launch/collect_data.launch"
MONO_IR_IMGS_XML_ADDR = "/home/why/ROS_self/publish_collect_review_data/src/review_data/src/calib_src/config/mono_ir_imgs.xml"
MONO_IR_SETTING_XML_ADDR = "/home/why/ROS_self/publish_collect_review_data/src/review_data/src/calib_src/config/mono_ir_setting.xml"

DEBUG = False
debuglogger = lambda  a : (print(a) if DEBUG else ...)

	


class MonoIrCalib:
	def __init__(self):
		self.success_imgs_lst = []
		self.fail_imgs_lst = []

	def start(self, option):
		save_dir = self.get_save_dir()
		if option != "all":
			prefix = option
			address_lst = self.get_calib_ir_address_lst(save_dir, prefix)
			if len(address_lst) < 3:
				print("[error] less calib data")
				exit(0)
			self.write_mono_ir_imgs_xml(address_lst)
			self.write_mono_ir_setting_xml(save_dir, prefix)
			res = self.run_mono_ir_calib_exe()
			if(res == 0):
				self.success_imgs_lst.append(prefix)
			else:
				self.fail_imgs_lst.append(prefix)

		else:
			prefix_count_dict = dict()
			ir_fam = re.compile("[0-9]{4}_[0-9]+__ir")
			for file in os.listdir(save_dir):
				res_lst = ir_fam.findall(file)
				if res_lst:
					prefix = res_lst[0][:4]
					val = prefix_count_dict.get(prefix, 0)
					prefix_count_dict[prefix] = val + 1
					continue
			for prefix, count in prefix_count_dict.items():
				if count < 3:
					print("[error]", prefix, " only have {} calib data (less than 3)".format(count))
					self.fail_imgs_lst.append(prefix)
					continue
				address_lst = self.get_calib_ir_address_lst(save_dir, prefix)
				self.write_mono_ir_imgs_xml(address_lst)
				self.write_mono_ir_setting_xml(save_dir, prefix)
				res = self.run_mono_ir_calib_exe()
				if(res == 0):
					self.success_imgs_lst.append(prefix)
				else:
					self.fail_imgs_lst.append(prefix)
		print("success = ", self.success_imgs_lst)
		print("fail = ", self.fail_imgs_lst)
				

	def get_save_dir(self):
		dom = ET.parse(COLLECT_DATA_LAUNCH_ADDR)
		root = dom.getroot()
		for node in root.findall('param'):
			if node.get("name") == "save_dir":
				save_dir = node.get("value")
				break
		return save_dir

	def get_calib_ir_address_lst(self, save_dir, prefix):
		
		ir_fam = re.compile(prefix + "_[0-9]+__ir")
		address_lst = []
		for file in os.listdir(save_dir):
			res_lst = ir_fam.findall(file)
			if res_lst:
				address_lst.append(save_dir + file)
				continue
		return address_lst

	def write_mono_ir_imgs_xml(self, address_lst):
		dom = ET.parse(MONO_IR_IMGS_XML_ADDR)
		root = dom.getroot()
		
		images_node = root.find('images')
		images_node.text = "\n" + "\n".join(address_lst) + "\n"
		dom.write(MONO_IR_IMGS_XML_ADDR) 
		self.add_xml_declaration(MONO_IR_IMGS_XML_ADDR)

	def add_xml_declaration(self, addr):
		with open(addr, "r+") as f:
			lines = f.readlines()	
		lines.insert(0, '<?xml version="1.0"?>\n') 
		s = ''.join(lines) 
		debuglogger(s)               
		with open(addr, "w+") as f:
			f.write(s)

	def write_mono_ir_setting_xml(self, save_dir, prefix):
		dom = ET.parse(MONO_IR_SETTING_XML_ADDR)
		root = dom.getroot()
		setting_node = root.find('Settings')
		output_node = setting_node.find("Write_outputFileName")
		
		print(output_node.text)
		output_node.text = '"' + save_dir + prefix + '_monocalib.yaml"'
		dom.write(MONO_IR_SETTING_XML_ADDR) 
		self.add_xml_declaration(MONO_IR_SETTING_XML_ADDR)


	def run_mono_ir_calib_exe(self):
		res = os.system("rosrun review_data mono_ir_calib")
		res >>= 8  # high 8 bit is return value
		return res


if __name__ == "__main__":
	try:
		option = sys.argv[1]
	except Exception as e:
		print("[error] please input prefix or all!")
		exit(0)
	mono_ir_calib = MonoIrCalib()
	mono_ir_calib.start(option)

