#!/usr/bin/env python3
# coding=UTF-8
# ir_img_size: [640, 480]  ==crop==> [640, 360] ==resize==> [1920, 1080]
# rgb_img_size: [1920, 1080]
import os
import xml.etree.ElementTree as ET
import sys
import re
import cv2
import shutil

COLLECT_DATA_LAUNCH_ADDR = "/home/why/ROS_self/publish_collect_review_data/src/collect_data/launch/collect_data.launch"
MONO_IR_IMGS_XML_ADDR = "/home/why/ROS_self/publish_collect_review_data/src/review_data/src/calib_src/config/mono_imgs.xml"
MONO_IR_SETTING_XML_ADDR = "/home/why/ROS_self/publish_collect_review_data/src/review_data/src/calib_src/config/mono_setting.xml"

DEBUG = False
debuglogger = lambda  a : (print(a) if DEBUG else ...)

class InputError(Exception):
	def __init__(self, file_num):
		self.file_num = file_num
	
	def __str__(self):
		return "[error] {} calib data(less than 3)".format(self.file_num)


# #############################################
# prefix(e.g., 0000, 0001), suffix(e.g, ir/rgb/ir_resize)
# ############################################3

class MonoIrCalib:
	def __init__(self):
		self.success_imgs_lst = []
		self.fail_imgs_lst = []

	def start(self, PREFIX, SUFFIX):
		save_dir = self.get_save_dir()
		if PREFIX != "all":
			prefix = PREFIX
			address_lst = self.get_calib_img_address_lst(save_dir, prefix, SUFFIX)
			if len(address_lst) < 3:
				raise InputError(len(address_lst))
			self.write_mono_imgs_xml(address_lst)
			self.write_mono_setting_xml(save_dir, prefix, SUFFIX)
			res = self.run_mono_calib_exe()
			if(res == 0):
				self.success_imgs_lst.append(prefix)
			else:
				self.fail_imgs_lst.append(prefix)

		else:
			prefix_count_dict = dict()
			img_fam = re.compile("[0-9]{4}_[0-9]+__" + SUFFIX + "[.]jpg")
			for file in os.listdir(save_dir):
				res_lst = img_fam.findall(file)
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
				address_lst = self.get_calib_img_address_lst(save_dir, prefix, SUFFIX)
				self.write_mono_imgs_xml(address_lst)
				self.write_mono_setting_xml(save_dir, prefix, SUFFIX)
				res = self.run_mono_calib_exe()
				if(res == 0):
					self.success_imgs_lst.append(prefix)
				else:
					self.fail_imgs_lst.append(prefix)
		
		# print("success = ", self.success_imgs_lst)
		# print("fail = ", self.fail_imgs_lst)
		return self.success_imgs_lst, self.fail_imgs_lst

	def get_save_dir(self):
		dom = ET.parse(COLLECT_DATA_LAUNCH_ADDR)
		root = dom.getroot()
		for node in root.findall('param'):
			if node.get("name") == "save_dir":
				save_dir = node.get("value")
				break
		return save_dir

	def get_calib_img_address_lst(self, save_dir, prefix, suffix):
		
		ir_fam = re.compile(prefix + "_[0-9]+__" + suffix + "[.]jpg")
		address_lst = []
		for file in os.listdir(save_dir):
			res_lst = ir_fam.findall(file)
			if res_lst:
				address_lst.append(save_dir + file)
				continue
		return address_lst

	def write_mono_imgs_xml(self, address_lst):
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

	def write_mono_setting_xml(self, save_dir, prefix, suffix):
		dom = ET.parse(MONO_IR_SETTING_XML_ADDR)
		root = dom.getroot()
		setting_node = root.find('Settings')
		output_node = setting_node.find("Write_outputFileName")
		output_node.text = '"' + save_dir + prefix + '_monocalib_' + suffix + '.yaml"'
		is_ir_node = setting_node.find("Input_IR")

		if(suffix == "ir" or suffix == "ir_resize"):
			is_ir_node.text = "1"
		elif(suffix == "rgb"):
			is_ir_node.text = "0"

		dom.write(MONO_IR_SETTING_XML_ADDR) 
		self.add_xml_declaration(MONO_IR_SETTING_XML_ADDR)


	def run_mono_calib_exe(self):
		"""
		rosrun ros node
		"""
		res = os.system("rosrun review_data mono_calib")
		res >>= 8  # high 8 bit is return value
		return res

	@staticmethod
	def resize(input_ir):
		# print(input_ir.shape)
		output_ir = input_ir[60:420, :] # ir [640, 480] -> [640, 360]
		output_ir = cv2.resize(output_ir, (1920, 1080))
		# print(output_ir.shape)
		# cv2.imshow("resize", output_ir)
		# cv2.waitKey(0)

		return output_ir
	
	@staticmethod
	def output_resize_ir(file_dir = "/media/why/LENOVO_USB_HDD/ir_rgb_registration_data_collect/"):
		fam = re.compile("ir.jpg$")
		
		for file in os.listdir(file_dir):
			res = fam.findall(file)
			if res:
				ir_file_addr = file_dir + file
				print(file_dir + file)
				ir = cv2.imread(ir_file_addr, -1)
				save_ir = MonoIrCalib.resize(ir)
				save_dir = file_dir + file.replace("ir.jpg", "ir_resize.jpg")
				cv2.imwrite(save_dir, save_ir)
		

if __name__ == "__main__":
	
	try:
		fam = re.compile("[0-9]{4}|all")
		PREFIX = sys.argv[1]
		res = fam.findall(PREFIX)
		if res[0] != PREFIX:
			raise -1
	except Exception as e:
		print("[error] please input prefix(e.g., 0000) or all!")
		exit(0)
	try:
		SUFFIX = sys.argv[2]
		if (SUFFIX != "ir" and SUFFIX != "rgb" and SUFFIX != "ir_resize"):
			raise -1
	except Exception as e:
		print("[error] please input ir, rgb or ir_resize!")
		exit(0)
	mono_ir_calib = MonoIrCalib()
	mono_ir_calib.start(PREFIX, SUFFIX)

