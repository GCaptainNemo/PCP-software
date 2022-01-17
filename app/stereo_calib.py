#!/usr/bin/env python3
# coding=UTF-8
import os
import xml.etree.ElementTree as ET
import sys
import re
COLLECT_DATA_LAUNCH_ADDR = "/home/why/ROS_self/publish_collect_review_data/src/collect_data/launch/collect_data.launch"
STEREO_IMGS_XML_ADDR = "/home/why/ROS_self/publish_collect_review_data/src/review_data/src/calib_src/config/stereo_imgs.xml"
STEREO_SETTING_XML_ADDR = "/home/why/ROS_self/publish_collect_review_data/src/review_data/src/calib_src/config/stereo_setting.xml"
MONO_RGB_CALIB_ADDR = "/home/why/ROS_self/publish_collect_review_data/src/review_data/src/calib_src/config/param3.yaml"
DEBUG = False
debuglogger = lambda  a : (print(a) if DEBUG else ...)

	
SUFFIX = "ir" # ir/ir_resize

class InputError(Exception):
	def __init__(self, file_num):
		self.file_num = file_num
	
	def __str__(self):
		return "[error] {} stereo calib data(less than 1)".format(self.file_num)


class StereoCalib:
	def __init__(self):
		self.success_imgs_lst = []
		self.fail_imgs_lst = []

	def start(self, option):
		save_dir = self.get_save_dir()
		if option != "all": # prefix
			prefix = option
			pair_address_lst = self.get_calib_ir_rgb_address_lst(save_dir, prefix)
			if not pair_address_lst:
				raise InputError(0)
			self.write_stereo_imgs_xml(pair_address_lst)
			self.write_stereo_setting_xml(save_dir, prefix)
			res = self.run_stereo_calib_exe()
			if(res == 0):
				self.success_imgs_lst.append(prefix)
			else:
				self.fail_imgs_lst.append(prefix)

		else:
			prefix_count_dict = dict()
			ir_fam = re.compile("[0-9]{4}_[0-9]+__" + SUFFIX + "[.]jpg")
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
				pair_address_lst = self.get_calib_ir_rgb_address_lst(save_dir, prefix)
				self.write_stereo_imgs_xml(pair_address_lst)
				self.write_stereo_setting_xml(save_dir, prefix)
				res = self.run_stereo_calib_exe()
				if(res == 0):
					self.success_imgs_lst.append(prefix)
				else:
					self.fail_imgs_lst.append(prefix)
		print("success = ", self.success_imgs_lst)
		print("fail = ", self.fail_imgs_lst)
		return self.success_imgs_lst, self.fail_imgs_lst
				

	def get_save_dir(self):
		dom = ET.parse(COLLECT_DATA_LAUNCH_ADDR)
		root = dom.getroot()
		for node in root.findall('param'):
			if node.get("name") == "save_dir":
				save_dir = node.get("value")
				break
		return save_dir

	def get_calib_ir_rgb_address_lst(self, save_dir, prefix):
		"""
		get ir/rgb img pair address
		"""
		# ####################################################################
		# check mono ir calib data
		# ####################################################################
		mono_ir_calib_reg = prefix + "_monocalib_" + SUFFIX + "[.]yaml"
		ir_calib_fam = re.compile(mono_ir_calib_reg)
		for ir_file in os.listdir(save_dir):
			res_lst = ir_calib_fam.findall(ir_file)
			if res_lst:
				break
		if not res_lst:
			print("[error] ", prefix, "doesn't have intrinsic parameters!")
			return []

		# ####################################################################
		# get ir/rgb pair
		# ####################################################################
		ir_match_reg = prefix + "_[0-9]+__" + SUFFIX + "[.]jpg"
		ir_fam = re.compile(ir_match_reg)
		pair_address_lst = []
		file_lst = os.listdir(save_dir)
		for ir_file in file_lst:
			ir_res_lst = ir_fam.findall(ir_file)
			if ir_res_lst:
				rgb_match_reg = ir_res_lst[0].replace(SUFFIX, "rgb")
				if rgb_match_reg in file_lst:
					pair_address_lst.append(save_dir + ir_file + "    " + save_dir + rgb_match_reg)
		return pair_address_lst

	def write_stereo_imgs_xml(self, address_lst):
		dom = ET.parse(STEREO_IMGS_XML_ADDR)
		root = dom.getroot()
		
		images_node = root.find('images')
		images_node.text = "\n" + "\n".join(address_lst) + "\n"
		dom.write(STEREO_IMGS_XML_ADDR) 
		self.add_xml_declaration(STEREO_IMGS_XML_ADDR)

	def add_xml_declaration(self, addr):
		with open(addr, "r+") as f:
			lines = f.readlines()	
		lines.insert(0, '<?xml version="1.0"?>\n') 
		s = ''.join(lines) 
		debuglogger(s)               
		with open(addr, "w+") as f:
			f.write(s)

	def write_stereo_setting_xml(self, save_dir, prefix):
		# #######################################################################
		# check mono ir calib data
		# #######################################################################
		mono_ir_calib_reg = prefix + "_monocalib_" + SUFFIX + "[.]yaml"
		ir_calib_fam = re.compile(mono_ir_calib_reg)
		for file in os.listdir(save_dir):
			res_lst = ir_calib_fam.findall(file)
			if res_lst:
				break
		if not res_lst:
			print("[error] ", prefix, "doesn't have intrinsic parameters!")
			return
		ir_calib_addr = save_dir + file

		dom = ET.parse(STEREO_SETTING_XML_ADDR)
		root = dom.getroot()
		setting_node = root.find('Settings')
		output_node = setting_node.find("Write_outputFileName")		
		output_node.text = '"' + save_dir + prefix + '_stereocalib_' + SUFFIX + '.yaml"'
		ir_par_node = setting_node.find("InputIrParAddr")
		ir_par_node.text = ir_calib_addr
		rgb_par_node = setting_node.find("InputRgbParAddr")
		rgb_par_node.text = MONO_RGB_CALIB_ADDR
		dom.write(STEREO_SETTING_XML_ADDR) 
		self.add_xml_declaration(STEREO_SETTING_XML_ADDR)


	def run_stereo_calib_exe(self):
		res = os.system("rosrun review_data stereo_calib")
		res >>= 8  # high 8 bit is return value
		return res


if __name__ == "__main__":
	try:
		fam = re.compile("[0-9]{4}|all")
		option = sys.argv[1]
		res = fam.findall(option)
		if res[0] != option:
			raise -1
	except Exception as e:
		print("[error] please input prefix(e.g., 0000) or all!")
		exit(0)
	mono_ir_calib = StereoCalib()
	mono_ir_calib.start(option)


