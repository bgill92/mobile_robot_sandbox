#!/usr/bin/env python3

import math

from read_bag_data import read_bag_data

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

from scipy.spatial.transform import Rotation as R

def get_base_link_tf_data(topics_and_data):
	base_link_tfs = []
	for msg in topics_and_data['/tf']:
		for transform in msg.transforms:
			if transform.child_frame_id == 'base_link':
				base_link_tfs.append(transform.transform)

	return base_link_tfs

def convert_transforms_to_x_y_yaw(transforms):
	data = []
	for msg in transforms:
		r = R.from_quat([msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w])
		data.append([msg.translation.x, msg.translation.y, r.as_euler('xyz')[2]])

	return data

def plot_base_link_transforms(fig, transforms):
	data = convert_transforms_to_x_y_yaw(transforms)

	arrow_length = 0.05

	ax = fig.gca()

	for x, y, yaw in data[::10]:
		dx = arrow_length * math.cos(yaw)
		dy = arrow_length * math.sin(yaw)

		ax.arrow(x, y, dx, dy, head_width=0.01, length_includes_head=True, color='blue')

def get_odom_data(topics_and_data):
	odom_data = []
	for msg in topics_and_data['/odom']:
		odom_data.append(msg.pose)

	return odom_data

def convert_odom_to_x_y_yaw(odom_data):
	data = []
	for msg in odom_data:
		r = R.from_quat([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
		data.append([msg.pose.position.x, msg.pose.position.y, r.as_euler('xyz')[2]])

	return data

def plot_odom(fig, odom_data):
	data = convert_odom_to_x_y_yaw(odom_data)

	arrow_length = 0.05

	ax = fig.gca()

	for x, y, yaw in data[::10]:
		dx = arrow_length * math.cos(yaw)
		dy = arrow_length * math.sin(yaw)

		ax.arrow(x, y, dx, dy, head_width=0.01, length_includes_head=True, color='red')

if __name__ == '__main__':
	bag_path = "./rosbag2_2025_01_02-05_29_39/rosbag2_2025_01_02-05_29_39_0.db3"

	topics_and_data = read_bag_data(bag_path)

	fig = plt.gcf()

	base_link_tfs = get_base_link_tf_data(topics_and_data)

	plot_base_link_transforms(fig, base_link_tfs)

	odom_data = get_odom_data(topics_and_data)

	plot_odom(fig, odom_data)

	plt.axis("equal")  # Equal scaling on both axes
	plt.grid(True)
	plt.show()
