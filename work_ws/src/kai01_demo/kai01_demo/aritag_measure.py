#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import time
import math
import collections

# ROS2 messages
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped

# ROS2 core
import rclpy
from rclpy.node import Node

# TF2
from tf2_ros import TransformBroadcaster

# AprilTag detection
from dt_apriltags import Detector

# CV bridge
from cv_bridge import CvBridge

	# ========== 绘制tag角点和中心点 ==========
def draw_tags(image, tags, corners_color=(0, 0, 255), center_color=(0, 255, 0)):
	"""绘制AprilTag的四个角点和中心点
	
	参数:
		image: 要绘制的图像
		tags: AprilTag检测结果列表
		corners_color: 角点颜色 BGR格式, 默认红色
		center_color: 中心点颜色 BGR格式, 默认绿色
	"""
	for tag in tags:
		# 绘制四个角点
		corners = tag.corners.astype(int)
		for corner in corners:
			cv2.circle(image, tuple(corner), 5, corners_color, -1)
		
		# 绘制中心点
		center = tag.center.astype(int)
		cv2.circle(image, tuple(center), 6, center_color, -1)
		cv2.circle(image, tuple(center), 8, center_color, 2)
class AprilTagMeasureNode(Node):
	def __init__(self, name):
		super().__init__(name)
		
		# ========== 参数声明 ==========
		self.declare_parameter("tag_size", 0.035)  # AprilTag
		self.declare_parameter("camera_frame_id", "camera_color_optical_frame")  # 相机光学坐标系名称
		
		# 从参数加载
		self.tag_size = self.get_parameter("tag_size").get_parameter_value().double_value
		self.camera_frame_id = self.get_parameter("camera_frame_id").get_parameter_value().string_value
		
		# ========== 初始化状态变量 ==========
		self.bridge = CvBridge()
		self.pr_time = time.time()
		self.fps_buffer = collections.deque(maxlen=10)
		self.camera_K = None       # 相机内参矩阵(3x3)
		self.camera_params = None  # [fx, fy, cx, cy]
		self.scale_x = 1.0
		self.scale_y = 1.0
		self._display_error_logged = False
		
		# ========== AprilTag检测器 ==========
		self.at_detector = Detector(
			searchpath=['apriltags'],
			families='tag36h11',
			nthreads=8,
			quad_decimate=1.0,
			quad_sigma=0.0,
			refine_edges=1,
			decode_sharpening=0.5,
			debug=0
		)
		
		# ========== 相机内参订阅 ==========
		self.camera_info_sub = self.create_subscription(
			CameraInfo,
			'/camera/color/camera_info',
			self.camera_info_callback,
			10
		)
		
		# ========== RGB图像订阅 ==========
		self.rgb_sub = self.create_subscription(
			Image,
			'/camera/color/image_raw',
			self.rgb_callback,
			10
		)
		
		# ========== TF广播器 ==========
		self.tf_broadcaster = TransformBroadcaster(self)
		
		# ========== 显示窗口 ==========
		try:
			cv2.namedWindow("aritag_measure", cv2.WINDOW_AUTOSIZE)
			self.get_logger().info("Display window created successfully")
		except Exception as e:
			self.get_logger().warn(f"Failed to create display window: {e}")
		
		self.get_logger().info("AprilTag Measure Node initialized (RGB only, no depth)")
		self.get_logger().info(f"Tag size: {self.tag_size}m, Camera frame: {self.camera_frame_id}")

	# ========== 相机内参回调 ==========
	def camera_info_callback(self, msg):
		"""相机内参回调(只获取一次)"""
		if self.camera_K is None:
			self.camera_K = np.array(msg.k).reshape(3, 3)
			fx = self.camera_K[0, 0]
			fy = self.camera_K[1, 1]
			cx = self.camera_K[0, 2]
			cy = self.camera_K[1, 2]
			self.camera_params = [fx, fy, cx, cy]
			# 计算缩放比(原始图像 -> 640x480)
			self.scale_x = 640.0 / msg.width
			self.scale_y = 480.0 / msg.height
			self.get_logger().info(
				f"Camera intrinsics received: fx={fx:.2f}, fy={fy:.2f}, "
				f"cx={cx:.2f}, cy={cy:.2f}, scale=({self.scale_x:.3f}, {self.scale_y:.3f})"
			)

	# ========== 绘制坐标轴 ==========
	def _draw_tag_axes(self, image, tag):
		"""在tag中心绘制3D坐标轴(X=红, Y=绿, Z=蓝)
		
		使用solvePnP求解位姿，然后投影坐标轴到图像上
		"""
		if self.camera_K is None:
			return None
		
		half = self.tag_size / 2.0
		# tag四角3D坐标 (tag坐标系, Z=0平面)
		obj_points = np.array([
			[-half, -half, 0.0],
			[ half, -half, 0.0],
			[ half,  half, 0.0],
			[-half,  half, 0.0],
		], dtype=np.float64)
		
		# 检测到的角点像素坐标
		img_points = tag.corners.astype(np.float64)
		
		# 使用缩放后的内参(匹配640x480图像)
		K_scaled = self.camera_K.copy()
		K_scaled[0, 0] *= self.scale_x
		K_scaled[0, 2] *= self.scale_x
		K_scaled[1, 1] *= self.scale_y
		K_scaled[1, 2] *= self.scale_y
		
		# solvePnP求解位姿
		success, rvec, tvec = cv2.solvePnP(obj_points, img_points, K_scaled, None)
		if not success:
			return None
		
		# 绘制坐标轴 (长度=tag边长)
		axis_length = self.tag_size
		cv2.drawFrameAxes(image, K_scaled, None, rvec, tvec, axis_length, 3)
		
		# 发布TF变换
		self._publish_tag_tf(tag, rvec, tvec)
		
		return tvec



	# ========== 发布tag的TF变换 ==========
	def _publish_tag_tf(self, tag, rvec, tvec):
		"""发布tag到camera_frame_id坐标系的TF变换
		
		参数:
			tag: AprilTag检测结果
			rvec: solvePnP返回的旋转向量
			tvec: solvePnP返回的平移向量
		"""
		# 旋转向量转旋转矩阵再转四元数
		R, _ = cv2.Rodrigues(rvec)
		# 3x3旋转矩阵转四元数
		q = self._rotation_matrix_to_quaternion(R)
		
		# 构建TF消息
		t = TransformStamped()
		t.header.stamp = self.get_clock().now().to_msg()
		t.header.frame_id = self.camera_frame_id
		t.child_frame_id = f"tag_{tag.tag_id}"
		
		# 平移 (solvePnP的tvec单位是米)
		t.transform.translation.x = float(tvec[0][0])
		t.transform.translation.y = float(tvec[1][0])
		t.transform.translation.z = float(tvec[2][0])
		
		# 旋转
		t.transform.rotation.x = q[0]
		t.transform.rotation.y = q[1]
		t.transform.rotation.z = q[2]
		t.transform.rotation.w = q[3]
		
		self.tf_broadcaster.sendTransform(t)

	def _rotation_matrix_to_quaternion(self, R):
		"""3x3旋转矩阵转四元数 [x, y, z, w]"""
		trace = R[0, 0] + R[1, 1] + R[2, 2]
		if trace > 0:
			s = 0.5 / math.sqrt(trace + 1.0)
			w = 0.25 / s
			x = (R[2, 1] - R[1, 2]) * s
			y = (R[0, 2] - R[2, 0]) * s
			z = (R[1, 0] - R[0, 1]) * s
		elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
			s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
			w = (R[2, 1] - R[1, 2]) / s
			x = 0.25 * s
			y = (R[0, 1] + R[1, 0]) / s
			z = (R[0, 2] + R[2, 0]) / s
		elif R[1, 1] > R[2, 2]:
			s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
			w = (R[0, 2] - R[2, 0]) / s
			x = (R[0, 1] + R[1, 0]) / s
			y = 0.25 * s
			z = (R[1, 2] + R[2, 1]) / s
		else:
			s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
			w = (R[1, 0] - R[0, 1]) / s
			x = (R[0, 2] + R[2, 0]) / s
			y = (R[1, 2] + R[2, 1]) / s
			z = 0.25 * s
		return [x, y, z, w]

	# ========== RGB图像回调 ==========
	def rgb_callback(self, msg):
		"""主回调: AprilTag检测+位姿估计+显示"""
		if self.camera_K is None:
			self.get_logger().warn_throttle(5.0, "Waiting for camera intrinsics...")
			return
		
		# 1. 图像预处理
		rgb_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
		result_image = cv2.resize(rgb_image, (640, 480))
		
		# 2. AprilTag检测 + 位姿估计
		gray = cv2.cvtColor(result_image, cv2.COLOR_RGB2GRAY)
		tags = self.at_detector.detect(
			gray,
			estimate_tag_pose=True,
			camera_params=self.camera_params,
			tag_size=self.tag_size
		)
		tags = sorted(tags, key=lambda t: t.tag_id)
		
		# 3. 绘制角点和中心点
		draw_tags(result_image, tags, corners_color=(0, 0, 255), center_color=(0, 255, 0))
		
		# 4. 绘制坐标轴 + 显示坐标和距离
		self._display_tag_pose(tags, result_image)
		
		# 5. 右上角显示FPS
		self._display_fps(result_image)
		
		# 6. 显示图像
		try:
			display = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
			cv2.imshow("aritag_measure", display)
			cv2.waitKey(1)
		except Exception as e:
			if not self._display_error_logged:
				self.get_logger().warn(f"Display failed: {e}")
				self._display_error_logged = True

	# ========== 显示tag位姿信息 ==========
	def _display_tag_pose(self, tags, result_image):
		"""绘制坐标轴并显示tag在相机坐标系下的坐标和距离"""
		y_offset = 30
		
		if len(tags) == 0:
			cv2.putText(result_image, "No AprilTag detected", (10, y_offset),
					   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
			return
		
		cv2.putText(result_image, f"Frame: {self.camera_frame_id}", (10, y_offset),
				   cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
		y_offset += 22
		
		for tag in tags:
			# 绘制坐标轴并获取平移向量
			tvec = self._draw_tag_axes(result_image, tag)
			if tvec is None:
				continue
			
			cx, cy, cz = tvec[0][0], tvec[1][0], tvec[2][0]
			# dist = math.sqrt(cx**2 + cy**2 + cz**2)
			dist = cz
			# tag中心附近显示距离
			center_px = tag.center.astype(int)
			cv2.putText(result_image, f"d={dist*1000:.0f}mm",
					   (center_px[0]+15, center_px[1]-10),
					   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
			
			# 顶部信息栏: 坐标系 + 坐标值 + 距离
			info = f"ID{tag.tag_id}: [{cx:.3f},{cy:.3f},{cz:.3f}]m  d={dist*1000:.1f}mm"
			cv2.putText(result_image, info, (10, y_offset),
					   cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
			y_offset += 22

	# ========== 右上角FPS ==========
	def _display_fps(self, result_image):
		"""在右上角显示帧率"""
		cur_time = time.time()
		time_diff = cur_time - self.pr_time
		self.pr_time = cur_time
		
		if time_diff > 0:
			self.fps_buffer.append(1.0 / time_diff)
		
		fps = int(sum(self.fps_buffer) / len(self.fps_buffer)) if self.fps_buffer else 0
		fps_text = f"FPS: {fps}"
		h, w = result_image.shape[:2]
		text_size = cv2.getTextSize(fps_text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0]
		cv2.putText(result_image, fps_text, (w - text_size[0] - 10, 30),
				   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)


def main():
	rclpy.init()
	node = AprilTagMeasureNode('aritag_measure_node')
	node.get_logger().info("AprilTag Measure Node started")
	rclpy.spin(node)
