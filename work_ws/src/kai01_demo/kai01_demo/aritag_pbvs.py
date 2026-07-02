#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import time
import math
import collections

# ROS2 messages
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped, TwistStamped

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
		
		# PBVS 参数
		self.declare_parameter("target_tag_id", 2)          # 用于伺服的目标tag ID
		self.declare_parameter("desired_distance", 0.25)    # 期望距离(米): 相机到tag的Z轴距离
		self.declare_parameter("lambda_gain", 0.4)          # PBVS增益 λ (控制收敛速度)
		self.declare_parameter("max_linear_vel", 0.08)      # 最大线速度(米/秒)
		self.declare_parameter("max_angular_vel", 0.5)      # 最大角速度(弧度/秒)
		self.declare_parameter("position_tolerance", 0.005) # 位置容差(米), 小于此值认为收敛
		self.declare_parameter("angle_tolerance", 0.05)     # 角度容差(弧度)
		
		# 从参数加载
		self.tag_size = self.get_parameter("tag_size").get_parameter_value().double_value
		self.camera_frame_id = self.get_parameter("camera_frame_id").get_parameter_value().string_value
		self.target_tag_id = self.get_parameter("target_tag_id").get_parameter_value().integer_value
		self.desired_distance = self.get_parameter("desired_distance").get_parameter_value().double_value
		self.lambda_gain = self.get_parameter("lambda_gain").get_parameter_value().double_value
		self.max_linear_vel = self.get_parameter("max_linear_vel").get_parameter_value().double_value
		self.max_angular_vel = self.get_parameter("max_angular_vel").get_parameter_value().double_value
		self.position_tolerance = self.get_parameter("position_tolerance").get_parameter_value().double_value
		self.angle_tolerance = self.get_parameter("angle_tolerance").get_parameter_value().double_value
		
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
		
		# ========== PBVS 控制状态 ==========
		self.pbvs_active = False          # PBVS控制是否激活
		self.current_rvec = None          # 当前帧的旋转向量
		self.current_tvec = None          # 当前帧的平移向量
		self.converged = False            # 是否已收敛
		
		# ========== PBVS Twist 发布器 (直接发布真实速度, 不经过servo_node) ==========
		self.twist_pub = self.create_publisher(
			TwistStamped,
			'/delta_twist_cmds',
			10
		)
		
		# ========== 显示窗口 ==========
		try:
			cv2.namedWindow("aritag_measure", cv2.WINDOW_AUTOSIZE)
			self.get_logger().info("Display window created successfully")
		except Exception as e:
			self.get_logger().warn(f"Failed to create display window: {e}")
		
		self.get_logger().info("AprilTag Measure Node initialized (RGB only, no depth)")
		self.get_logger().info(f"Tag size: {self.tag_size}m, Camera frame: {self.camera_frame_id}")
		self.get_logger().info(f"PBVS target tag: {self.target_tag_id}, desired distance: {self.desired_distance}m")
		self.get_logger().info(f"PBVS lambda: {self.lambda_gain}, max vel: lin={self.max_linear_vel} ang={self.max_angular_vel}")
		self.get_logger().info("验证模式: 直接发布六维速度到 /delta_twist_cmds")

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

	# ========== PBVS 控制核心 ==========
	def _compute_pbvs_velocity(self, rvec, tvec):
		"""PBVS控制律: 根据当前位姿误差计算相机期望速度
		
		原理 (Chaumette & Hutchinson, 2006):
		  误差 e = [t_err; θu_err] (6维)
		  交互矩阵 L_e = [[I3, [t_err]_x]; [0, L_θu]]
		  控制律 v = -λ * L_e^{-1} * e
		
		参数:
			rvec: solvePnP返回的旋转向量 (3x1)
			tvec: solvePnP返回的平移向量 (3x1), 单位:米
		
		返回:
			v: 6维相机速度 [vx, vy, vz, wx, wy, wz]
			   坐标系: 相机光学坐标系 (ROS: X右,Y下,Z前)
		"""
		# --- 1. 当前位姿 ---
		t_cur = tvec.flatten()              # (3,) 当前tag在相机系的位置
		R_cur, _ = cv2.Rodrigues(rvec)       # (3x3) tag->相机 旋转矩阵
		
		# --- 2. 期望位姿 ---
		# 期望: tag在相机正前方 desired_distance 处, 相机正对tag (无旋转偏差)
		# 在相机坐标系下: tag中心在 (0, 0, desired_distance)
		t_des = np.array([0.0, 0.0, self.desired_distance])
		R_des = np.eye(3)  # 期望旋转为单位矩阵(相机正对tag)
		
		# --- 3. 位姿误差 ---
		# 平移误差
		t_err = t_cur - t_des  # (3,)
		
		# 旋转误差: R_err = R_cur @ R_des^T = R_cur
		# 转为 angle-axis 表示
		R_err = R_cur  # 因为 R_des = I
		theta, u = self._rotation_to_angle_axis(R_err)
		theta_u_err = theta * u  # (3,) 旋转误差
		
		# 完整误差向量 e = [t_err; theta_u_err]
		e = np.concatenate([t_err, theta_u_err])  # (6,)
		
		# --- 4. 构建交互矩阵 L_e (6x6) ---
		# L_e = [[I3, skew(t_err)],
		#        [0,  L_theta_u   ]]
		L_e = np.zeros((6, 6))
		L_e[:3, :3] = np.eye(3)
		L_e[:3, 3:] = self._skew_symmetric(t_err)
		L_e[3:, 3:] = self._L_theta_u(theta, u)
		
		# --- 5. 控制律: v = -λ * L_e^{-1} * e ---
		try:
			L_e_inv = np.linalg.inv(L_e)
		except np.linalg.LinAlgError:
			return None
		
		v = -self.lambda_gain * L_e_inv @ e  # (6,) 相机速度
		
		# --- 6. 速度限幅 ---
		v_lin = v[:3]
		v_ang = v[3:]
		lin_norm = np.linalg.norm(v_lin)
		ang_norm = np.linalg.norm(v_ang)
		
		if lin_norm > self.max_linear_vel:
			v_lin = v_lin * (self.max_linear_vel / lin_norm)
		if ang_norm > self.max_angular_vel:
			v_ang = v_ang * (self.max_angular_vel / ang_norm)
		
		v = np.concatenate([v_lin, v_ang])
		
		return v, t_err, theta_u_err
	
	@staticmethod
	def _skew_symmetric(v):
		"""构造3x3反对称矩阵 [v]_x"""
		return np.array([
			[0,    -v[2],  v[1]],
			[v[2],  0,    -v[0]],
			[-v[1], v[0],  0   ]
		])
	
	@staticmethod
	def _rotation_to_angle_axis(R):
		"""旋转矩阵转 angle-axis 表示
		
		返回:
			theta: 旋转角度(弧度)
			u: 旋转轴(单位向量, 3维)
		"""
		trace = np.trace(R)
		cos_theta = np.clip((trace - 1.0) / 2.0, -1.0, 1.0)
		theta = np.arccos(cos_theta)
		
		if abs(theta) < 1e-10:
			# 几乎无旋转
			return 0.0, np.array([0.0, 0.0, 1.0])
		
		if abs(theta - np.pi) < 1e-6:
			# theta ≈ π, 需要特殊处理
			# 找 R+I 中范数最大的列
			M = R + np.eye(3)
			col_norms = [np.linalg.norm(M[:, i]) for i in range(3)]
			best = np.argmax(col_norms)
			u = M[:, best] / np.linalg.norm(M[:, best])
		else:
			# 标准公式
			u = np.array([
				R[2, 1] - R[1, 2],
				R[0, 2] - R[2, 0],
				R[1, 0] - R[0, 1]
			]) / (2.0 * np.sin(theta))
		
		return theta, u
	
	@staticmethod
	def _L_theta_u(theta, u):
		"""计算旋转误差的交互矩阵 L_{θu} (3x3)
		
		公式 (Chaumette 2004):
		  L_{θu} = I3 + (θ/2)[u]_x + (1 - sinc(θ)/sinc²(θ/2)) * [u]_x²
		其中 sinc(x) = sin(x)/x
		"""
		I3 = np.eye(3)
		skew_u = AprilTagMeasureNode._skew_symmetric(u)
		skew_u2 = skew_u @ skew_u  # [u]_x²
		
		if abs(theta) < 1e-10:
			return I3
		
		# 计算系数 (1 - sinc(θ)/sinc²(θ/2))
		sinc_theta = np.sin(theta) / theta
		half_theta = theta / 2.0
		sinc_half = np.sin(half_theta) / half_theta
		coeff = 1.0 - sinc_theta / (sinc_half ** 2)
		
		return I3 + (theta / 2.0) * skew_u + coeff * skew_u2
	
	# ========== 发布PBVS速度命令 ==========
	def _publish_pbvs_twist(self, v):
		"""将PBVS计算的相机期望六维速度发布为TwistStamped
		
		验证模式: 直接发布真实速度值(m/s, rad/s), 不归一化
		坐标系: 相机光学坐标系
		"""
		msg = TwistStamped()
		msg.header.stamp = self.get_clock().now().to_msg()
		msg.header.frame_id = self.camera_frame_id
		
		# 直接发布真实六维速度
		msg.twist.linear.x = float(v[0])
		msg.twist.linear.y = float(v[1])
		msg.twist.linear.z = float(v[2])
		msg.twist.angular.x = float(v[3])
		msg.twist.angular.y = float(v[4])
		msg.twist.angular.z = float(v[5])
		
		self.twist_pub.publish(msg)
	
	# ========== RGB图像回调 ==========
	def rgb_callback(self, msg):
		"""主回调: AprilTag检测+位姿估计+PBVS控制+显示"""
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
		
		# 5. PBVS 控制
		self._run_pbvs_control(tags, result_image)
		
		# 6. 右上角显示FPS
		self._display_fps(result_image)
		
		# 7. 显示图像
		try:
			display = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
			cv2.imshow("aritag_measure", display)
			cv2.waitKey(1)
		except Exception as e:
			if not self._display_error_logged:
				self.get_logger().warn(f"Display failed: {e}")
				self._display_error_logged = True

	# ========== PBVS 控制执行 ==========
	def _run_pbvs_control(self, tags, result_image):
		"""在每帧中执行PBVS控制(如果激活)
		
		查找目标tag, 计算控制律, 发布速度命令
		"""
		# 查找目标tag
		target_tag = None
		for tag in tags:
			if tag.tag_id == self.target_tag_id:
				target_tag = tag
				break
		
		# 绘制状态信息
		h, w = result_image.shape[:2]
		status_y = h - 60
		
		if not self.pbvs_active:
			cv2.putText(result_image, "[Detect Only] Press 's' to start PBVS",
					   (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
			return
		
		if target_tag is None:
			cv2.putText(result_image, f"[PBVS] Tag {self.target_tag_id} NOT found! Stop!",
					   (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
			# 目标丢失, 发送零速度停止
			self._publish_zero_twist()
			return
		
		# 提取目标tag的位姿 (需要重新用solvePnP计算)
		rvec, tvec = self._get_tag_pose(target_tag)
		if rvec is None:
			cv2.putText(result_image, "[PBVS] Pose estimation failed!",
					   (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
			return
		
		# 计算PBVS控制律
		result = self._compute_pbvs_velocity(rvec, tvec)
		if result is None:
			cv2.putText(result_image, "[PBVS] Singular matrix!",
					   (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
			return
		
		v, t_err, theta_u_err = result
		
		# 检查收敛
		t_err_norm = np.linalg.norm(t_err)
		r_err_norm = np.linalg.norm(theta_u_err)
		
		if t_err_norm < self.position_tolerance and r_err_norm < self.angle_tolerance:
			self.converged = True
			self._publish_zero_twist()
			cv2.putText(result_image, "[PBVS] CONVERGED!",
					   (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
			return
		
		self.converged = False
		
		# 发布速度命令
		self._publish_pbvs_twist(v)
		
		# 显示控制状态
		info = (f"[PBVS ACTIVE] t_err={t_err_norm*1000:.1f}mm "
				f"r_err={np.degrees(r_err_norm):.1f}deg "
				f"v=({v[0]*1000:.1f},{v[1]*1000:.1f},{v[2]*1000:.1f})mm/s")
		cv2.putText(result_image, info,
				   (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
	
	def _get_tag_pose(self, tag):
		"""对单个tag执行solvePnP获取位姿"""
		if self.camera_K is None:
			return None, None
		
		half = self.tag_size / 2.0
		obj_points = np.array([
			[-half, -half, 0.0],
			[ half, -half, 0.0],
			[ half,  half, 0.0],
			[-half,  half, 0.0],
		], dtype=np.float64)
		
		img_points = tag.corners.astype(np.float64)
		
		K_scaled = self.camera_K.copy()
		K_scaled[0, 0] *= self.scale_x
		K_scaled[0, 2] *= self.scale_x
		K_scaled[1, 1] *= self.scale_y
		K_scaled[1, 2] *= self.scale_y
		
		success, rvec, tvec = cv2.solvePnP(obj_points, img_points, K_scaled, None)
		if not success:
			return None, None
		return rvec, tvec
	
	def _publish_zero_twist(self):
		"""发布零速度命令(停止运动)"""
		msg = TwistStamped()
		msg.header.stamp = self.get_clock().now().to_msg()
		msg.header.frame_id = self.camera_frame_id
		self.twist_pub.publish(msg)

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
	node = AprilTagMeasureNode('aritag_pbvs_node')
	node.get_logger().info("AprilTag PBVS Node started")
	node.get_logger().info("在窗口中按 's' 启动PBVS控制, 按 'q' 退出")
	
	try:
		while rclpy.ok():
			rclpy.spin_once(node, timeout_sec=0.001)
			
			key = cv2.waitKey(1) & 0xFF
			if key == ord('s'):
				node.pbvs_active = not node.pbvs_active
				node.converged = False
				state = "ACTIVE" if node.pbvs_active else "STOPPED"
				node.get_logger().info(f"PBVS control: {state}")
				if not node.pbvs_active:
					node._publish_zero_twist()
			elif key == ord('q'):
				node.get_logger().info("Exiting...")
				break
	except KeyboardInterrupt:
		pass
	finally:
		node._publish_zero_twist()
		node.destroy_node()
		rclpy.shutdown()
