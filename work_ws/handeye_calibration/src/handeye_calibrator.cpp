/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Hand-Eye Calibration Node
 *  A complete hand-eye calibration data collection and computation program
 *********************************************************************/

/* Author: kai01 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <std_msgs/msg/string.hpp>
#include <pluginlib/class_loader.hpp>
#include <moveit/handeye_calibration_target/handeye_target_base.h>
#include <moveit/handeye_calibration_solver/handeye_solver_base.h>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <memory>

namespace handeye_calibration
{

class HandEyeCalibrator : public rclcpp::Node
{
public:
  HandEyeCalibrator() : Node("handeye_calibrator"), clock_(std::make_shared<rclcpp::Clock>())
  {
    // 声明参数
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<std::string>("end_effector_frame", "tool0");
    this->declare_parameter<std::string>("camera_frame", "camera_color_optical_frame");
    this->declare_parameter<std::string>("target_type", "Aruco");  // Aruco 或 Charuco
    this->declare_parameter<std::string>("mount_type", "eye_in_hand");  // eye_in_hand 或 eye_to_hand
    this->declare_parameter<int>("min_samples", 10);
    this->declare_parameter<std::string>("solver_method", "Tsai1989");
    this->declare_parameter<std::string>("output_file", "/tmp/handeye_calibration_result.yaml");

    // 获取参数
    base_frame_ = this->get_parameter("base_frame").as_string();
    end_effector_frame_ = this->get_parameter("end_effector_frame").as_string();
    camera_frame_ = this->get_parameter("camera_frame").as_string();
    target_type_ = this->get_parameter("target_type").as_string();
    mount_type_str_ = this->get_parameter("mount_type").as_string();
    min_samples_ = this->get_parameter("min_samples").as_int();
    solver_method_ = this->get_parameter("solver_method").as_string();
    output_file_ = this->get_parameter("output_file").as_string();

    // 设置挂载类型
    if (mount_type_str_ == "eye_in_hand")
    {
      mount_type_ = moveit_handeye_calibration::EYE_IN_HAND;
      RCLCPP_INFO(this->get_logger(), "Mount type: Eye-in-Hand (camera on end-effector)");
    }
    else if (mount_type_str_ == "eye_to_hand")
    {
      mount_type_ = moveit_handeye_calibration::EYE_TO_HAND;
      RCLCPP_INFO(this->get_logger(), "Mount type: Eye-to-Hand (camera fixed externally)");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid mount type: %s", mount_type_str_.c_str());
      rclcpp::shutdown();
    }

    // TF2 Buffer 和 Listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 初始化目标板检测器
    if (!initializeTarget())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize calibration target");
      rclcpp::shutdown();
    }

    // 创建服务
    sample_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/handeye_calibration/sample",
      std::bind(&HandEyeCalibrator::sampleCallback, this, std::placeholders::_1, std::placeholders::_2));

    calibrate_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/handeye_calibration/calibrate",
      std::bind(&HandEyeCalibrator::calibrateCallback, this, std::placeholders::_1, std::placeholders::_2));

    reset_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/handeye_calibration/reset",
      std::bind(&HandEyeCalibrator::resetCallback, this, std::placeholders::_1, std::placeholders::_2));

    // 订阅相机图像和内参
    image_sub_ = image_transport::create_subscription(
      this, "/camera/color/image_raw",
      std::bind(&HandEyeCalibrator::imageCallback, this, std::placeholders::_1),
      "raw");

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/color/camera_info", 10,
      std::bind(&HandEyeCalibrator::cameraInfoCallback, this, std::placeholders::_1));

    // 发布检测结果图像话题(可在宿主机用 RViz/rqt_image_view 查看)
    image_pub_ = image_transport::create_publisher(this, "/handeye_calibration/detection_image");

    // 发布检测状态信息
    status_pub_ = this->create_publisher<std_msgs::msg::String>("/handeye_calibration/status", 10);

    // 创建 OpenCV 本地窗口(如果 Docker 支持 GUI)
    try
    {
      cv::namedWindow("Hand-Eye Calibration", cv::WINDOW_AUTOSIZE);
      enable_local_display_ = true;
      RCLCPP_INFO(this->get_logger(), "OpenCV window created successfully");
    }
    catch (...)
    {
      enable_local_display_ = false;
      RCLCPP_WARN(this->get_logger(), "Cannot create OpenCV window, using ROS2 topic only");
    }

    RCLCPP_INFO(this->get_logger(), "Detection image published to: /handeye_calibration/detection_image");
    RCLCPP_INFO(this->get_logger(), "View with: ros2 run rqt_image_view rqt_image_view or RViz");

    RCLCPP_INFO(this->get_logger(), "===========================================");
    RCLCPP_INFO(this->get_logger(), "Hand-Eye Calibration Node Started");
    RCLCPP_INFO(this->get_logger(), "===========================================");
    RCLCPP_INFO(this->get_logger(), "Commands:");
    RCLCPP_INFO(this->get_logger(), "  - Move robot to a pose");
    RCLCPP_INFO(this->get_logger(), "  - Call service: ros2 service call /handeye_calibration/sample std_srvs/srv/Trigger");
    RCLCPP_INFO(this->get_logger(), "  - Collect at least %d samples", min_samples_);
    RCLCPP_INFO(this->get_logger(), "  - Call service: ros2 service call /handeye_calibration/calibrate std_srvs/srv/Trigger");
    RCLCPP_INFO(this->get_logger(), "  - Reset: ros2 service call /handeye_calibration/reset std_srvs/srv/Trigger");
    RCLCPP_INFO(this->get_logger(), "===========================================");
  }

  ~HandEyeCalibrator()
  {
    if (enable_local_display_)
    {
      cv::destroyAllWindows();
    }
  }

private:
  // 发布检测结果图像
  void publishDetectionImage(const sensor_msgs::msg::Image::ConstSharedPtr& original_msg,
                             const cv::Mat& gray_image)
  {
    cv::Mat display_image;
    if (gray_image.channels() == 1)
    {
      cv::cvtColor(gray_image, display_image, cv::COLOR_GRAY2BGR);
    }
    else
    {
      cv::cvtColor(gray_image, display_image, cv::COLOR_RGB2BGR);
    }

    // 在图像上绘制状态信息
    std::string status_text = target_detected_ ? "TARGET DETECTED" : "TARGET NOT DETECTED";
    cv::Scalar text_color = target_detected_ ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
    cv::putText(display_image, status_text, cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, text_color, 2);

    std::string samples_text = "Samples: " + std::to_string(eef_poses_.size()) + "/" + std::to_string(min_samples_);
    cv::putText(display_image, samples_text, cv::Point(10, 60),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 0), 2);

    // 在本地窗口显示(如果支持)
    if (enable_local_display_)
    {
      cv::imshow("Hand-Eye Calibration", display_image);
      cv::waitKey(1);  // 1ms 刷新窗口
    }

    // 转换为 ROS 消息并发布
    cv_bridge::CvImage out_msg;
    out_msg.header = original_msg->header;
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg.image = display_image;
    image_pub_.publish(out_msg.toImageMsg());

    // 发布状态文本
    auto status_msg = std_msgs::msg::String();
    status_msg.data = status_text + " | " + samples_text;
    status_pub_->publish(status_msg);
  }

  // 初始化目标板检测器
  bool initializeTarget()
  {
    try
    {
      target_loader_ = std::make_unique<pluginlib::ClassLoader<moveit_handeye_calibration::HandEyeTargetBase>>(
        "moveit_calibration_plugins", "moveit_handeye_calibration::HandEyeTargetBase");

      std::string target_plugin_name;
      if (target_type_ == "Aruco")
      {
        target_plugin_name = "HandEyeTarget/Aruco";
        target_ = target_loader_->createUniqueInstance(target_plugin_name);

        // 配置 ArUco 参数
        target_->setParameter("markers, X", 4);
        target_->setParameter("markers, Y", 3);
        target_->setParameter("marker size (px)", 200);
        target_->setParameter("marker separation (px)", 20);
        target_->setParameter("marker border (bits)", 1);
        target_->setParameter("ArUco dictionary", "DICT_4X4_250");
        target_->setParameter("measured marker size (m)", 0.0256);
        target_->setParameter("measured separation (m)", 0.0066);
      }
      else if (target_type_ == "Charuco")
      {
        target_plugin_name = "HandEyeTarget/Charuco";
        target_ = target_loader_->createUniqueInstance(target_plugin_name);

        target_->setParameter("squares, X", 3);
        target_->setParameter("squares, Y", 4);
        target_->setParameter("marker size (px)", 45);
        target_->setParameter("square size (px)", 60);
        target_->setParameter("margin size (px)", 2);
        target_->setParameter("marker border (bits)", 1);
        target_->setParameter("ArUco dictionary", "DICT_5X5_250");
        target_->setParameter("longest board side (m)", 0.24); 
        target_->setParameter("measured marker size (m)", 0.045);  
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Unknown target type: %s", target_type_.c_str());
        return false;
      }

      if (!target_->initialize())
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize target");
        return false;
      }

      RCLCPP_INFO(this->get_logger(), "Calibration target initialized: %s", target_type_.c_str());
      return true;
    }
    catch (const pluginlib::PluginlibException& ex)
    {
      RCLCPP_ERROR(this->get_logger(), "Exception while creating target plugin: %s", ex.what());
      return false;
    }
  }

  // 相机图像回调
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    if (!camera_info_received_)
    {
      // 每 50 帧提醒一次
      static int frame_count = 0;
      if (++frame_count % 50 == 0)
      {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *clock_, 5000,
          "Waiting for camera info... Please ensure camera driver is publishing /camera/color/camera_info");
      }
      return;
    }

    try
    {
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
      cv::Mat gray_image = cv_ptr->image;

      // 检测目标板位姿
      if (target_->detectTargetPose(gray_image))
      {
        current_target_pose_ = target_->getTransformStamped(camera_frame_);
        target_detected_ = true;
      }
      else
      {
        target_detected_ = false;
      }

      // 始终发布检测结果图像(通过 ROS2 话题,可在宿主机查看)
      publishDetectionImage(msg, gray_image);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  // 相机内参回调
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
  {
    if (!camera_info_received_)
    {
      if (target_->setCameraIntrinsicParams(msg))
      {
        camera_info_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Camera intrinsic parameters set successfully");
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to set camera intrinsic parameters");
      }
    }
  }

  // 采样服务回调
  void sampleCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;

    if (!camera_info_received_)
    {
      response->success = false;
      response->message = "Camera info not received yet";
      RCLCPP_WARN(this->get_logger(), "Camera info not received yet");
      return;
    }

    if (!target_detected_)
    {
      response->success = false;
      response->message = "Target not detected in current image";
      RCLCPP_WARN(this->get_logger(), "Target not detected in current image");
      return;
    }

    try
    {
      // 获取机械臂末端位姿
      geometry_msgs::msg::TransformStamped eef_transform =
        tf_buffer_->lookupTransform(base_frame_, end_effector_frame_, tf2::TimePointZero);

      Eigen::Isometry3d eef_pose = tf2::transformToEigen(eef_transform);
      Eigen::Isometry3d target_pose = tf2::transformToEigen(current_target_pose_);

      // 保存位姿数据对
      eef_poses_.push_back(eef_pose);
      target_poses_.push_back(target_pose);

      int num_samples = eef_poses_.size();

      RCLCPP_INFO(this->get_logger(), "===========================================");
      RCLCPP_INFO(this->get_logger(), "Sample %d collected", num_samples);
      RCLCPP_INFO(this->get_logger(), "End-effector position: (%.4f, %.4f, %.4f)",
        eef_pose.translation().x(), eef_pose.translation().y(), eef_pose.translation().z());
      RCLCPP_INFO(this->get_logger(), "Target position: (%.4f, %.4f, %.4f)",
        target_pose.translation().x(), target_pose.translation().y(), target_pose.translation().z());
      RCLCPP_INFO(this->get_logger(), "Progress: %d/%d samples", num_samples, min_samples_);
      RCLCPP_INFO(this->get_logger(), "===========================================");

      response->success = true;
      response->message = "Sample " + std::to_string(num_samples) + " collected";
    }
    catch (tf2::TransformException& ex)
    {
      response->success = false;
      response->message = "TF lookup failed: " + std::string(ex.what());
      RCLCPP_ERROR(this->get_logger(), "TF lookup failed: %s", ex.what());
    }
  }

  // 标定服务回调
  void calibrateCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;

    if (eef_poses_.size() < static_cast<size_t>(min_samples_))
    {
      response->success = false;
      response->message = "Not enough samples. Need at least " + std::to_string(min_samples_);
      RCLCPP_WARN(this->get_logger(), "Not enough samples. Current: %zu, Required: %d",
        eef_poses_.size(), min_samples_);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Starting hand-eye calibration with %zu samples...", eef_poses_.size());

    try
    {
      // 初始化求解器
      solver_loader_ = std::make_unique<pluginlib::ClassLoader<moveit_handeye_calibration::HandEyeSolverBase>>(
        "moveit_calibration_plugins", "moveit_handeye_calibration::HandEyeSolverBase");

      solver_ = solver_loader_->createUniqueInstance("OpenCV");
      solver_->initialize();

      // 执行标定
      std::string error_message;
      bool success = solver_->solve(eef_poses_, target_poses_, mount_type_, solver_method_, &error_message);

      if (success)
      {
        // 获取结果
        Eigen::Isometry3d result = solver_->getCameraRobotPose();

        RCLCPP_INFO(this->get_logger(), "===========================================");
        RCLCPP_INFO(this->get_logger(), "Calibration Result:");
        RCLCPP_INFO(this->get_logger(), "===========================================");
        RCLCPP_INFO(this->get_logger(), "Translation:");
        RCLCPP_INFO(this->get_logger(), "  X: %.6f", result.translation().x());
        RCLCPP_INFO(this->get_logger(), "  Y: %.6f", result.translation().y());
        RCLCPP_INFO(this->get_logger(), "  Z: %.6f", result.translation().z());

        Eigen::Vector3d euler_angles = result.rotation().eulerAngles(0, 1, 2);
        RCLCPP_INFO(this->get_logger(), "Rotation (Euler angles - XYZ):");
        RCLCPP_INFO(this->get_logger(), "  Roll:  %.6f rad (%.2f deg)", euler_angles.x(), euler_angles.x() * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "  Pitch: %.6f rad (%.2f deg)", euler_angles.y(), euler_angles.y() * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "  Yaw:   %.6f rad (%.2f deg)", euler_angles.z(), euler_angles.z() * 180.0 / M_PI);

        RCLCPP_INFO(this->get_logger(), "Transformation Matrix:");
        RCLCPP_INFO(this->get_logger(), "\n%.6f %.6f %.6f %.6f",
          result.matrix()(0, 0), result.matrix()(0, 1), result.matrix()(0, 2), result.matrix()(0, 3));
        RCLCPP_INFO(this->get_logger(), "%.6f %.6f %.6f %.6f",
          result.matrix()(1, 0), result.matrix()(1, 1), result.matrix()(1, 2), result.matrix()(1, 3));
        RCLCPP_INFO(this->get_logger(), "%.6f %.6f %.6f %.6f",
          result.matrix()(2, 0), result.matrix()(2, 1), result.matrix()(2, 2), result.matrix()(2, 3));
        RCLCPP_INFO(this->get_logger(), "%.6f %.6f %.6f %.6f",
          result.matrix()(3, 0), result.matrix()(3, 1), result.matrix()(3, 2), result.matrix()(3, 3));
        RCLCPP_INFO(this->get_logger(), "===========================================");

        // 保存结果
        if (saveResult(result))
        {
          response->success = true;
          response->message = "Calibration successful! Result saved to " + output_file_;
        }
        else
        {
          response->success = true;
          response->message = "Calibration successful but failed to save result";
        }
      }
      else
      {
        response->success = false;
        response->message = "Calibration failed: " + error_message;
        RCLCPP_ERROR(this->get_logger(), "Calibration failed: %s", error_message.c_str());
      }
    }
    catch (const pluginlib::PluginlibException& ex)
    {
      response->success = false;
      response->message = "Exception while creating solver plugin: " + std::string(ex.what());
      RCLCPP_ERROR(this->get_logger(), "Exception: %s", ex.what());
    }
  }

  // 重置服务回调
  void resetCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;

    eef_poses_.clear();
    target_poses_.clear();

    RCLCPP_INFO(this->get_logger(), "All samples cleared");
    response->success = true;
    response->message = "All samples cleared";
  }

  // 保存标定结果
  bool saveResult(const Eigen::Isometry3d& result)
  {
    try
    {
      YAML::Node config;

      config["calibration_result"] = YAML::Node();
      config["calibration_result"]["mount_type"] = mount_type_str_;
      config["calibration_result"]["solver_method"] = solver_method_;
      config["calibration_result"]["num_samples"] = eef_poses_.size();
      config["calibration_result"]["base_frame"] = base_frame_;
      config["calibration_result"]["end_effector_frame"] = end_effector_frame_;
      config["calibration_result"]["camera_frame"] = camera_frame_;

      // 平移
      config["calibration_result"]["translation"]["x"] = result.translation().x();
      config["calibration_result"]["translation"]["y"] = result.translation().y();
      config["calibration_result"]["translation"]["z"] = result.translation().z();

      // 旋转矩阵
      std::vector<double> rotation_matrix;
      for (int i = 0; i < 3; ++i)
      {
        for (int j = 0; j < 3; ++j)
        {
          rotation_matrix.push_back(result.matrix()(i, j));
        }
      }
      config["calibration_result"]["rotation_matrix"] = rotation_matrix;

      // 欧拉角
      Eigen::Vector3d euler_angles = result.rotation().eulerAngles(0, 1, 2);
      config["calibration_result"]["euler_angles"]["roll"] = euler_angles.x();
      config["calibration_result"]["euler_angles"]["pitch"] = euler_angles.y();
      config["calibration_result"]["euler_angles"]["yaw"] = euler_angles.z();

      // 四元数
      Eigen::Quaterniond quat(result.rotation());
      config["calibration_result"]["quaternion"]["w"] = quat.w();
      config["calibration_result"]["quaternion"]["x"] = quat.x();
      config["calibration_result"]["quaternion"]["y"] = quat.y();
      config["calibration_result"]["quaternion"]["z"] = quat.z();

      // 写入文件
      std::ofstream fout(output_file_);
      fout << config;
      fout.close();

      RCLCPP_INFO(this->get_logger(), "Result saved to: %s", output_file_.c_str());
      return true;
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to save result: %s", e.what());
      return false;
    }
  }

  // 成员变量
  std::shared_ptr<rclcpp::Clock> clock_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // 插件加载器
  std::unique_ptr<pluginlib::ClassLoader<moveit_handeye_calibration::HandEyeTargetBase>> target_loader_;
  pluginlib::UniquePtr<moveit_handeye_calibration::HandEyeTargetBase> target_;

  std::unique_ptr<pluginlib::ClassLoader<moveit_handeye_calibration::HandEyeSolverBase>> solver_loader_;
  pluginlib::UniquePtr<moveit_handeye_calibration::HandEyeSolverBase> solver_;

  // 订阅者
  image_transport::Subscriber image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  // 服务
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr sample_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;

  // 参数
  std::string base_frame_;
  std::string end_effector_frame_;
  std::string camera_frame_;
  std::string target_type_;
  std::string mount_type_str_;
  std::string solver_method_;
  std::string output_file_;
  int min_samples_;

  // 标定模式
  moveit_handeye_calibration::SensorMountType mount_type_;

  // 数据
  std::vector<Eigen::Isometry3d> eef_poses_;
  std::vector<Eigen::Isometry3d> target_poses_;

  // 发布者
  image_transport::Publisher image_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  // 状态
  bool camera_info_received_ = false;
  bool target_detected_ = false;
  bool enable_local_display_ = false;  // 是否支持本地窗口显示
  geometry_msgs::msg::TransformStamped current_target_pose_;
};

}  // namespace handeye_calibration

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<handeye_calibration::HandEyeCalibrator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
