
#pragma once

#include <vector>
#include <moveit/handeye_calibration_target/handeye_target_base.h>

// opencv
#include <opencv2/aruco.hpp>

namespace moveit_handeye_calibration
{
class HandEyeArucoTarget : public HandEyeTargetBase
{
public:
  HandEyeArucoTarget();
  ~HandEyeArucoTarget() = default;

  virtual bool initialize() override;

  virtual bool createTargetImage(cv::Mat& image) const override;

  virtual bool detectTargetPose(cv::Mat& image) override;

protected:
  virtual bool setTargetIntrinsicParams(int markers_x, int markers_y, int marker_size, int separation, int border_bits,
                                        const std::string& dictionary_id);

  virtual bool setTargetDimension(double marker_measured_size, double marker_measured_separation);

private:
  // Predefined ARUCO dictionaries in OpenCV for creating ARUCO marker board
  const std::map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME> ARUCO_DICTIONARY = {
    { "DICT_4X4_250", cv::aruco::DICT_4X4_250 },
    { "DICT_5X5_250", cv::aruco::DICT_5X5_250 },
    { "DICT_6X6_250", cv::aruco::DICT_6X6_250 },
    { "DICT_7X7_250", cv::aruco::DICT_7X7_250 },
    { "DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL }
  };

  // Target intrinsic params
  int markers_x_;                                        // Number of markers along X axis
  int markers_y_;                                        // Number of markers along Y axis
  int marker_size_;                                      // Marker size in pixels
  int separation_;                                       // Marker separation distance in pixels
  int border_bits_;                                      // Margin of boarder in bits
  cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_id_;  // Marker dictionary id

  // Target real dimensions in meters
  double marker_size_real_;        // Printed marker size
  double marker_separation_real_;  // Printed marker separation distance

  std::mutex aruco_mutex_;
};

}  // namespace moveit_handeye_calibration
