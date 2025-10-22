#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include "cv_bridge/cv_bridge.h"
#include "nav_msgs/msg/odometry.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/core/types.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/empty.hpp"

namespace fs = std::filesystem;

class TakePictureServer : public rclcpp::Node {
public:
  TakePictureServer()
      : Node("take_picture_server"), recording_(false),
        image_save_path_("/home/labor/Documents/research/orbbec_competition/image/"),
        video_save_path_("/home/labor/Documents/research/orbbec_competition/video/") {

    // 声明参数并读取
    this->declare_parameter("image_save_path", image_save_path_);
    this->declare_parameter("video_save_path", video_save_path_);
    this->get_parameter("image_save_path", image_save_path_);
    this->get_parameter("video_save_path", video_save_path_);

    // 路径检查
    if (!fs::exists(image_save_path_)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Path %s not exist. Please create directory first.",
                   image_save_path_.c_str());
      rclcpp::shutdown();
      return;
    }
    if (!fs::exists(video_save_path_)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Path %s not exist. Please create directory first.",
                   video_save_path_.c_str());
      rclcpp::shutdown();
      return;
    }

    // 订阅与服务
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera_rgb", 10,
        std::bind(&TakePictureServer::imageCb, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&TakePictureServer::odomCb, this, std::placeholders::_1));

    take_picture_srv_ = this->create_service<std_srvs::srv::Empty>(
        "/take_picture",
        std::bind(&TakePictureServer::takePictureCb, this,
                  std::placeholders::_1, std::placeholders::_2));
    start_record_srv_ = this->create_service<std_srvs::srv::Empty>(
        "/start_record",
        std::bind(&TakePictureServer::startRecord, this,
                  std::placeholders::_1, std::placeholders::_2));
    stop_record_srv_ = this->create_service<std_srvs::srv::Empty>(
        "/stop_record",
        std::bind(&TakePictureServer::stopRecord, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Take picture server started.");
    RCLCPP_INFO(this->get_logger(), "Images will save to: %s",
                image_save_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "Videos will save to: %s",
                video_save_path_.c_str());
  }

private:
  // ===== 服务回调 =====
  void takePictureCb(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                     std::shared_ptr<std_srvs::srv::Empty::Response>) {
    if (!camera_msg_) {
      RCLCPP_ERROR(this->get_logger(), "No camera message received.");
      return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(camera_msg_, camera_msg_->encoding);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat bgr_image;
    cv::cvtColor(cv_ptr->image, bgr_image, cv::COLOR_RGB2BGR);

    auto now_c =
        std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_c), "%Y%m%d%H%M%S");
    std::string filename = image_save_path_ + ss.str() + ".jpg";

    cv::imwrite(filename, bgr_image);
    RCLCPP_INFO(this->get_logger(), "Image saved to %s", filename.c_str());
  }

  void startRecord(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                   std::shared_ptr<std_srvs::srv::Empty::Response>) {
    if (recording_) {
      RCLCPP_WARN(this->get_logger(), "Video already recording.");
      return;
    }

    auto now_c =
        std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_c), "%Y%m%d%H%M%S");
    std::string filename = video_save_path_ + ss.str() + ".mp4";

    int fourcc = cv::VideoWriter::fourcc('a', 'v', 'c', '1'); // 比 H264 更兼容
    output_video_.open(filename, fourcc, 30.0, cv::Size(1280, 720), true);
    if (!output_video_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Unable to open video for write.");
      return;
    }

    recording_ = true;
    RCLCPP_INFO(this->get_logger(), "Start recording video: %s",
                filename.c_str());
  }

  void stopRecord(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                  std::shared_ptr<std_srvs::srv::Empty::Response>) {
    if (!recording_) {
      RCLCPP_WARN(this->get_logger(), "No recording in progress.");
      return;
    }

    output_video_.release();
    recording_ = false;
    RCLCPP_INFO(this->get_logger(), "Stop recording video.");
  }

  // ===== 订阅回调 =====
  void imageCb(const sensor_msgs::msg::Image::SharedPtr msg) {
    camera_msg_ = msg;

    if (recording_) {
      cv_bridge::CvImagePtr cv_ptr;
      try {
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
      } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }
      cv::Mat bgr_image;
      cv::cvtColor(cv_ptr->image, bgr_image, cv::COLOR_RGB2BGR);
      output_video_.write(bgr_image);
    }
  }

  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg) { odom_msg_ = msg; }

  // ===== 成员变量 =====
  bool recording_;
  std::string image_save_path_;
  std::string video_save_path_;
  cv::VideoWriter output_video_;

  sensor_msgs::msg::Image::SharedPtr camera_msg_;
  nav_msgs::msg::Odometry::SharedPtr odom_msg_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr take_picture_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_record_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_record_srv_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TakePictureServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
