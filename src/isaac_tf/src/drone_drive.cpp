#include "drone_drive.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/empty.hpp"

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <cmath>
#include <vector>

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace Drone;

namespace Drone {

DroneDrive::DroneDrive() : Node("drone_drive") {
  posititon_cmd_sub_ = this->create_subscription<drone_msgs::msg::PositionCommand>(
      "planning/pos_cmd", 5,
      std::bind(&DroneDrive::positionCommandCb, this, _1));

  joint_pub_ =
      this->create_publisher<sensor_msgs::msg::JointState>("/drone_cmd", 10);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 1, std::bind(&DroneDrive::odomCb, this, _1));

  camera_pose_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("/camera_pose", 5);

  exec_status_sub_ = this->create_subscription<drone_msgs::msg::ExecStatus>(
      "/planning/exec_status", 1,
      std::bind(&DroneDrive::execStatusCb, this, _1));

  nav_pub_ =
      this->create_publisher<nav_msgs::msg::Path>("/waypoint_generator/waypoints", 1);

  set_yaw_client_ = this->create_client<std_srvs::srv::Empty>("/set_yaw");
  take_picture_client_ = this->create_client<std_srvs::srv::Empty>("/take_picture");
  start_record_client_ = this->create_client<std_srvs::srv::Empty>("/start_record");
  stop_record_client_ = this->create_client<std_srvs::srv::Empty>("/stop_record");

  cmd_pub_timer_ = this->create_wall_timer(
      10ms, std::bind(&DroneDrive::cmdPubTimerCb, this));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(this->get_logger(), "DroneDrive node started.");
}

// yaw convert to [-2PI, 2PI]
double DroneDrive::yawConvert(double yaw_in) {
  static int loop_cnt = 0;
  static double last_yaw = yaw_in;
  double out_yaw = 0;
  if (fabs(yaw_in - last_yaw) > 1.5 * M_PI) {
    if (yaw_in < 0)
      ++loop_cnt;
    else
      --loop_cnt;
  }
  last_yaw = yaw_in;
  if (loop_cnt > 0)
    out_yaw = yaw_in + 2 * M_PI;
  else if (loop_cnt < 0)
    out_yaw = yaw_in - 2 * M_PI;
  else
    out_yaw = yaw_in;

  if (out_yaw > 2 * M_PI) {
    --loop_cnt;
    out_yaw = yaw_in;
  } else if (out_yaw < -2 * M_PI) {
    ++loop_cnt;
    out_yaw = yaw_in;
  }
  return out_yaw;
}

void DroneDrive::positionCommandCb(
    const drone_msgs::msg::PositionCommand::SharedPtr msg) {
  planner_pos_cmd_.x = msg->position.x;
  planner_pos_cmd_.y = msg->position.y;
  planner_pos_cmd_.z = msg->position.z;
  planner_pos_cmd_.yaw = msg->yaw;
  pos_cmd_update_ = true;
}

void DroneDrive::execStatusCb(const drone_msgs::msg::ExecStatus::SharedPtr msg) {
  exec_status_ = msg->exec_flag;
}

bool DroneDrive::inPosition(PosCmd p2) {
  const double DIS_ABS = 0.2;
  geometry_msgs::msg::Pose p1 = odom_.pose.pose;
  double dis = sqrt(pow(p1.position.x - p2.x, 2) +
                    pow(p1.position.y - p2.y, 2) +
                    pow(p1.position.z - p2.z, 2));
  return dis < DIS_ABS;
}

bool DroneDrive::inYaw(PosCmd p2) {
  const double YAW_ABS = 5.0 / 180.0 * M_PI;
  geometry_msgs::msg::Pose p1 = odom_.pose.pose;

  tf2::Quaternion q(p1.orientation.x, p1.orientation.y,
                    p1.orientation.z, p1.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return fabs(yaw - p2.yaw) < YAW_ABS;
}

void DroneDrive::pubWaypoint(PosCmd p) {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "world";
  pose.header.stamp = this->now();
  pose.pose.position.x = p.x;
  pose.pose.position.y = p.y;
  pose.pose.position.z = p.z;

  tf2::Quaternion q;
  q.setRPY(0, 0, p.yaw);
  q.normalize();
  pose.pose.orientation = tf2::toMsg(q);

  nav_msgs::msg::Path path;
  path.poses.push_back(pose);
  path.header.frame_id = "world";
  path.header.stamp = this->now();
  nav_pub_->publish(path);
}

void DroneDrive::odomCb(const nav_msgs::msg::Odometry::SharedPtr msg) {
  odom_ = *msg;
  geometry_msgs::msg::TransformStamped tf_stamped;
  try {
    tf_stamped = tf_buffer_->lookupTransform("odom", "camera", tf2::TimePointZero);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    return;
  }

  geometry_msgs::msg::PoseStamped camera_pose;
  camera_pose.header.frame_id = tf_stamped.child_frame_id;
  camera_pose.header.stamp = this->now();
  camera_pose.pose.position.x = tf_stamped.transform.translation.x;
  camera_pose.pose.position.y = tf_stamped.transform.translation.y;
  camera_pose.pose.position.z = tf_stamped.transform.translation.z;
  camera_pose.pose.orientation = tf_stamped.transform.rotation;
  camera_pose_pub_->publish(camera_pose);
}

void DroneDrive::cmdPubTimerCb() {
  static int waypoint_now = 0;
  static bool finish = false;

  if (exec_status_ == NONE || finish)
    return;

  if (inPosition(waypoint_list_[waypoint_now])) {
    bool next = false;
    if (waypoint_list_[waypoint_now].planner_ctrl == false) {
      if (smooth_pos_.trajComplete())
        next = true;
    } else if (exec_status_ == WAIT_TARGET)
      next = true;

    if (next) {
      if (waypoint_list_[waypoint_now].picture) {
        auto srv = std::make_shared<std_srvs::srv::Empty::Request>();
        if (!take_picture_client_->service_is_ready() ||
            !take_picture_client_->async_send_request(srv)) {
          RCLCPP_WARN(this->get_logger(), "Take picture failed.");
        }
      }

      ++waypoint_now;
      if (waypoint_now >= waypoint_list_.size()) {
        RCLCPP_INFO(this->get_logger(), "All waypoint executed. Finish.");
        finish = true;
        return;
      }

      if (waypoint_now == VIDEO_START_) {
        auto req = std::make_shared<std_srvs::srv::Empty::Request>();
        start_record_client_->async_send_request(req);
      } else if (waypoint_now == VIDEO_STOP_ + 1) {
        auto req = std::make_shared<std_srvs::srv::Empty::Request>();
        stop_record_client_->async_send_request(req);
      }

      RCLCPP_INFO(this->get_logger(),
                  "New waypoint: x=%.1f y=%.1f z=%.1f yaw=%.1f",
                  waypoint_list_[waypoint_now].x,
                  waypoint_list_[waypoint_now].y,
                  waypoint_list_[waypoint_now].z,
                  waypoint_list_[waypoint_now].yaw);

      if (waypoint_list_[waypoint_now].planner_ctrl) {
        pos_cmd_update_ = false;
        pubWaypoint(waypoint_list_[waypoint_now]);
        return;
      } else {
        smooth_pos_.genNew(waypoint_list_[waypoint_now - 1],
                           waypoint_list_[waypoint_now]);
      }
    }
  }

  if (waypoint_list_[waypoint_now].planner_ctrl && !pos_cmd_update_)
    return;

  sensor_msgs::msg::JointState joint_cmd;
  joint_cmd.name = {"x_joint", "y_joint", "z_joint", "R_body"};

  if (waypoint_list_[waypoint_now].planner_ctrl) {
    joint_cmd.position = {planner_pos_cmd_.x, planner_pos_cmd_.y,
                          planner_pos_cmd_.z, yawConvert(planner_pos_cmd_.yaw)};
  } else {
    joint_cmd.position = smooth_pos_.getPosNow();
  }

  joint_cmd.header.stamp = this->now();
  joint_pub_->publish(joint_cmd);
}

} // namespace Drone

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Drone::DroneDrive>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
