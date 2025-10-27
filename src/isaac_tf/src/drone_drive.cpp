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
#include <Eigen/Eigen>

using namespace std::chrono_literals;
using std::placeholders::_1;
// using namespace Drone;

namespace Drone {
double yawConvert(double yaw_in) {
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

DroneDrive::DroneDrive() : Node("drone_drive") {
  // Subscriber
  // "planning/pos_cmd" will add node namespace automatically
  posititon_cmd_sub_ = this->create_subscription<quadrotor_msgs::msg::PositionCommand>(
      "/drone_0_planning/pos_cmd", 5,
      std::bind(&DroneDrive::positionCommandCb, this, _1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/drone_0_visual_slam/odom", 1, std::bind(&DroneDrive::odomCb, this, _1));
  exec_status_sub_ = this->create_subscription<quadrotor_msgs::msg::ExecStatus>(
      "/planning/exec_status", 1,
      std::bind(&DroneDrive::execStatusCb, this, _1));

  //  Publisher
  joint_pub_ =
      this->create_publisher<sensor_msgs::msg::JointState>("/drone_0_cmd", 10);
  joint_debug_pub_ =
      this->create_publisher<drone_msgs::msg::JointState>("/drone_0_cmd_debug", 10);
  camera_pose_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("/drone_0_pcl_render_node/camera_pose", 5);
  nav_pub_ = 
      this->create_publisher<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 1);
  // Client
  set_yaw_client_ = this->create_client<std_srvs::srv::Empty>("/set_yaw");
  take_picture_client_ = this->create_client<std_srvs::srv::Empty>("/take_picture");
  start_record_client_ = this->create_client<std_srvs::srv::Empty>("/start_record");
  stop_record_client_ = this->create_client<std_srvs::srv::Empty>("/stop_record");

  cmd_pub_timer_ = this->create_wall_timer(
      10ms, std::bind(&DroneDrive::cmdPubTimerCb, this));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  constexpr double CENTER_X = 2.0;
  constexpr double CENTER_Y = 0.0;
  constexpr double RADIUS = 1.25;
  constexpr double SQRT_R = 1.118;
  // waypoint_list_= {
  //   {0, 0, 0, 0, false},
  //   {0, 0, 0.6, 0, false},
  //   // {7.18814, -4.13339, 2.9, 0, true},
  //   {7.18814, -4.13339, 2.9, 0, true},
  //   {7.18814, -4.13339, 2.9, 0, false, true},
  //   // {7.18814, 0.48393, 2.9, 0, true},
  //   {7.18814, 0.48393, 2.9, 0, true},
  //   {7.18814, 0.48393, 2.9, 0, false, true},
  //   // {7.18814, 5.48069, 2.9, 0, true},
  //   {7.18814, 5.48069, 2.9, 0, true},
  //   {7.18814, 5.48069, 2.9, 0, false, true},
  //   // {28, 3.50703, 2.9, -M_PI / 2, true},
  //   {28, 3.50703, 2.9, -M_PI / 2, true},
  //   // {28, -1.97831, 0.94074, -M_PI, true},
  //   {28, -1.97831, 0.94074, -M_PI, true},
  //   // {2, -2, 0.7, -M_PI, true},
  //   {2, -2, 0.7, -M_PI, true},
  //   // around franka
  //   {CENTER_Y+RADIUS, -CENTER_X, 0.7, -M_PI, false},
  //   {CENTER_Y+SQRT_R, -(CENTER_X-SQRT_R), 0.7, -M_PI*3/4, false},
  //   {CENTER_Y, -(CENTER_X-RADIUS), 0.7, -M_PI/2, false},
  //   {CENTER_Y-SQRT_R, -(CENTER_X-SQRT_R), 0.7, -M_PI/4, false},
  //   {CENTER_Y-RADIUS, -(CENTER_X), 0.7, 0, false},
  //   {CENTER_Y-SQRT_R, -(CENTER_X+SQRT_R), 0.7, M_PI/4, false},
  //   {CENTER_Y, -(CENTER_X+RADIUS), 0.7, M_PI/2, false},
  //   {CENTER_Y+SQRT_R, -(CENTER_X+SQRT_R), 0.7, M_PI*3/4, false},
  //   // around franks 2
  //   {CENTER_Y+RADIUS, -CENTER_X, 0.4, M_PI, false},
  //   {CENTER_Y+SQRT_R, -(CENTER_X+SQRT_R), 0.4, M_PI*3/4, false},
  //   {CENTER_Y, -(CENTER_X+RADIUS), 0.4, M_PI/2, false},
  //   {CENTER_Y-SQRT_R, -(CENTER_X+SQRT_R), 0.4, M_PI/4, false},
  //   {CENTER_Y-RADIUS, -(CENTER_X), 0.4, 0, false},
  //   {CENTER_Y-SQRT_R, -(CENTER_X-SQRT_R), 0.4, -M_PI/4, false},
  //   {CENTER_Y, -(CENTER_X-RADIUS), 0.4, -M_PI/2, false},
  //   {CENTER_Y+SQRT_R, -(CENTER_X-SQRT_R), 0.4, -M_PI*3/4, false},
  //   {CENTER_Y+RADIUS, -CENTER_X, 0.4, -M_PI, false},

  //   {0, 0, 0.4, 0, true},
  //   {0, 0, 0, 0, false},
  // };

  // Test
  waypoint_list_= {
    {0, 0, 0, 0, false},
    {0, 0, 0.6, 0, false},
    // {7.18814, -4.13339, 2.9, 0, true},
    {7.18814, -4.13339, 2.0, 0, true},
    {0, 0, 0.4, 0, true},
    {0, 0, 0, 0, false},
  };
  // // This is becuse the axis is not different
  // Eigen::Matrix3d R;
  // R << 0, -1, 0,
  //      1,  0, 0,
  //      0,  0, 1;
  // for (auto &wp : waypoint_list_) {
  //   Eigen::Vector3d pos(wp.x, wp.y, wp.z);
  //   Eigen::Vector3d pos_rot = R * pos;
  //   wp.x = pos_rot(0);
  //   wp.y = pos_rot(1);
  //   // wp.z = pos_rot(2);
  //   wp.yaw += M_PI / 2;
  // }
  // waypoint_list_[0].yaw = 0;

  RCLCPP_INFO(this->get_logger(), "DroneDrive node started.");
}


void DroneDrive::positionCommandCb(
    const quadrotor_msgs::msg::PositionCommand::SharedPtr msg) {
  // Just store the command
  planner_pos_cmd_.x = msg->position.x;
  planner_pos_cmd_.y = msg->position.y;
  planner_pos_cmd_.z = msg->position.z;
  planner_pos_cmd_.yaw = msg->yaw;
  pos_cmd_update_ = true;
}

void DroneDrive::execStatusCb(const quadrotor_msgs::msg::ExecStatus::SharedPtr msg) {
  exec_status_ = msg->exec_flag;
}

bool DroneDrive::inPosition(PosCmd p2) {
  constexpr double DIS_ABS = 0.2;
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

  nav_pub_->publish(pose);
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

  if (exec_status_ == NONE || finish) return;

  if (inPosition(waypoint_list_[waypoint_now])) {
    bool next = false;
    // Check if needing switch to next waypoint
    if (!waypoint_list_[waypoint_now].planner_ctrl) {
      if (smooth_pos_.trajComplete(this->now())) next = true;
    } else if (exec_status_ == WAIT_TARGET) next = true;

    if (next) {
      // Check if needing take picture at this waypoint
      if (waypoint_list_[waypoint_now].picture) {
        if (!take_picture_client_->wait_for_service(2s)) {
          RCLCPP_WARN(this->get_logger(), "Service /take_picture not available!");
          return;
        }
        auto srv = std::make_shared<std_srvs::srv::Empty::Request>();
        auto res = take_picture_client_->async_send_request(srv);
        RCLCPP_INFO(this->get_logger(), "Take picture!");
      }

      // Switch to next
      ++waypoint_now;

      // Check if all waypoints are visited
      if (waypoint_now >= waypoint_list_.size()) {
        RCLCPP_INFO(this->get_logger(), "All waypoint executed. Finish.");
        finish = true;
        return;
      }

      // Check if needing start record or stop record
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
        if (waypoint_list_[waypoint_now - 1].planner_ctrl == false) {
          if (!set_yaw_client_->wait_for_service(2s)) {
            RCLCPP_WARN(this->get_logger(), "Service /set_yaw not available!");
            return;
          }
          auto srv = std::make_shared<std_srvs::srv::Empty::Request>();
          auto res = set_yaw_client_->async_send_request(srv);
          RCLCPP_INFO(this->get_logger(), "Reset yaw!");
        }
        pubWaypoint(waypoint_list_[waypoint_now]);
        return;
      } else {
        smooth_pos_.genNew(waypoint_list_[waypoint_now - 1],
                           waypoint_list_[waypoint_now], this->now());
      }
    }
  } 

  if (waypoint_list_[waypoint_now].planner_ctrl) {
    if (!pos_cmd_update_) return;
    else pos_cmd_update_=false;
  }

  // Send target pose to drone in sim
  sensor_msgs::msg::JointState joint_cmd;
  joint_cmd.name = {"x_joint", "y_joint", "z_joint", "R_body"};

  if (waypoint_list_[waypoint_now].planner_ctrl) {
    joint_cmd.position = {planner_pos_cmd_.x, planner_pos_cmd_.y,
                          planner_pos_cmd_.z, yawConvert(planner_pos_cmd_.yaw)};
  } else {
    joint_cmd.position = smooth_pos_.getPosNow(this->now());
  }

  joint_cmd.header.stamp = this->now();
  joint_pub_->publish(joint_cmd);

  // Debug for curve visualization
  drone_msgs::msg::JointState joint_debug;
  joint_debug.x = joint_cmd.position[0];
  joint_debug.y = joint_cmd.position[1];
  joint_debug.z = joint_cmd.position[2];
  joint_debug.yaw = joint_cmd.position[3];
  joint_debug_pub_->publish(joint_debug);
}

} // namespace Drone

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Drone::DroneDrive>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
