#ifndef __DRONE_DRIVE__
#define __DRONE_DRIVE__

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <vector>
#include <memory>

#include "drone_msgs/msg/position_command.hpp"
#include "drone_msgs/msg/exec_status.hpp"

namespace Drone {

struct PosCmd {
  double x{0.0}, y{0.0}, z{0.0}, yaw{0.0};
  bool picture{false};
  bool planner_ctrl{false};
};

class SmoothTraj {
public:
  void genNew(const PosCmd &, const PosCmd &) {}
  bool trajComplete() const { return true; }
  std::vector<double> getPosNow() const { return {0.0, 0.0, 0.0, 0.0}; }
};

// === 主类声明 ===
class DroneDrive : public rclcpp::Node {
public:
  DroneDrive();

  // Subscriber
  rclcpp::Subscription<drone_msgs::msg::PositionCommand>::SharedPtr posititon_cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<drone_msgs::msg::ExecStatus>::SharedPtr exec_status_sub_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr nav_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr camera_pose_pub_;

  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr set_yaw_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr take_picture_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr start_record_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr stop_record_client_;

  rclcpp::TimerBase::SharedPtr cmd_pub_timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  nav_msgs::msg::Odometry odom_;
  PosCmd planner_pos_cmd_;
  SmoothTraj smooth_pos_;
  std::vector<PosCmd> waypoint_list_;
  bool pos_cmd_update_{false};

  int VIDEO_START_{2};
  int VIDEO_STOP_{5};

  enum ExecState : uint8_t {
    INIT = 0,
    WAIT_TARGET,
    GEN_NEW_TRAJ,
    REPLAN_TRAJ,
    EXEC_TRAJ,
    EMERGENCY_STOP,
    NONE
  };
  uint8_t exec_status_{NONE};

  // ==== 回调函数 ====
  void positionCommandCb(const drone_msgs::msg::PositionCommand::SharedPtr msg);
  void execStatusCb(const drone_msgs::msg::ExecStatus::SharedPtr msg);
  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg);
  void cmdPubTimerCb();

  // ==== 功能函数 ====
  double yawConvert(double yaw_in);
  bool inPosition(PosCmd target);
  bool inYaw(PosCmd target);
  void pubWaypoint(PosCmd target);
};

}
#endif