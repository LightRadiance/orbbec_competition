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

// #include "drone_msgs/msg/position_command.hpp"
// #include "drone_msgs/msg/exec_status.hpp"
#include "quadrotor_msgs/msg/exec_status.hpp"
#include "quadrotor_msgs/msg/position_command.hpp"
#include "drone_msgs/msg/joint_state.hpp"

namespace Drone {

// Yaw convert to [-2PI, 2PI]
double yawConvert(double yaw_in);

struct PosCmd {
  double x{0.0}, y{0.0}, z{0.0}, yaw{0.0};
  bool planner_ctrl{false}; // Wheteher controlled by planner
  bool picture{false}; // Whether to take picture 
};

// This class may be not complete
class SmoothTraj {
private:
  #define TRAJ_TIME (1.0)
  PosCmd start_pos, end_pos;
  double start_time;

public:
  // Set new trajectory
  void genNew(const PosCmd &start, const PosCmd &end, const rclcpp::Time& now) {
    start_pos = start;
    end_pos = end;
    start_time = now.seconds();
  }

  bool trajComplete(const rclcpp::Time& now) const { return now.seconds()>=start_time+TRAJ_TIME; }

  std::vector<double> getPosNow(const rclcpp::Time& now) const { 
    if (trajComplete(now)) return {end_pos.x, end_pos.y, end_pos.z, yawConvert(end_pos.yaw)};
    double scale = (now.seconds() - start_time) / TRAJ_TIME;
    double x, y, z, yaw;
    x = scale * (end_pos.x - start_pos.x) + start_pos.x;
    y = scale * (end_pos.y - start_pos.y) + start_pos.y;
    z = scale * (end_pos.z - start_pos.z) + start_pos.z;
    yaw = scale * (end_pos.yaw - start_pos.yaw) + start_pos.yaw;
    yaw = yawConvert(yaw);
    return {x, y, z, yaw};
  }
};

class DroneDrive : public rclcpp::Node {
public:
  DroneDrive();

private:
  // Subscriber
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr posititon_cmd_sub_;
  rclcpp::Subscription<quadrotor_msgs::msg::ExecStatus>::SharedPtr exec_status_sub_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;\
  rclcpp::Publisher<drone_msgs::msg::JointState>::SharedPtr joint_debug_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr nav_pub_;
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

  int VIDEO_START_{3};
  int VIDEO_STOP_{18};

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

  // Callback
  void positionCommandCb(const quadrotor_msgs::msg::PositionCommand::SharedPtr msg);
  void execStatusCb(const quadrotor_msgs::msg::ExecStatus::SharedPtr msg);
  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg);
  void cmdPubTimerCb();


  // Check if reaching target(x, y z)
  bool inPosition(PosCmd target);
  // Check if reaching target(yaw)
  bool inYaw(PosCmd target);

  void pubWaypoint(PosCmd target);
};

}
#endif