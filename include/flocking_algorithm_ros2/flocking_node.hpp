#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "mss_msgs/msg/navigation.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <Eigen/Dense>
#include <mutex>
#include <unordered_map>
#include <vector>
#include <string>

namespace flocking
{

struct AgentState
{
  Eigen::Vector2d pos{0.0, 0.0};
  Eigen::Vector2d vel{0.0, 0.0};   // world-frame velocity (x,y)
  double          yaw{0.0};        // psi
  rclcpp::Time    stamp{};
};

class FlockingNode : public rclcpp::Node
{
public:
  FlockingNode();

private:
  // utils
  static std::string make_topic_from_pattern(const std::string &pattern, size_t idx);
  static double wrapAngle(double a);
  static double sat(double x, double limit);
  static Eigen::Vector2d sat2D(const Eigen::Vector2d &v, double limit);

  // callbacks
  void ownOdomCallbackIdx(size_t idx, const mss_msgs::msg::Navigation::SharedPtr msg);
  void flockOdomCallback(const mss_msgs::msg::Navigation::SharedPtr msg);
  void leaderOdomCallback(const mss_msgs::msg::Navigation::SharedPtr msg);
  void controlTimerCB();

  // forces
  Eigen::Vector2d sepForce(const AgentState &self);
  Eigen::Vector2d alignForce(const AgentState &self);
  Eigen::Vector2d cohForce(const AgentState &self);
  Eigen::Vector2d boundaryForce(const AgentState &self);
  Eigen::Vector2d leaderForce(const AgentState &self, bool &leader_ok);

  void pruneStale(double max_age_sec);

  // params / states
  size_t agent_count_{1};
  std::string agent_id_prefix_{"agent_"};
  std::string own_odom_pattern_{"/agent_%d/odom"};
  std::string cmd_vel_pattern_{"/agent_%d/cmd_vel"};

  double loop_rate_hz_{20.0};

  // radii & gains
  double r_neighbor_{8.0};
  double r_separation_{2.0};
  double k_sep_{5.5};
  double k_align_{0.6};
  double k_coh_{0.4};
  double k_bound_{1.0};

  double max_speed_{100.0};
  double max_force_{100.0};
  double k_yaw_{1.5};
  double stale_sec_{1.0};

  bool use_boundary_{true};
  double xmin_{-50.0}, xmax_{50.0}, ymin_{-50.0}, ymax_{50.0};

  // leader
  std::string leader_odom_topic_{"/agent_leader/odom"};
  std::string leader_id_{"leader"};
  double k_leader_pos_{1.2};
  double k_leader_align_{0.5};

  const double pi = 3.14159265358979;

  // cmd semantics
  // "psi_angle": cmd.angular.z = psi_des, "yaw_rate": cmd.angular.z = k_yaw*(psi_des - psi)
  std::string twist_semantics_{"psi_angle"};

  // IO
  std::vector<std::string> agent_ids_;
  std::vector<rclcpp::Subscription<mss_msgs::msg::Navigation>::SharedPtr> own_odom_subs_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> cmd_pubs_;
  rclcpp::Subscription<mss_msgs::msg::Navigation>::SharedPtr flock_sub_;
  rclcpp::Publisher<mss_msgs::msg::Navigation>::SharedPtr    flock_pub_;

  rclcpp::Subscription<mss_msgs::msg::Navigation>::SharedPtr leader_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  // shared state
  std::mutex mtx_;
  std::unordered_map<std::string, AgentState> states_; // includes leader under leader_id_

};

} // namespace flocking
