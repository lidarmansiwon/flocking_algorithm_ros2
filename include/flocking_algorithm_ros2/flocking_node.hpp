#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <unordered_map>
#include <string>
#include <mutex>
#include <optional>
#include <cmath>
#include <vector>
#include <memory>

#include <Eigen/Dense>

namespace flocking
{

struct AgentState
{
  Eigen::Vector2d pos{0.0, 0.0};
  Eigen::Vector2d vel{0.0, 0.0};
  double yaw{0.0};
  rclcpp::Time stamp{};
};

class FlockingNode : public rclcpp::Node
{
public:
  FlockingNode();

private:
  // Callbacks
  void ownOdomCallbackIdx(size_t idx, const nav_msgs::msg::Odometry::SharedPtr msg);
  void flockOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void controlTimerCB();

  // Boids terms (계산은 "self" 기준으로, 이웃은 states_에서 참조)
  Eigen::Vector2d sepForce(const AgentState &self);
  Eigen::Vector2d alignForce(const AgentState &self);
  Eigen::Vector2d cohForce(const AgentState &self);
  Eigen::Vector2d boundaryForce(const AgentState &self);

  // Helpers
  static double wrapAngle(double a);
  static double sat(double x, double limit);
  static Eigen::Vector2d sat2D(const Eigen::Vector2d &v, double limit);
  void pruneStale(double max_age_sec);

  // Topic name helper
  static std::string make_topic_from_pattern(const std::string &pattern, size_t idx);

private:
  // --- 멀티 에이전트 설정 ---
  size_t agent_count_{1};
  std::string agent_id_prefix_;           // 기본: "agent_"
  std::vector<std::string> agent_ids_;    // ["agent_0", "agent_1", ...]
  std::string own_odom_pattern_;          // 기본: "/agent_%d/odom"
  std::string cmd_vel_pattern_;           // 기본: "/agent_%d/cmd_vel"

  // 단일 토픽 모음(공용)
  std::string flock_topic_{"/flock/odometry"};

  // 공통 파라미터
  double loop_rate_hz_{};
  double r_neighbor_{}, r_separation_{};
  double k_sep_{}, k_align_{}, k_coh_{}, k_bound_{};
  double max_speed_{}, max_force_{}, k_yaw_{}, stale_sec_{};
  bool use_boundary_{};
  double xmin_{}, xmax_{}, ymin_{}, ymax_{};

  // IO — 에이전트별
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> own_odom_subs_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> cmd_pubs_;

  // IO — 공용
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr flock_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr flock_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  // 상태 저장 (id -> state)
  std::mutex mtx_;
  std::unordered_map<std::string, AgentState> states_;
};

} // namespace flocking
