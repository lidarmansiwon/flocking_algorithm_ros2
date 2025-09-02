#include "flocking_algorithm_ros2/flocking_node.hpp"
#include "tf2/utils.hpp"
#include <cmath>
#include <cstdio>
#include "mss_msgs/msg/navigation.hpp"

using std::placeholders::_1;

namespace flocking
{

// %d 패턴 치환
std::string FlockingNode::make_topic_from_pattern(const std::string &pattern, size_t idx)
{
  char buf[256];
  std::snprintf(buf, sizeof(buf), pattern.c_str(), static_cast<int>(idx));
  return std::string(buf);
}

FlockingNode::FlockingNode() : rclcpp::Node("flocking_node")
{
  // --- 멀티 에이전트 파라미터 ---
  agent_count_      = static_cast<size_t>(this->declare_parameter<int>("agent_count", 1));
  agent_id_prefix_  = this->declare_parameter<std::string>("agent_id_prefix", "agent_");
  own_odom_pattern_ = this->declare_parameter<std::string>("own_odom_pattern", "/agent%d/navigation");
  cmd_vel_pattern_  = this->declare_parameter<std::string>("cmd_vel_pattern", "/agent_%d/cmd_vel");

  loop_rate_hz_   = this->declare_parameter<double>("loop_rate_hz", 20.0);

  // 보이드 반경 및 가중치
  r_neighbor_     = this->declare_parameter<double>("r_neighbor", 8.0);
  r_separation_   = this->declare_parameter<double>("r_separation", 2.0);
  k_sep_          = this->declare_parameter<double>("k_sep", 5.5);
  k_align_        = this->declare_parameter<double>("k_align", 0.6);
  k_coh_          = this->declare_parameter<double>("k_coh", 0.4);
  k_bound_        = this->declare_parameter<double>("k_bound", 1.0);

  // 속도/가속 Max
  max_speed_      = this->declare_parameter<double>("max_speed", 100.0);
  max_force_      = this->declare_parameter<double>("max_force", 100.0);

  // 요 제어 이득 (yaw_rate 모드에서만 사용)
  k_yaw_          = this->declare_parameter<double>("k_yaw", 1.5);

  // 상태 만료 시간
  stale_sec_      = this->declare_parameter<double>("stale_sec", 1.0);

  // 경계
  use_boundary_   = this->declare_parameter<bool>("use_boundary", true);
  xmin_           = this->declare_parameter<double>("xmin", -50.0);
  xmax_           = this->declare_parameter<double>("xmax", 50.0);
  ymin_           = this->declare_parameter<double>("ymin", -50.0);
  ymax_           = this->declare_parameter<double>("ymax", 50.0);

  // 리더 추종 관련
  leader_odom_topic_ = this->declare_parameter<std::string>("leader_odom_topic", "/agent_leader/navigation");
  leader_id_         = this->declare_parameter<std::string>("leader_id", "leader");
  k_leader_pos_      = this->declare_parameter<double>("k_leader_pos", 1.2);
  k_leader_align_    = this->declare_parameter<double>("k_leader_align", 0.5);

  // Twist 의미 설정
  twist_semantics_   = this->declare_parameter<std::string>("twist_semantics", "yaw_rate"); // or "yaw_rate"

  // --- 에이전트 ID 및 I/O 동적 생성 ---
  agent_ids_.resize(agent_count_);
  own_odom_subs_.resize(agent_count_);
  cmd_pubs_.resize(agent_count_);

  for (size_t i = 0; i < agent_count_; ++i)
  {
    agent_ids_[i] = agent_id_prefix_ + std::to_string(i);

    const std::string odom_topic = make_topic_from_pattern(own_odom_pattern_, i);
    const std::string cmd_topic  = make_topic_from_pattern(cmd_vel_pattern_,  i);

    // 각 에이전트별 /odom 구독
    own_odom_subs_[i] = this->create_subscription<mss_msgs::msg::Navigation>(
      odom_topic, rclcpp::QoS(50).best_effort(),
      [this, i](const mss_msgs::msg::Navigation::SharedPtr msg)
      { this->ownOdomCallbackIdx(i, msg); });

    // 각 에이전트별 /cmd_vel 퍼블리시
    cmd_pubs_[i] = this->create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);

    RCLCPP_INFO(get_logger(), "Agent[%zu]: id=%s, odom=%s, cmd=%s",
                i, agent_ids_[i].c_str(), odom_topic.c_str(), cmd_topic.c_str());
  }

  // 공용 플록 상태 브로드캐스트/수신
  flock_sub_ = this->create_subscription<mss_msgs::msg::Navigation>(
      "/flock/odometry", rclcpp::QoS(50).best_effort(),
      std::bind(&FlockingNode::flockOdomCallback, this, _1));

  flock_pub_ = this->create_publisher<mss_msgs::msg::Navigation>("/flock/odometry", rclcpp::QoS(50).best_effort());

  // 리더 Odom 구독
  leader_sub_ = this->create_subscription<mss_msgs::msg::Navigation>(
      leader_odom_topic_, rclcpp::QoS(50).best_effort(),
      std::bind(&FlockingNode::leaderOdomCallback, this, _1));

  const auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / loop_rate_hz_));
  timer_ = this->create_wall_timer(period, std::bind(&FlockingNode::controlTimerCB, this));

  RCLCPP_INFO(get_logger(), "Flocking node started. agent_count=%zu, leader_topic=%s, twist_semantics=%s",
              agent_count_, leader_odom_topic_.c_str(), twist_semantics_.c_str());
}

// --- 각 에이전트의 /odom 콜백 (인덱스로 구분) ---
void FlockingNode::ownOdomCallbackIdx(size_t idx, const mss_msgs::msg::Navigation::SharedPtr msg)
{
  if (idx >= agent_ids_.size()) return;
  const std::string &my_id = agent_ids_[idx];

  // 공용 토픽으로 재발행 (frame_id = my_id)
  auto repub = *msg;
  repub.header.frame_id = my_id;
  flock_pub_->publish(repub);

  // 내부 상태 저장
  AgentState self;
  self.pos = Eigen::Vector2d(msg->x, msg->y);
  self.vel = Eigen::Vector2d(msg->u, msg->v);
  self.yaw = msg->psi*pi/180;
  self.stamp = msg->header.stamp;

//   RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
//                        "RX ODOM [%s] stamp=(%u.%u) x=%.2f y=%.2f",
//                        my_id.c_str(), msg->header.stamp.sec, msg->header.stamp.nanosec,
//                        msg->x, msg->y);

  std::scoped_lock lk(mtx_);
  states_[my_id] = self;
}

// --- 공용 토픽 수신: 모든 에이전트 상태 갱신 ---
void FlockingNode::flockOdomCallback(const mss_msgs::msg::Navigation::SharedPtr msg)
{
  const std::string id = msg->header.frame_id.empty() ? std::string("unknown") : msg->header.frame_id;
  AgentState st;
  st.pos = Eigen::Vector2d(msg->x, msg->y);
  st.vel = Eigen::Vector2d(msg->u, msg->v);
  st.yaw = msg->psi*pi/180;
  st.stamp = msg->header.stamp;

  std::scoped_lock lk(mtx_);
  states_[id] = st;
}

// --- 리더 /agent_leader/odom 콜백 ---
void FlockingNode::leaderOdomCallback(const mss_msgs::msg::Navigation::SharedPtr msg)
{
  AgentState st;
  st.pos = Eigen::Vector2d(msg->x, msg->y);
  st.vel = Eigen::Vector2d(msg->u, msg->v);
  st.yaw = msg->psi*pi/180;
  st.stamp = msg->header.stamp;

  std::scoped_lock lk(mtx_);
  // 리더를 states_에도 포함시켜 이웃 회피/정렬/응집에 반영
  states_[leader_id_] = st;
}

// --- 주기 제어 루프 ---
void FlockingNode::controlTimerCB()
{
  // 오래된 이웃 제거
  {
    std::scoped_lock lk(mtx_);
    pruneStale(stale_sec_);
  }

  const double dt = 1.0 / loop_rate_hz_;

  // 모든 에이전트 반복
  for (size_t i = 0; i < agent_ids_.size(); ++i)
  {
    const std::string &my_id = agent_ids_[i];

    AgentState self;
    {
      std::scoped_lock lk(mtx_);
      auto it = states_.find(my_id);
      if (it == states_.end()) {
        // 아직 내 /odom 미수신 → 스킵
        continue;
      }
      self = it->second;
    }

    // 힘 합력 (leader + flock + boundary)
    Eigen::Vector2d f = Eigen::Vector2d::Zero();
    bool leader_ok = false;
    f += leaderForce(self, leader_ok);      // 리더 추종 (존재 시)
    f += k_sep_   * sepForce(self);         // 분리
    f += k_align_ * alignForce(self);       // 정렬
    f += k_coh_   * cohForce(self);         // 응집
    if (use_boundary_) f += k_bound_ * boundaryForce(self);
    f = sat2D(f, max_force_);

    // 속도 갱신 (이산 적분) + 포화
    Eigen::Vector2d v_des = self.vel + f * dt;
    v_des = sat2D(v_des, max_speed_);

    // 원하는 선수각 / 선수방향 속도 산출
    const double psi_des = (std::atan2(-v_des.y(), v_des.x()));
    const double u_des   = v_des.norm();

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x  = sat(u_des, max_speed_);

    if (twist_semantics_ == "yaw_rate") {
      // 기존과 동일: 각속도 r 명령

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10,
                   "agent_num [%s], psi_des [%f], self.yaw [%f]", my_id.c_str() ,psi_des*180/pi, self.yaw*180/pi);
      const double err = wrapAngle(psi_des - self.yaw);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10,
                   "psi_deg [%f]" ,err*180/pi);
    //   cmd.angular.z = k_yaw_ * err;
      cmd.angular.z = err*180/pi;
    } else {
      // 기본: 각도 ψ 자체를 angular.z에 담아 퍼블리시(수신측에서 ψ 명령으로 해석)
      cmd.angular.z = wrapAngle(psi_des);
    }

    if (cmd_pubs_[i]) cmd_pubs_[i]->publish(cmd);
  }
}

// --- Forces (이웃은 states_ 사용: mutex 보호) ---
Eigen::Vector2d FlockingNode::sepForce(const AgentState &self)
{
  Eigen::Vector2d acc = Eigen::Vector2d::Zero();
  int count = 0;
  std::scoped_lock lk(mtx_);
  for (const auto &kv : states_) {
    const auto &s  = kv.second;
    // 자기 자신 또는 동일 위치 필터
    if ((self.pos - s.pos).squaredNorm() < 1e-12) continue;
    const Eigen::Vector2d d = self.pos - s.pos;
    const double dist = d.norm();
    if (dist > 1e-6 && dist < r_separation_) {
      acc += d.normalized() * (1.0 / (dist * dist)); // 가까울수록 강한 반발
      count++;
    }
  }
  if (count > 0) acc /= static_cast<double>(count);
  return sat2D(acc, max_force_);
}

// 정렬
Eigen::Vector2d FlockingNode::alignForce(const AgentState &self)
{
  Eigen::Vector2d avg_v = Eigen::Vector2d::Zero();
  int count = 0;
  std::scoped_lock lk(mtx_);
  for (const auto &kv : states_) {
    const auto &s = kv.second;
    const double dist = (self.pos - s.pos).norm();
    if (dist < r_neighbor_ && dist > 1e-6) {
      avg_v += s.vel;
      count++;
    }
  }
  if (count == 0) return Eigen::Vector2d::Zero();
  avg_v /= static_cast<double>(count);
  Eigen::Vector2d steer = avg_v - self.vel;
  return sat2D(steer, max_force_);
}

// 응집
Eigen::Vector2d FlockingNode::cohForce(const AgentState &self)
{
  Eigen::Vector2d center = Eigen::Vector2d::Zero();
  int count = 0;
  std::scoped_lock lk(mtx_);
  for (const auto &kv : states_) {
    const auto &s = kv.second;
    const double dist = (self.pos - s.pos).norm();
    if (dist < r_neighbor_ && dist > 1e-6) {
      center += s.pos;
      count++;
    }
  }
  if (count == 0) return Eigen::Vector2d::Zero();
  center /= static_cast<double>(count);
  Eigen::Vector2d desired = center - self.pos;
  if (desired.norm() > 1e-6) desired = desired.normalized() * std::min(max_speed_, desired.norm());
  Eigen::Vector2d steer = desired - self.vel;
  return sat2D(steer, max_force_);
}

// 영역 유지
Eigen::Vector2d FlockingNode::boundaryForce(const AgentState &self)
{
  const double margin = r_neighbor_ * 0.5;
  Eigen::Vector2d f = Eigen::Vector2d::Zero();
  const double x = self.pos.x();
  const double y = self.pos.y();
  if (x - xmin_ < margin) f.x() += 1.0 / std::max(1e-3, (x - xmin_));
  if (xmax_ - x < margin) f.x() -= 1.0 / std::max(1e-3, (xmax_ - x));
  if (y - ymin_ < margin) f.y() += 1.0 / std::max(1e-3, (y - ymin_));
  if (ymax_ - y < margin) f.y() -= 1.0 / std::max(1e-3, (ymax_ - y));
  return sat2D(f, max_force_);
}

// 리더 추종 항
Eigen::Vector2d FlockingNode::leaderForce(const AgentState &self, bool &leader_ok)
{
  leader_ok = false;
  AgentState leader{};
  {
    std::scoped_lock lk(mtx_);
    auto it = states_.find(leader_id_);
    if (it == states_.end()) {
      return Eigen::Vector2d::Zero();
    }
    leader = it->second;
  }

  // 리더가 너무 오래되었는지 pruneStale에서 관리되므로 여기선 단순 사용
  leader_ok = true;

  // 위치 추종 성분
  Eigen::Vector2d to_leader = leader.pos - self.pos;
  Eigen::Vector2d desired = to_leader;
  if (desired.norm() > 1e-6) {
    desired = desired.normalized() * std::min(max_speed_, desired.norm());
  }
  Eigen::Vector2d f_pos = desired - self.vel;

  // 속도 정렬 성분
  Eigen::Vector2d f_align = leader.vel - self.vel;

  // 너무 가까우면 강한 분리(리더와의 충돌 회피)
  Eigen::Vector2d f_rep = Eigen::Vector2d::Zero();
  const double dist = to_leader.norm();
  if (dist > 1e-6 && dist < r_separation_) {
    f_rep = -(to_leader.normalized()) * (1.0 / (dist * dist));
  }

  Eigen::Vector2d f = k_leader_pos_ * f_pos + k_leader_align_ * f_align + k_sep_ * f_rep;
  return sat2D(f, max_force_);
}

void FlockingNode::pruneStale(double max_age_sec)
{
  const rclcpp::Time now = this->now();
  for (auto it = states_.begin(); it != states_.end(); ) {
    const double age = (now - it->second.stamp).seconds();
    if (age > max_age_sec) it = states_.erase(it);
    else ++it;
  }
}

// --- utils ---
double FlockingNode::wrapAngle(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

double FlockingNode::sat(double x, double limit)
{
  if (x >  limit) return  limit;
  if (x < -limit) return -limit;
  return x;
}

Eigen::Vector2d FlockingNode::sat2D(const Eigen::Vector2d &v, double limit)
{
  const double n = v.norm();
  if (n <= limit || n < 1e-9) return v;
  return v * (limit / n);
}

} // namespace flocking
