#include "flocking_algorithm_ros2/flocking_node.hpp"
#include "tf2/utils.hpp"

using std::placeholders::_1;

namespace flocking
{

FlockingNode::FlockingNode() : rclcpp::Node("flocking_node")
{
    agent_id_       = this->declare_parameter<std::string>("agent_id", "agent_01");
    own_odom_topic_ = this->declare_parameter<std::string>("own_odom_topic", "/odom");
    cmd_vel_topic_  = this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    loop_rate_hz_   = this->declare_parameter<double>("loop_rate_hz", 20.0);


    r_neighbor_     = this->declare_parameter<double>("r_neighbor", 8.0);
    r_separation_   = this->declare_parameter<double>("r_separation", 2.0);
    k_sep_          = this->declare_parameter<double>("k_sep", 1.5);
    k_align_        = this->declare_parameter<double>("k_align", 0.6);
    k_coh_          = this->declare_parameter<double>("k_coh", 0.4);
    k_bound_        = this->declare_parameter<double>("k_bound", 1.0);
    max_speed_      = this->declare_parameter<double>("max_speed", 2.0);
    max_force_      = this->declare_parameter<double>("max_force", 1.0);
    k_yaw_          = this->declare_parameter<double>("k_yaw", 1.5);
    stale_sec_      = this->declare_parameter<double>("stale_sec", 1.0);


    use_boundary_   = this->declare_parameter<bool>("use_boundary", true);
    xmin_           = this->declare_parameter<double>("xmin", -50.0);
    xmax_           = this->declare_parameter<double>("xmax", 50.0);
    ymin_           = this->declare_parameter<double>("ymin", -50.0);
    ymax_           = this->declare_parameter<double>("ymax", 50.0);   

    // Publishers & Subscribers
    own_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        own_odom_topic_, rclcpp::SensorDataQoS(),  
        std::bind(&FlockingNode::ownOdomCallback, this, _1)  
    );

    flock_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/flock/odometry", rclcpp::QoS(50).best_effort(),
    std::bind(&FlockingNode::flockOdomCallback, this, _1));


    flock_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/flock/odometry", rclcpp::QoS(50).best_effort());
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);


    const auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / loop_rate_hz_));
    timer_ = this->create_wall_timer(period, std::bind(&FlockingNode::controlTimerCB, this));


    RCLCPP_INFO(get_logger(), "Flocking node started. id=%s", agent_id_.c_str());
}

void FlockingNode::ownOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Republish to shared flock topic with frame_id = agent_id_
    auto repub = *msg;
    repub.header.frame_id = agent_id_;
    flock_pub_->publish(repub);

    // store as self state
    AgentState self;
    self.pos = Eigen::Vector2d(msg->pose.pose.position.x, msg->pose.pose.position.y);
    self.vel = Eigen::Vector2d(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
    self.yaw = tf2::getYaw(msg->pose.pose.orientation);
    self.stamp = msg->header.stamp;

    std::scoped_lock lk(mtx_);
    states_[agent_id_] = self;
}

void FlockingNode::flockOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    const std::string id = msg->header.frame_id.empty() ? std::string("unknown") : msg->header.frame_id;
    AgentState st;
    st.pos = Eigen::Vector2d(msg->pose.pose.position.x, msg->pose.pose.position.y);
    st.vel = Eigen::Vector2d(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
    st.yaw = tf2::getYaw(msg->pose.pose.orientation);
    st.stamp = msg->header.stamp;


    std::scoped_lock lk(mtx_);
    states_[id] = st;
}


void FlockingNode::controlTimerCB()
{
    // Collect self
    AgentState self;
    {
        std::scoped_lock lk(mtx_);
        pruneStale(stale_sec_);
        auto it = states_.find(agent_id_);
        if (it == states_.end()) {
            // 아직 own odom 수신 전
            return;
    }
        self = it->second;
}


    // Compute forces
    Eigen::Vector2d f = Eigen::Vector2d::Zero();
    f += k_sep_ * sepForce(self);
    f += k_align_ * alignForce(self);
    f += k_coh_ * cohForce(self);
    if (use_boundary_) f += k_bound_ * boundaryForce(self);


    f = sat2D(f, max_force_);


    // Desired velocity update (simple Euler step)
    const double dt = 1.0 / loop_rate_hz_;
    Eigen::Vector2d v_des = self.vel + f * dt;
    v_des = sat2D(v_des, max_speed_);


    // Convert to unicycle Twist (linear.x & angular.z)
    const double speed = v_des.norm();
    const double heading_des = std::atan2(v_des.y(), v_des.x());
    const double err = wrapAngle(heading_des - self.yaw);


    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = speed; // surge
    cmd.angular.z = k_yaw_ * err; // yaw rate command
    cmd_pub_->publish(cmd);
}

Eigen::Vector2d FlockingNode::sepForce(const AgentState &self)
{
    // Strong repulsion when within r_separation
    Eigen::Vector2d acc = Eigen::Vector2d::Zero();
    int count = 0;
    std::scoped_lock lk(mtx_);
    for (const auto &kv : states_) {
        const auto &id = kv.first;
        if (id == agent_id_) continue;
        const auto &s = kv.second;
        const Eigen::Vector2d d = self.pos - s.pos;
        const double dist = d.norm();
        if (dist > 1e-6 && dist < r_separation_) {
            // inverse-square repulsion
            acc += d.normalized() * (1.0 / (dist * dist));
            count++;
        }
    }
    if (count > 0) acc /= static_cast<double>(count);
    return sat2D(acc, max_force_);
}


Eigen::Vector2d FlockingNode::alignForce(const AgentState &self)
{
    Eigen::Vector2d avg_v = Eigen::Vector2d::Zero();
    int count = 0;
    std::scoped_lock lk(mtx_);
    for (const auto &kv : states_) {
        const auto &id = kv.first;
        if (id == agent_id_) continue;
        const auto &s = kv.second;
        const double dist = (self.pos - s.pos).norm();
        if (dist < r_neighbor_) {
            avg_v += s.vel;
            count++;
        }
    }
    if (count == 0) return Eigen::Vector2d::Zero();
    avg_v /= static_cast<double>(count);
    // steer towards average velocity
    Eigen::Vector2d steer = avg_v - self.vel;
    return sat2D(steer, max_force_);
}

Eigen::Vector2d FlockingNode::cohForce(const AgentState &self)
{
    Eigen::Vector2d center = Eigen::Vector2d::Zero();
    int count = 0;
    std::scoped_lock lk(mtx_);
    for (const auto &kv : states_) {
        const auto &id = kv.first;
        if (id == agent_id_) continue;
        const auto &s = kv.second;
        const double dist = (self.pos - s.pos).norm();
        if (dist < r_neighbor_) {
            center += s.pos;
            count++;
        }
    }
    if (count == 0) return Eigen::Vector2d::Zero();
    center /= static_cast<double>(count);
    // desired direction to center
    Eigen::Vector2d desired = center - self.pos;
    // target speed proportional to distance (clamped)
    if (desired.norm() > 1e-6) desired = desired.normalized() * std::min(max_speed_, desired.norm());
    Eigen::Vector2d steer = desired - self.vel;
    return sat2D(steer, max_force_);
}

Eigen::Vector2d FlockingNode::boundaryForce(const AgentState &self)
{
    // Simple rectangular keep-in using 1/distance repulsion near walls
    const double margin = r_neighbor_ * 0.5; // start pushing before hitting boundary
    Eigen::Vector2d f = Eigen::Vector2d::Zero();


    const double x = self.pos.x();
    const double y = self.pos.y();


    if (x - xmin_ < margin) f.x() += 1.0 / std::max(1e-3, (x - xmin_));
    if (xmax_ - x < margin) f.x() -= 1.0 / std::max(1e-3, (xmax_ - x));
    if (y - ymin_ < margin) f.y() += 1.0 / std::max(1e-3, (y - ymin_));
    if (ymax_ - y < margin) f.y() -= 1.0 / std::max(1e-3, (ymax_ - y));


    return sat2D(f, max_force_);
}

void FlockingNode::pruneStale(double max_age_sec)
{
    const rclcpp::Time now = this->now();
    for (auto it = states_.begin(); it != states_.end(); )
    {
        const double age = (now - it->second.stamp).seconds();
        if (age > max_age_sec)
        it = states_.erase(it);
        else
        ++it;
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
    if (x > limit) return limit;
    if (x < -limit) return -limit;
    return x;
}


Eigen::Vector2d FlockingNode::sat2D(const Eigen::Vector2d &v, double limit)
{
    const double n = v.norm();
    if (n <= limit || n < 1e-9) return v;
    return v * (limit / n);
}

}