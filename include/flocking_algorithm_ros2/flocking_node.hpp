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
    void ownOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void flockOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void controlTimerCB();

    // Boids terms
    Eigen::Vector2d sepForce(const AgentState &self);
    Eigen::Vector2d alignForce(const AgentState &self);
    Eigen::Vector2d cohForce(const AgentState &self);
    Eigen::Vector2d boundaryForce(const AgentState &self);

    // Helpers
    static double wrapAngle(double a);
    static double sat(double x, double limit);
    static Eigen::Vector2d sat2D(const Eigen::Vector2d &v,double limit);
    void pruneStale(double max_age_sec);

private:
    //Params
    std::string agent_id_;
    std::string own_odom_topic_;
    std::string cmd_vel_topic_;
    double loop_rate_hz_{};

    double r_neighbor_{}; // neighbor radius for align/cohesion
    double r_separation_{}; // separation radius
    double k_sep_{}; // weights
    double k_align_{};
    double k_coh_{};
    double k_bound_{};
    double max_speed_{}; // desired speed cap [m/s]
    double max_force_{}; // acceleration cap [m/s^2]
    double k_yaw_{}; // yaw P-gain to track desired heading
    double stale_sec_{}; // state timeout

    bool use_boundary_{};
    double xmin_{}, xmax_{}, ymin_{}, ymax_{}; // rectangular boundary 정의

    // IO
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr own_odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr flock_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr flock_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;


    // State storage (id -> state)
    std::mutex mtx_;
    std::unordered_map<std::string, AgentState> states_;
};

}