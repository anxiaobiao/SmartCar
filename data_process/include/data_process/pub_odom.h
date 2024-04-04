#include"rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include"geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

class PubOdom : public rclcpp::Node
{
private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _Odom_topic;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _twist;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // 发布里程计话题函数
    void pub_odom(const geometry_msgs::msg::Twist::SharedPtr input_twist);

public:
    PubOdom();

private:
    rclcpp::Time _last_time;

    bool pub_odom_tf_ = false;
    std::string odom_frame = "odom";
    std::string base_footprint_frame = "base_footprint";

    double _xPos;
    double _yPos;
    double _heading;

};