#include"../include/data_process/pub_odom.h"

PubOdom::PubOdom() : Node("PublicOdometry")
{
    RCLCPP_INFO(this->get_logger(), "public odometry topic");

    this->declare_parameter<std::string>("odom_frame","odom");
    this->declare_parameter<std::string>("base_footprint_frame","base_footprint"); 

    this->get_parameter<std::string>("odom_frame",odom_frame);
    this->get_parameter<std::string>("base_footprint_frame",base_footprint_frame);
    this->get_parameter<bool>("pub_odom_tf",pub_odom_tf_);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    _Odom_topic = this->create_publisher<nav_msgs::msg::Odometry>(
        "/odom_raw",
        10
    );

    _twist = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_self_vel",
        10,
        std::bind(&PubOdom::pub_odom, this, std::placeholders::_1)
    );
}

void PubOdom::pub_odom(const geometry_msgs::msg::Twist::SharedPtr input_twist)
{
    // 计算积分时间
    rclcpp::Time curren_time = rclcpp::Clock().now();
    double dt = (curren_time - _last_time).seconds();
    _last_time = curren_time;

    double vel_x = input_twist->linear.x;
    double vel_y = input_twist->linear.y;
    double vel_angle = input_twist->angular.z;

    double delta_heading = vel_angle * dt; //radians
    double delta_x = (vel_x * cos(_heading)-vel_y*sin(_heading)) * dt; //m
    double delta_y = (vel_x * sin(_heading)+vel_y*cos(_heading)) * dt; //m	
    _xPos += delta_x;
    _yPos += delta_y;
    _heading += delta_heading;

    tf2::Quaternion myQuaternion;
    geometry_msgs::msg::Quaternion odom_quat ; 
    myQuaternion.setRPY(0.00,0.00,_heading );

    odom_quat.x = myQuaternion.x();
    odom_quat.y = myQuaternion.y();
    odom_quat.z = myQuaternion.z();
    odom_quat.w = myQuaternion.w();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = curren_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    // robot's position in x,y and z
    odom.pose.pose.position.x = _xPos;
    odom.pose.pose.position.y = _yPos;
    odom.pose.pose.position.z = 0.0;
    // robot's heading in quaternion

    odom.pose.pose.orientation = odom_quat;
    odom.pose.covariance[0] = 0.001;
    odom.pose.covariance[7] = 0.001;
    odom.pose.covariance[35] = 0.001;
    // linear speed from encoders
    odom.twist.twist.linear.x = vel_x;
    odom.twist.twist.linear.y = vel_y;
    odom.twist.twist.linear.y = 0.0; // vy = 0.0
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    // angular speed from encoders
    odom.twist.twist.angular.z = vel_angle;
    odom.twist.covariance[0] = 0.0001;
    odom.twist.covariance[7] = 0.0001;

    // RCLCPP_INFO(this->get_logger(), "或得到的数据：%.2f, %.2f, %.2f", odom_quat.x, odom_quat.y, odom_quat.z);

    _Odom_topic -> publish(odom);

    // if (pub_odom_tf_)
    // {
    
        geometry_msgs::msg::TransformStamped t;
        rclcpp::Time now = this->get_clock()->now();
        t.header.stamp = now;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_footprint";
        t.transform.translation.x = _xPos;
        t.transform.translation.y = _yPos;
        t.transform.translation.z = 0.0;
        
        t.transform.rotation.x = myQuaternion.x();
        t.transform.rotation.y = myQuaternion.y();
        t.transform.rotation.z = myQuaternion.z();
        t.transform.rotation.w = myQuaternion.w();
        
        tf_broadcaster_->sendTransform(t);
            
    // }
}

int main(int argc ,char** argv)
{
    rclcpp::init(argc, argv);

    auto pub_odom = std::make_shared<PubOdom>();

    rclcpp::spin(pub_odom);

    rclcpp::shutdown();

    return 0;
}
