#include "communication/husky_comms.h"

HuskyComms::HuskyComms( ros::NodeHandle nh, husky_inekf_data::husky_data_t* husky_data_buffer)
                        : nh_(nh), husky_data_buffer_(husky_data_buffer) 
{
    std::string imu_topic, odom_topic;

    nh_.param<std::string>("imu_topic", imu_topic, "/zed_node/imu/data");
    nh_.param<std::string>("odom_topic", odom_topic, "/gx5_0/nav/odom");

    // Initialize subscribers with queue size of 1000
    imu_sub_ = nh_.subscribe(imu_topic, 1000, &HuskyComms::imuCallback, this);
    odom_sub_ = nh_.subscribe(odom_topic, 1000, &HuskyComms::odomCallback, this);
}

// Note no transformation mat needed between imu and odometry for husky
void HuskyComms::imuCallback(const sensor_msgs::Imu& imu_msg) 
{
    std::shared_ptr<sensor_msgs::Imu> imu_ptr = 
                        std::make_shared<sensor_msgs::Imu>(imu_msg);

    std::lock_guard<std::mutex> lock(husky_data_buffer_->imu_mutex);
    husky_data_buffer_->imu_q.push(imu_ptr);
}


void HuskyComms::odomCallback(const nav_msgs::Odometry& odom_msg)
{
    std::shared_ptr<nav_msgs::Odometry> odom_ptr = 
                        std::make_shared<nav_msgs::Odometry>(odom_msg);

    std::lock_guard<std::mutex> lock(husky_data_buffer_->odom_mutex);
    husky_data_buffer_->odom_q.push(odom_ptr);
}
