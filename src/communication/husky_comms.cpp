#include "communication/husky_comms.h"

HuskyComms::HuskyComms(ros::NodeHandle n) : n_(n) {
    std::string imu_topic, odom_topic;

    n_.param<std::string>("imu_topic", imu_topic, "/zed_node/imu/data");
    n_.param<std::string>("odom_topic", odom_topic, "/gx5_0/nav/odom");

    imu_sub_.subscribe(n_, imu_topic, 100); // Set queue size to 0.1s of data assuming 1kHz
    odom_sub_.subscribe(n_, odom_topic, 100);

    // sync_.reset(new Sync(TimeSyncPolicy(10), sub_1_, sub_2_));
    // sync_->registerCallback(boost::bind(&Node::callback, this, _1, _2));
}

void HuskyComms::imuCallback(const sensor_msgs::Imu& msg) {
    
}

void HuskyComms::odomCallback(const nav_msgs::Odometry& msg) {
    
}