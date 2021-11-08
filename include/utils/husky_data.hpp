#pragma once
// Utility libraries
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"

#include <mutex>
#include <thread>
#include <memory>
#include <queue>

namespace husky_inekf_data {

struct husky_data_t {
    husky_data_t() {}

    std::mutex imu_mutex;
    std::mutex odom_mutex;
    std::queue<std::shared_ptr<sensor_msgs::Imu>> imu_q;
    std::queue<std::shared_ptr<sensor_msgs::JointState>> joint_state_q;
    std::queue<std::shared_ptr<nav_msgs::Odometry>> odom_q;
};


}