#pragma once
// Utility libraries
#include "ros/ros.h"

/* Husky Data Structures */
#include "utils/imu.hpp"
#include "utils/joint_state.hpp"
#include "utils/velocity.hpp"
#include "utils/camera_odom.hpp"

#include <mutex>
#include <thread>
#include <memory>
#include <queue>
#include <stack>

namespace husky_inekf {

struct husky_data_t {
    husky_data_t() {}

    std::mutex imu_mutex;
    std::mutex joint_state_mutex;
    std::mutex wheel_vel_mutex;
    std::mutex cam_vel_mutex;
    std::mutex gps_vel_mutex;
    std::queue<std::shared_ptr<ImuMeasurement<double> > > imu_q;
    // Use vector like a stack, using vector to enable O(1) clear operation
    std::queue<std::shared_ptr<JointStateMeasurement> > joint_state_q;
    std::queue<std::shared_ptr<CameraOdomMeasurement> > camera_odom_q;
    std::queue<std::shared_ptr<VelocityMeasurement> > wheel_velocity_q;
    std::queue<std::shared_ptr<VelocityMeasurement> > camera_velocity_q;
    std::queue<std::shared_ptr<VelocityMeasurement> > gps_velocity_q;
};

} // end husky_inekf namespace