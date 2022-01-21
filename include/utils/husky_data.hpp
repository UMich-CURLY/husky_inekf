#pragma once
// Utility libraries
#include "ros/ros.h"

/* Husky Data Structures */
#include "utils/imu.hpp"
#include "utils/joint_state.hpp"
#include "utils/velocity.hpp"

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
    std::mutex velocity_mutex;
    std::queue<std::shared_ptr<ImuMeasurement<double> > > imu_q;
    // Use vector like a stack, using vector to enable O(1) clear operation
    std::queue<std::shared_ptr<JointStateMeasurement> > joint_state_q;
    std::queue<std::shared_ptr<VelocityMeasurement> > velocity_q;
};

} // end husky_inekf namespace