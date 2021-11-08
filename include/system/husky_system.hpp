#ifndef CHEETAHSYSTEM_H
#define CHEETAHSYSTEM_H

#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <memory>
#include "ros/ros.h"


#include "estimator/body_estimator.hpp"
#include "system/husky_state.hpp"
#include "utils/husky_data.hpp"
#include "utils/joint_state.hpp"
#include "utils/imu.hpp"

#include "communication/pose_publisher_node.hpp"

// Threading
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/circular_buffer.hpp>

// TODO: Singleton design pattern (there should only be one HuskySystem)
class HuskySystem {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        // Default Contructor
        HuskySystem(ros::NodeHandle* nh, husky_inekf::husky_data_t* husky_data_buffer);
        // Step forward one iteration of the system
        void step();

    private:
        // ROS NodeHandle
        ros::NodeHandle* nh_;
        // ROS pose publisher
        PosePublisherNode pose_publisher_node_;
        // ROS timestamp
        ros::Time timestamp_;
        // Passive Time Synchronizer
        PassiveTimeSync ts_;
        // Cassie's current state estimate
        HuskyState state_;
        // Husky data queues
        husky_inekf_data::husky_data_t* husky_buffer_;
        // Cheetah lcm data queue mtx
        boost::mutex* cdata_mtx_;
        // Invariant extended Kalman filter for estimating the robot's body state
        BodyEstimator estimator_;
        // Most recent data packet
        husky_inekf_data::JointStateMeasurement joint_state_packet_;
        husky_inekf_data::ImuMeasurement imu_packet_;

        // Update most recent packet to use
        bool updateNextIMU();
        bool updateNextJointState();

        // Publish output path
        void poseCallback(const HuskyState& state_);
        // Output file
        std::string file_name_;
        std::string tum_file_name_;
        // Publish path node enable flag
        bool enable_pose_publisher_;
};

#endif // CHEETAHSYSTEM_H