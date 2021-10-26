#ifndef CHEETAHSYSTEM_H
#define CHEETAHSYSTEM_H

#include <Eigen/Dense>
#include "estimator/body_estimator.hpp"
#include "system/cheetah_state.hpp"
#include <iostream>
#include <fstream>
#include <memory>
#include "ros/ros.h"
#include "utils/cheetah_data_t.hpp"
#include "utils/PassiveTimeSync.h"
#include "pose_publisher_node.hpp"

// Threading
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/circular_buffer.hpp>

// TODO: Singleton design pattern (there should only be one CheetahSystem)
class CheetahSystem {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        // Default Contructor
        CheetahSystem(lcm::LCM* lcm, ros::NodeHandle* nh, boost::mutex* cdata_mtx, cheetah_lcm_data_t* cheetah_buffer);
        // Step forward one iteration of the system
        void step();

    private:
        // LCM handle
        lcm::LCM* lcm_;
        // ROS NodeHandle
        ros::NodeHandle* nh_;
        // ROS pose publisher
        PosePublisherNode pose_publisher_node_;
        // ROS timestamp
        ros::Time timestamp_;
        // Passive Time Synchronizer
        PassiveTimeSync ts_;
        // Cassie's current state estimate
        CheetahState state_;
        // Cheetah lcm data queues
        cheetah_lcm_data_t* cheetah_buffer_;
        // Cheetah lcm data queue mtx
        boost::mutex* cdata_mtx_;
        // Invariant extended Kalman filter for estimating the robot's body state
        BodyEstimator estimator_;
        // Most recent data packet
        cheetah_lcm_packet_t cheetah_packet_;
        // Update most recent packet to use
        bool updateNextPacket();
        // Publish output path
        void poseCallback(const CheetahState& state_);
        // Output file
        std::string file_name_;
        std::string tum_file_name_;
        // Publish path node enable flag
        bool enable_pose_publisher_;
};

#endif // CHEETAHSYSTEM_H