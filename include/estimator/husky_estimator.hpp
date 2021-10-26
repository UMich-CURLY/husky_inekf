#pragma once
#ifndef BODYESTIMATOR_H
#define BODYESTIMATOR_H

#include <Eigen/Dense>
#include <vector>
#include "ros/ros.h"
#include "utils/cheetah_data_t.hpp"
#include "system/cheetah_state.hpp"
#include "InEKF.h"
#include <lcm/lcm-cpp.hpp>
#include "communication/lcm-types/cheetah_inekf_lcm/pose_t.hpp"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Path.h"
// #include "visualization_msgs/MarkerArray.h"
// #include "inekf_msgs/State.h"

class BodyEstimator {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        BodyEstimator(lcm::LCM* lcm);
        bool enabled();
        void enableFilter();
        void disable();
        bool biasInitialized();
        void initBias(cheetah_lcm_packet_t& cheetah_data);
        // void initState();
        void initState(const double t, const cheetah_lcm_packet_t& cheetah_data, const CheetahState& state);
        void setContacts(CheetahState& state);
        void update(cheetah_lcm_packet_t& cheetah_data, CheetahState& state);
        void correctKinematics(CheetahState& state);
        inekf::InEKF getFilter() const;
        inekf::RobotState getState() const;
        void publishMarkers(double time, std::string map_frame_id, uint32_t seq);
        void publishPath();
        void publishPose(double time, std::string map_frame_id, uint32_t seq);

    private:
        // LCM related
        lcm::LCM* lcm_;
        std::string LCM_POSE_CHANNEL;
        // ROS related
        ros::Publisher visualization_pub_;
        std::vector<geometry_msgs::PoseStamped> poses_;
        // inekf related
        inekf::InEKF filter_;
        bool enabled_ = false;
        bool bias_initialized_ = false;
        bool static_bias_initialization_ = false;
        bool estimator_debug_enabled_ = false;
        bool lcm_publish_visualization_markers_ = false;
        std::vector<Eigen::Matrix<double,6,1>,Eigen::aligned_allocator<Eigen::Matrix<double,6,1>>> bias_init_vec_;
        Eigen::Vector3d bg0_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d ba0_ = Eigen::Vector3d::Zero();
        double t_prev_;
        uint32_t seq_;
        Eigen::Matrix<double,6,1> imu_prev_;
        const Eigen::Matrix<double,12,12> encoder_cov_ = 0.0174533*0.0174533 * Eigen::Matrix<double,12,12>::Identity(); // 1 deg std dev 
        const Eigen::Matrix<double,3,3> prior_kinematics_cov_ = 0.05*0.05 * Eigen::Matrix<double,3,3>::Identity(); // 5 cm std Adds to FK covariance
};

#endif // BODYESTIMATOR_H