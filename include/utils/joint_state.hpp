#pragma once
#include "utils/measurement.h"
#include <stdint.h>
#include <string>

namespace husky_inekf_data {
    class JointStateMeasurement : public Measurement {
        public:
            // Construct Encoder measurement
            JointStateMeasurement(unsigned int ENCODER_DIM) {
                type_ = JOINT_STATE;
                encoder_dim_ = ENCODER_DIM;
            }
        
            // void setKinematicsArray(const inekf_msgs::KinematicsArray& kinematics) {
            //     kin_arr = kinematics;
            // }

            // const inekf_msgs::KinematicsArray& getKinematicsArray() {
            //     return kin_arr;
            // }

            Eigen::VectorXd joint_position;
            Eigen::VectorXd joint_velocity;
            Eigen::VectorXd joint_effort;

            // linear wheel velocity
            // linear_velocity = joint_velocity * wheel_radius
            // wheel_radius = 0.1651 m
            // https://github.com/husky/husky/blob/noetic-devel/husky_description/urdf/husky.urdf.xacro#L79

            Eigen::VectorXd linear_velocity;
            
            // body velocity from wheel encoders
            // body_lin_vel = (v_r + v_l) / 2
            // body_ang_vel = (v_r-v_l)/w
            // w = 0.555 m
            Eigen::VectorXd body_lin_vel;
            Eigen::VectorXd body_ang_vel;

        private:
            unsigned int encoder_dim_;
    };
}