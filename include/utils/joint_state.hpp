#pragma once

/* ROS specific interface */
#include "sensor_msgs/JointState.h"
#include <unsupported/Eigen/MatrixFunctions>

#include "utils/measurement.h"
#include <stdint.h>
#include <vector>
#include <string>
#include <memory>

namespace husky_inekf {

    class JointStateMeasurement : public Measurement {
        public:
            // Construct Encoder measurement
            JointStateMeasurement(unsigned int ENCODER_DIM) {
                type_ = JOINT_STATE;
                encoder_dim_ = ENCODER_DIM;
            }
            
            // Constructor with default wheel radius
            JointStateMeasurement(  const sensor_msgs::JointState& joint_msg,
                                    unsigned int ENCODER_DIM): 
                                    encoder_dim_(ENCODER_DIM){

                wheel_radius_ = 0.1651;
                vehicle_track_width_ = 0.555;
                vehicle_length_ = 0.540;
                type_ = JOINT_STATE;
                joint_position_.resize(encoder_dim_, 1);
                joint_velocity_.resize(encoder_dim_, 1);
                joint_effort_.resize(encoder_dim_, 1);
                linear_velocity_.resize(encoder_dim_, 1);
                // TODO: Make this initialization more flexible if necessary
                body_lin_vel_.resize(3, 1);
                body_ang_vel_.resize(3, 1);
                // Initialize Robot State
                setJointPosition(joint_msg.position);
                setJointVelocity(joint_msg.velocity);
                setJointEffort(joint_msg.effort);
                setLinearVelocity();
                setBodyVelocity();

                setHeader(joint_msg.header);
            }

            JointStateMeasurement(  const sensor_msgs::JointState& joint_msg,
                                    unsigned int ENCODER_DIM,
                                    double wheel_radius_in,
                                    double vehicle_track_width_in,
                                    double vehicle_length_in): 
                                    encoder_dim_(ENCODER_DIM),
                                    wheel_radius_(wheel_radius_in),
                                    vehicle_track_width_(vehicle_track_width_in),
                                    vehicle_length_(vehicle_length_in){
                type_ = JOINT_STATE;
                joint_position_.resize(encoder_dim_, 1);
                joint_velocity_.resize(encoder_dim_, 1);
                joint_effort_.resize(encoder_dim_, 1);
                linear_velocity_.resize(encoder_dim_, 1);
                // TODO: Make this initialization more flexible if necessary
                body_lin_vel_.resize(3, 1);
                body_ang_vel_.resize(3, 1);
                // Initialize Robot State
                setJointPosition(joint_msg.position);
                setJointVelocity(joint_msg.velocity);
                setJointEffort(joint_msg.effort);
                setLinearVelocity();
                setBodyVelocity();

                setHeader(joint_msg.header);
            }

            void setJointPosition(const std::vector<double>& position) {
                for (int i = 0; i < encoder_dim_; ++i) {
                    joint_position_(i) = position[i];
                }
            }

            void setJointVelocity(const std::vector<double>& velocity) {
                for (int i = 0; i < encoder_dim_; ++i) {
                    joint_velocity_(i) = velocity[i];
                }
            }

            void setJointEffort(const std::vector<double>& effort) {
                for (int i = 0; i < encoder_dim_; ++i) {
                    joint_effort_(i) = effort[i];
                }
            }

            void setLinearVelocity() {
                for (int i = 0; i < encoder_dim_; ++i) {
                    linear_velocity_(i) = joint_velocity_(i) * wheel_radius_; 
                }
            }

            void setBodyVelocity() {
                body_lin_vel_.setZero();
                body_ang_vel_.setZero();

                double vr = (linear_velocity_(1) + linear_velocity_(3)) / 2.0;
                double vl = (linear_velocity_(0) + linear_velocity_(2)) / 2.0;
                body_lin_vel_(0) = (vr + vl) / 2.0 ; // [x y z]
                body_ang_vel_(2) = (vr - vl) / vehicle_track_width_* std::pow(std::cos(std::atan2(vehicle_length_, vehicle_track_width_)), 2); // [dx dy dz]
            }

            inline const Eigen::VectorXd& getJointPosition() const {
                return joint_position_;
            }

            inline const Eigen::VectorXd& getJointVelocity() const {
                return joint_velocity_;
            }

            inline const Eigen::VectorXd& getJointEffort() const {
                return joint_effort_;
            }

            inline const Eigen::VectorXd& getLinearVelocity() const {
                return linear_velocity_;
            }

            inline const Eigen::VectorXd&  getBodyLinearVelocity() const {
                return body_lin_vel_;
            }

            inline const Eigen::VectorXd&  getBodyAngularVelocity() const {
                return body_ang_vel_;
            }
    
        private:
            double wheel_radius_;
            // static constexpr double wheel_radius_ = 0.3;

            double vehicle_track_width_;
            double vehicle_length_;
            unsigned int encoder_dim_; 

            // Joint Encoder Information From Wheels
            //  [front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel]
            // https://github.com/husky/husky/issues/172
            Eigen::VectorXd joint_position_;
            Eigen::VectorXd joint_velocity_;
            Eigen::VectorXd joint_effort_;

            // linear wheel velocity
            // linear_velocity = joint_velocity * wheel_radius
            // wheel_radius = 0.1651 m
            // https://github.com/husky/husky/blob/noetic-devel/husky_description/urdf/husky.urdf.xacro#L79

            Eigen::VectorXd linear_velocity_;
            
            // body velocity from wheel encoders
            // body_lin_vel = (v_r + v_l) / 2
            // body_ang_vel = (v_r-v_l)/w
            // w = 0.555 m
            Eigen::VectorXd body_lin_vel_;
            Eigen::VectorXd body_ang_vel_;
    };

    typedef std::shared_ptr<JointStateMeasurement> JointStateMeasurementPtr;
} // end husky_inekf namespace