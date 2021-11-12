#pragma once

/* ROS specific interface */
#include "sensor_msgs/JointState.h"

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

            JointStateMeasurement(  const sensor_msgs::JointState& joint_msg,
                                    unsigned int ENCODER_DIM): 
                                    encoder_dim_(encoder_dim_) {
                type_ = JOINT_STATE;
                joint_position_.resize(encoder_dim_);
                joint_velocity_.resize(encoder_dim_);
                joint_effort_.resize(encoder_dim_);
                linear_velocity_.resize(encoder_dim_);

                // Initialize Robot State
                setJointPosition(joint_msg.position);
                setJointVelocity(joint_msg.velocity);
                setJointEffort(joint_msg.effort);
                setLinearVelocity();
                setBodyVelocity();
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
                double vr = (linear_velocity_(1) + linear_velocity_(3)) / 2.0;
                double vl = (linear_velocity_(0) + linear_velocity_(2)) / 2.0;
                body_lin_vel_ = (vr + vl) / 2.0;
                body_ang_vel_ = (vr - vl) / vehicle_track_width_;
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

            inline double getBodyLinearVelocity() const {
                return body_lin_vel_;
            }

            inline double getBodyAngularVelocity() const {
                return body_ang_vel_;
            }
    
        private:
            static constexpr double wheel_radius_ = 0.1651;
            static constexpr double vehicle_track_width_ = 0.555;
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
            double body_lin_vel_;
            double body_ang_vel_;
    };

    typedef std::shared_ptr<JointStateMeasurement> JointStateMeasurementPtr;
} // end husky_inekf namespace