#pragma once

/* ROS specific interface */
#include "geometry_msgs/TwistStamped.h"
#include <unsupported/Eigen/MatrixFunctions>

#include "utils/measurement.h"
#include <stdint.h>
#include <vector>
#include <string>
#include <memory>

namespace husky_inekf {

    class VelocityMeasurement : public Measurement {
        public:
            // Construct Encoder measurement
            VelocityMeasurement() {
                type_ = VELOCITY;
            }

            VelocityMeasurement(const geometry_msgs::TwistStamped& vel_msg){
                type_ = VELOCITY;

                lin_vel_.resize(3, 1);
                ang_vel_.resize(3, 1);
                setLinearVelocity(vel_msg.twist.linear);
                setAngularVelocity(vel_msg.twist.angular);                
                setHeader(vel_msg.header);
            }

            void setLinearVelocity(const geometry_msgs::Vector3& lin_vel_in) {
                
                lin_vel_(0) = lin_vel_in.x;
                lin_vel_(1) = lin_vel_in.y;
                lin_vel_(2) = lin_vel_in.z;
                
            }


            void setLinearVelocity(const std::vector<double>& lin_vel_in) {
                for(int i=0; i<3; ++i){
                    lin_vel_(i) = lin_vel_in[i];
                }
            }

            
            void setAngularVelocity(const geometry_msgs::Vector3& ang_vel_in) {
                
                ang_vel_(0) = ang_vel_in.x;
                ang_vel_(1) = ang_vel_in.y;
                ang_vel_(2) = ang_vel_in.z;
                
            }

            void setAngularVelocity(const std::vector<double>& ang_vel_in) {
                for(int i=0; i<3; ++i){
                    ang_vel_(i) = ang_vel_in[i];
                }
            }

            

            inline const Eigen::VectorXd&  getLinearVelocity() const {
                return lin_vel_;
            }

            inline const Eigen::VectorXd&  getAngularVelocity() const {
                return ang_vel_;
            }
    
        private:

            
            Eigen::VectorXd lin_vel_;
            Eigen::VectorXd ang_vel_;
    };

    typedef std::shared_ptr<VelocityMeasurement> VelocityMeasurementPtr;
} // end husky_inekf namespace