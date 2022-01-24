#pragma once

/* ROS specific interface */
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include "utils/measurement.h"
#include <stdint.h>
#include <vector>
#include <string>
#include <memory>
#include <Eigen/Geometry> 


namespace husky_inekf {

    class CameraOdomMeasurement : public Measurement {
        public:
            // Construct Encoder measurement
            CameraOdomMeasurement() {
                type_ = CAMERA_ODOM;
            }

            CameraOdomMeasurement(const nav_msgs::Odometry& camera_odom_msg){
                type_ = CAMERA_ODOM;

                lin_vel_.resize(3, 1);
                ang_vel_.resize(3, 1);
                position_.resize(3, 1);
                orientation_.resize(4, 1);
                setPosition(camera_odom_msg.pose.pose.position); // [x, y, z]
                setOrientaion(camera_odom_msg.pose.pose.orientation); // [x, y, z]
          
                setHeader(camera_odom_msg.header);
            }


            void setPosition(const geometry_msgs::Point& position_in) {
                
                position_(0) = position_in.x;
                position_(1) = position_in.y;
                position_(2) = position_in.z;
                
            }

            void setOrientation(const geometry_msgs::Quaternion& orientation_in) {
                Eigen::Quaternion<float64> orientation_quat(orientation_in.w,
                                                            orientation_in.x,
                                                            orientation_in.y,
                                                            orientation_in.z)
                auto euler = orientation_quat.toRotationMatrix().eulerAngles(0, 1, 2);
                orientation_(0) = orientation_in(0);
                orientation_(1) = orientation_in(1);
                orientation_(2) = orientation_in(2);
            }

            inline const Eigen::VectorXd&  getOrientation() const {
                return position_;
            }

            inline const Eigen::VectorXd&  getOrientation() const {
                return orientation_;
            }
    
        private:

            Eigen::VectorXd position_;
            Eigen::VectorXd orientation_;
    };

    typedef std::shared_ptr<CameraOdomMeasurement> CameraOdomMeasurementPtr;
} // end husky_inekf namespace