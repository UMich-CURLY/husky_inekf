#pragma once

/* ROS specific interface */
#include "geometry_msgs/TwistStamped.h"
#include <unsupported/Eigen/MatrixFunctions>
#include "nav_msgs/Odometry.h"
#include "utils/measurement.h"
#include <stdint.h>
#include <vector>
#include <string>
#include <memory>
#include <Eigen/Geometry> 
#include <Eigen/Dense>


namespace husky_inekf {

    class CameraOdomMeasurement : public Measurement {
        public:
            // Construct Encoder measurement
            CameraOdomMeasurement() {
                type_ = CAMERA_ODOM;           
                translation_ = Eigen::Vector3d::Zero();
                rotation_ = Eigen::Matrix3d::Identity();
                transformation_ = Eigen::Matrix4d::Identity();
            }

            CameraOdomMeasurement(const nav_msgs::Odometry& camera_odom_msg){
                type_ = CAMERA_ODOM;
                

                translation_ = Eigen::Vector3d::Zero();

                rotation_ = Eigen::Matrix3d::Identity();
                
                transformation_ = Eigen::Matrix4d::Identity();              
                
                setTranslation(camera_odom_msg.pose.pose.position); // [x, y, z]
                setRotation(camera_odom_msg.pose.pose.orientation); // [x, y, z]
                setHeader(camera_odom_msg.header);

                setTransformation();
            }


            void setTranslation(const geometry_msgs::Point& position_in) {
                
                translation_(0) = position_in.x;
                translation_(1) = position_in.y;
                translation_(2) = position_in.z;
                
            }

            void setRotation(const geometry_msgs::Quaternion& orientation_in) {
                Eigen::Quaternion<double> orientation_quat(orientation_in.w,
                                                            orientation_in.x,
                                                            orientation_in.y,
                                                            orientation_in.z);
                rotation_ = orientation_quat.toRotationMatrix();
            }

            void setTransformation() {
                transformation_.block<3,3>(0,0) = rotation_;
                transformation_.block<3,1>(0,3) = translation_;
            }

            inline const Eigen::Matrix4d&  getTransformation() const {

                return transformation_;
            }

            
    
        private:

            Eigen::Vector3d translation_;
            Eigen::Matrix3d rotation_;
            Eigen::Matrix4d transformation_;
    };

    typedef std::shared_ptr<CameraOdomMeasurement> CameraOdomMeasurementPtr;
} // end husky_inekf namespace