#pragma once
#include "utils/measurement.h"
#include "sensor_msgs/Imu.h"
#include <stdint.h>
#include <string>
#include <memory>

namespace husky_inekf {

    template <typename T>
    struct ImuOrientation {
        T w, x, y, z;
    };

    template <typename T>
    struct ImuAngularVelocity {
        T x, y, z;
    };

    template <typename T>
    struct ImuLinearAcceleration {
        T x, y, z;
    };

    template <typename T>
    class ImuMeasurement : public Measurement {
        public:
            ImuOrientation<T> orientation;
            ImuAngularVelocity<T> angular_velocity;
            ImuLinearAcceleration<T> linear_acceleration;

            Eigen::Matrix3d getRotation() { return R_; }
            
            void setRotation() {
                Eigen::Quaternion<double> q(orientation.w,orientation.x,orientation.y,orientation.z);
                R_ = q.toRotationMatrix();
            }

            // Construct IMU measurement
            ImuMeasurement() {
                type_ = IMU;
            }

            // Overloaded constructor for construction using ros imu topic
            ImuMeasurement(const sensor_msgs::Imu& imu_msg) {
                orientation = { 
                    imu_msg.orientation.w, 
                    imu_msg.orientation.x, 
                    imu_msg.orientation.y, 
                    imu_msg.orientation.z 
                };
                // angular_velocity = {    
                //     imu_msg.angular_velocity.x,
                //     imu_msg.angular_velocity.y,
                //     imu_msg.angular_velocity.z 
                // };
                // linear_acceleration = {
                //     imu_msg.linear_acceleration.x,
                //     imu_msg.linear_acceleration.y,
                //     imu_msg.linear_acceleration.z
                // };

                // angular_velocity = {    
                //     -imu_msg.angular_velocity.y,
                //     -imu_msg.angular_velocity.x,
                //     -imu_msg.angular_velocity.z 
                // };
                // linear_acceleration = {
                //     -imu_msg.linear_acceleration.y,
                //     -imu_msg.linear_acceleration.x,
                //     -imu_msg.linear_acceleration.z
                // };


                // default is (0, 0.7071, -0.7071, 0)
                Eigen::Quaternion<double> rotation_body_imu(0, 0.7071, -0.7071, 0);
                Eigen::Matrix3d rotation_body_imu_matrix = rotation_body_imu.toRotationMatrix();

                angular_velocity = {
                    rotation_body_imu_matrix.transpose()*imu_msg.angular_velocity[0],
                    rotation_body_imu_matrix.transpose()*imu_msg.angular_velocity[1],
                    rotation_body_imu_matrix.transpose()*imu_msg.angular_velocity[2],
                }

                linear_acceleration = {
                    rotation_body_imu_matrix.transpose()*imu_msg.linear_acceleration[0],
                    rotation_body_imu_matrix.transpose()*imu_msg.linear_acceleration[1],
                    rotation_body_imu_matrix.transpose()*imu_msg.linear_acceleration[2],
                };

                setHeader(imu_msg.header);

                type_ = IMU;
            }


        private:
            Eigen::Matrix3d R_;
    };

    typedef std::shared_ptr<ImuMeasurement<double>> ImuMeasurementPtr;
} // end husky_inekf namespace