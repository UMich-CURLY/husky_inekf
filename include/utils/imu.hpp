#pragma once
#include "utils/measurement.h"
#include "sensor_msgs/Imu.h"
#include <stdint.h>
#include <string>

namespace husky_inekf_data {

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
                angular_velocity = {    
                    imu_msg.angular_velocity.x,
                    imu_msg.angular_velocity.y,
                    imu_msg.angular_velocity.z 
                };
                linear_acceleration = {
                    imu_msg.linear_acceleration.x,
                    imu_msg.linear_acceleration.y,
                    imu_msg.linear_acceleration.z
                };

                header = MeasurementHeader(imu_msg.header);

                type_ = IMU;
            }

        private:
            Eigen::Matrix3d R_;
    };
}