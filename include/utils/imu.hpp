#pragma once
#include "utils/measurement.h"
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

        private:
            Eigen::Matrix3d R_;
    };
}