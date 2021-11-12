#pragma once
#ifndef CHEETAHSTATE_H
#define CHEETAHSTATE_H

//C Libraries
#include <stdint.h>

#include <iostream>
#include <memory>

//External Libraries
#include <Eigen/Dense>
#include "ros/ros.h"

//Husky Libraries
#include "utils/imu.hpp"
#include "utils/joint_state.hpp"
#include "utils/utils.hpp"
// #include "RosPublisher.h"
// #include "PassiveTimeSync.h"

namespace husky_inekf{

class HuskyState {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        HuskyState();

        template <typename T>
        void setImu(
            const std::shared_ptr<husky_inekf::ImuMeasurement<T>>& next_imu) {

            const auto imu_data = next_imu;
            
            Eigen::Vector3d euler = Rotation2Euler(this->getRotation());
            // Set orientation rates
            Eigen::Vector3d angularVelocity, eulerRates;
            angularVelocity <<  imu_data.get()->angular_velocity.x, 
                                imu_data.get()->angular_velocity.y, 
                                imu_data.get()->angular_velocity.z;
            eulerRates = AngularVelocity2EulerRates(euler, angularVelocity);
            dq_.block<3,1>(3,0) = eulerRates;

            return;
        }

        void setJointState(
            const std::shared_ptr<husky_inekf::JointStateMeasurement> 
            next_joint_state);
        void setBaseRotation(const Eigen::Matrix3d& R);
        void setBasePosition(const Eigen::Vector3d& p);
        void setBaseVelocity(const Eigen::Vector3d& v);
        void clear();

        Eigen::Matrix<double,10,1> q() const;
        Eigen::Matrix<double,10,1> dq() const;
        Eigen::Vector3d getPosition() const;
        Eigen::Quaternion<double> getQuaternion() const;
        Eigen::Matrix3d getRotation() const;
        Eigen::Vector3d getEulerAngles() const;
        Eigen::Vector3d getEulerRates() const;
        Eigen::Matrix<double, 4, 1> getEncoderPositions() const;
        Eigen::Matrix<double, 4, 1> getEncoderVelocities() const;
//         Eigen::Matrix<double,10,1> getMotorPositions() const;
//         Eigen::Matrix<double,10,1> getMotorVelocities() const;
//         Eigen::Matrix<double,4,1> getGRF() const;
        bool getLeftFrontContact() const;
        bool getLeftHindContact() const;
        bool getRightFrontContact() const;
        bool getRightHindContact() const;
        Eigen::Vector3d getAngularVelocity() const;
        Eigen::Vector3d getBodyVelocity() const;
        
        // Extract robot pose:
        double x() const;
        double y() const;
        double z() const;
        double yaw() const;
        double pitch() const;
        double roll() const;
        // Extract robot joint values:
        double rightFrontMotor() const;
        double leftFrontMotor() const;
        double rightHindMotor() const;
        double leftHindMotor() const;

        // Extract robot velocity:
        double dx() const;
        double dy() const;
        double dz() const;
        double dyaw() const;
        double dpitch() const;
        double droll() const;
        // Extract robot joint velocities:
        double drightFrontMotor() const;
        double dleftFrontMotor() const;
        double drightHindMotor() const;
        double dleftHindMotor() const;
        
        void setTime(double time_in){time_stamp_ = time_in;};
        double getTime() const{return time_stamp_;};

        friend std::ostream& operator<<(std::ostream& os, const  HuskyState& obj);  

    private:
        double time_stamp_;
        Eigen::Matrix<double, 10,1> q_;
        Eigen::Matrix<double, 10,1> dq_;
        Eigen::Matrix<double,4,1> GRF_; //!< ground reaction force
};

} // end namespace husky_inekf
#endif
