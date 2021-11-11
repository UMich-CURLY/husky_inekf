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
#include "system/husky_state.hpp"
#include "utils/utils.hpp"
// #include "RosPublisher.h"
// #include "PassiveTimeSync.h"

namespace husky_inekf{

class HuskyState {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        HuskyState();
//          HuskyState(const Eigen::Matrix<double,18,1>& q, const Eigen::Matrix<double,18,1>& dq, bool computeContacts = true);
        // HuskyState(const cheetah_lcm_packet_t& cheetah_data);

//         void set(const Eigen::Matrix<double,18,1>& q, const Eigen::Matrix<double,18,1>& dq, bool computeContacts = true);
        // void set(const cheetah_lcm_packet_t& cheetah_data);
        void setBaseRotation(const Eigen::Matrix3d& R);
        void setBasePosition(const Eigen::Vector3d& p);
        void setBaseVelocity(const Eigen::Vector3d& v);
//         void setMotorPositions(const Eigen::Matrix<double,10,1>& qM);
//         void setMotorVelocities(const Eigen::Matrix<double,10,1>& dqM);
        void clear();

        Eigen::Matrix<double,10,1> q() const;
        Eigen::Matrix<double,10,1> dq() const;
        Eigen::Vector3d getPosition() const;
        Eigen::Quaternion<double> getQuaternion() const;
        Eigen::Matrix3d getRotation() const;
        Eigen::Vector3d getEulerAngles() const;
        Eigen::Vector3d getEulerRates() const;
        Eigen::Matrix<double, 4, 1> getEncoderPositions() const;
        Eigen::Matrix<double, 4,1> getEncoderVelocities() const;
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

        friend std::ostream& operator<<(std::ostream& os, const  HuskyState& obj);  

    private:
        Eigen::Matrix<double, 10,1> q_;
        Eigen::Matrix<double, 10,1> dq_;
        Eigen::Matrix<double,4,1> GRF_; //!< ground reaction force
};

} // end namespace husky_inekf
#endif
