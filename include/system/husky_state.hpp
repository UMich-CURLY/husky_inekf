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

//Cheetah Libraries
#include "utils/cheetah_data_t.hpp"
#include "utils/utils.hpp"
// #include "RosPublisher.h"
// #include "PassiveTimeSync.h"

class CheetahState {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        CheetahState();
//          CheetahState(const Eigen::Matrix<double,18,1>& q, const Eigen::Matrix<double,18,1>& dq, bool computeContacts = true);
        CheetahState(const cheetah_lcm_packet_t& cheetah_data);

//         void set(const Eigen::Matrix<double,18,1>& q, const Eigen::Matrix<double,18,1>& dq, bool computeContacts = true);
        void set(const cheetah_lcm_packet_t& cheetah_data);
        void setBaseRotation(const Eigen::Matrix3d& R);
        void setBasePosition(const Eigen::Vector3d& p);
        void setBaseVelocity(const Eigen::Vector3d& v);
//         void setMotorPositions(const Eigen::Matrix<double,10,1>& qM);
//         void setMotorVelocities(const Eigen::Matrix<double,10,1>& dqM);
        void clear();

        Eigen::Matrix<double,18,1> q() const;
        Eigen::Matrix<double,18,1> dq() const;
        Eigen::Vector3d getPosition() const;
        Eigen::Quaternion<double> getQuaternion() const;
        Eigen::Matrix3d getRotation() const;
        Eigen::Vector3d getEulerAngles() const;
        Eigen::Vector3d getEulerRates() const;
        Eigen::Matrix<double, 12, 1> getEncoderPositions() const;
        Eigen::Matrix<double,12,1> getEncoderVelocities() const;
//         Eigen::Matrix<double,10,1> getMotorPositions() const;
//         Eigen::Matrix<double,10,1> getMotorVelocities() const;
//         Eigen::Matrix<double,4,1> getGRF() const;
        bool getLeftFrontContact() const;
        bool getLeftHindContact() const;
        bool getRightFrontContact() const;
        bool getRightHindContact() const;
        Eigen::Vector3d getAngularVelocity() const;
        Eigen::Vector3d getKinematicVelocity() const;
        Eigen::Vector3d getBodyVelocity() const;
        
        // Extract robot pose:
        double x() const;
        double y() const;
        double z() const;
        double yaw() const;
        double pitch() const;
        double roll() const;
        // Extract robot joint values:
        double rightFrontMotor1() const;
        double rightFrontMotor2() const;
        double rightFrontMotor3() const;
        double leftFrontMotor1() const;
        double leftFrontMotor2() const;
        double leftFrontMotor3() const;
        double rightHindMotor1() const;
        double rightHindMotor2() const;
        double rightHindMotor3() const;
        double leftHindMotor1() const;
        double leftHindMotor2() const;
        double leftHindMotor3() const;

        // Extract robot velocity:
        double dx() const;
        double dy() const;
        double dz() const;
        double dyaw() const;
        double dpitch() const;
        double droll() const;
        // Extract robot joint velocities:
        double drightFrontMotor1() const;
        double drightFrontMotor2() const;
        double drightFrontMotor3() const;
        double dleftFrontMotor1() const;
        double dleftFrontMotor2() const;
        double dleftFrontMotor3() const;
        double drightHindMotor1() const;
        double drightHindMotor2() const;
        double drightHindMotor3() const;
        double dleftHindMotor1() const;
        double dleftHindMotor2() const;
        double dleftHindMotor3() const;

        friend std::ostream& operator<<(std::ostream& os, const  CheetahState& obj);  

    private:
        Eigen::Matrix<double, 18,1> q_;
        Eigen::Matrix<double, 18,1> dq_;
        Eigen::Matrix<double,4,1> GRF_; //!< ground reaction force
        bool left_front_contact_;
        bool left_hind_contact_;
        bool right_front_contact_;
        bool right_hind_contact_;
};

#endif
