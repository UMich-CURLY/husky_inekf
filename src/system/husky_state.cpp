#include "system/cheetah_state.hpp"


#include "kin/Jp_Body_to_FrontLeftFoot.h"
#include "kin/Jp_Body_to_FrontRightFoot.h"
#include "kin/Jp_Body_to_HindLeftFoot.h"
#include "kin/Jp_Body_to_HindRightFoot.h"
#include "kin/p_Body_to_FrontLeftFoot.h"
#include "kin/p_Body_to_FrontRightFoot.h"
#include "kin/p_Body_to_HindLeftFoot.h"
#include "kin/p_Body_to_HindRightFoot.h"

// Default Constructor
CheetahState::CheetahState() {
    this->clear();
}

// Constructor from cassie_out_t
CheetahState::CheetahState(const cheetah_lcm_packet_t& cheetah_data) {
    this->set(cheetah_data);
}

// Set q and dq from cheetah_lcm_data_t
void CheetahState::set(const cheetah_lcm_packet_t& cheetah_data) { 

    const std::shared_ptr<cheetah_inekf_lcm::ImuMeasurement<double>> imu_data = cheetah_data.imu;
    
    // Set orientation
    Eigen::Quaternion<double> quat(imu_data.get()->orientation.w, 
                            imu_data.get()->orientation.x,
                            imu_data.get()->orientation.y,
                            imu_data.get()->orientation.z); 
    Eigen::Vector3d euler = Rotation2Euler(quat.toRotationMatrix()); // Eigen's eulerAngles function caused discontinuities in signal  
    q_.block<3,1>(3,0) = euler;

    // Set orientation rates
    Eigen::Vector3d angularVelocity, eulerRates;
    angularVelocity << imu_data.get()->angular_velocity.x, imu_data.get()->angular_velocity.y, imu_data.get()->angular_velocity.z;
    eulerRates = AngularVelocity2EulerRates(euler, angularVelocity);
    dq_.block<3,1>(3,0) = eulerRates;

    const std::shared_ptr<cheetah_inekf_lcm::JointStateMeasurement> joint_state_data = cheetah_data.joint_state;
    // Set encoders position & rates:
    q_.block<12,1>(6,0) = joint_state_data.get()->joint_position;
    dq_.block<12,1>(6,0) = joint_state_data.get()->joint_velocity;

    const std::shared_ptr<cheetah_inekf_lcm::ContactsMeasurement> contact_data = cheetah_data.contact;
    right_front_contact_ = contact_data.get()->getContacts()[0];
    left_front_contact_  = contact_data.get()->getContacts()[1];
    right_hind_contact_  = contact_data.get()->getContacts()[2];
    left_hind_contact_   = contact_data.get()->getContacts()[3];

    return;
}


// Set q and dq to 0
void CheetahState::clear() {
    q_ = Eigen::Matrix<double,18,1>::Zero();
    dq_ = Eigen::Matrix<double,18,1>::Zero();
    // GRF_ = Eigen::Matrix<double,4,1>::Zero();
    right_front_contact_ = 0;
    left_front_contact_ = 0;
    right_hind_contact_ = 0;
    left_hind_contact_ = 0;
    return; 
}


// Get q and dq
Eigen::Matrix<double,18,1>  CheetahState::q() const { return q_; }
Eigen::Matrix<double,18,1>  CheetahState::dq() const { return dq_; }

// Get base position
Eigen::Vector3d  CheetahState::getPosition() const { return q_.segment<3>(0); }

// Get base quaternion
Eigen::Quaternion<double>  CheetahState::getQuaternion() const { return Eigen::Quaternion<double>(this->getRotation()); }

// Get rotation matrix
Eigen::Matrix3d  CheetahState::getRotation() const { return Euler2Rotation(this->getEulerAngles()); }

// Extract Euler angles and rates
Eigen::Vector3d CheetahState::getEulerAngles() const { return q_.segment<3>(3); }
Eigen::Vector3d CheetahState::getEulerRates() const { return dq_.segment<3>(3); }

Eigen::Vector3d CheetahState::getAngularVelocity() const { 
    return EulerRates2AngularVelocity(this->getEulerAngles(), this->getEulerRates()); 
}

// Extract encoder positions
Eigen::Matrix<double, 12, 1> CheetahState::getEncoderPositions() const{
    return q_.segment<12>(6); //<! take 12 elements start from idx = 6
}

Eigen::Matrix<double,12,1> CheetahState::getEncoderVelocities() const {
    return dq_.segment<12>(6); //<! take 12 elements start from idx = 6
}

Eigen::Vector3d CheetahState::getKinematicVelocity() const {
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d w = this->getAngularVelocity();
    Eigen::Matrix<double,12,1> e = this->getEncoderPositions();
    Eigen::Matrix<double,12,1> e_dot = this->getEncoderVelocities();
    ///TODO: figure out how to compute this later if needed
    
    if (right_front_contact_ == 1) {
        Eigen::Vector3d pRF = p_Body_to_FrontRightFoot(e); // {I}_p_{IRF}
        Eigen::Matrix<double,3,12> J_pRF = Jp_Body_to_FrontRightFoot(e);
        velocity = -J_pRF*e_dot - skew(w)*pRF; // {I}_v_{WI}
    } else if (left_front_contact_ == 1) {
        Eigen::Vector3d pLF = p_Body_to_FrontLeftFoot(e); // {I}_p_{ILF}
        Eigen::Matrix<double,3,12> J_pLF = Jp_Body_to_FrontLeftFoot(e);
        velocity = -J_pLF*e_dot - skew(w)*pLF; // {I}_v_{WI}
    } else if (right_hind_contact_ == 1) {
        Eigen::Vector3d pRH = p_Body_to_HindRightFoot(e); // {I}_p_{IRH}
        Eigen::Matrix<double,3,12> J_pRH = Jp_Body_to_HindRightFoot(e);
        velocity = -J_pRH*e_dot - skew(w)*pRH; // {I}_v_{WI}
    } else if (left_hind_contact_ == 1){
        Eigen::Vector3d pLH = p_Body_to_HindLeftFoot(e); // {I}_p_{ILH}
        Eigen::Matrix<double,3,12> J_pLH = Jp_Body_to_HindLeftFoot(e);
        velocity = -J_pLH*e_dot - skew(w)*pLH; // {I}_v_{WI}
    }
    return velocity;
}

bool CheetahState::getLeftFrontContact() const {
    return left_front_contact_;
}

bool CheetahState::getLeftHindContact() const {
    return left_hind_contact_;
}

bool CheetahState::getRightFrontContact() const {
    return right_front_contact_;
}

bool CheetahState::getRightHindContact() const {
    return right_hind_contact_;
}

void CheetahState::setBaseRotation(const Eigen::Matrix3d& R) {
    q_.segment<3>(3) = Rotation2Euler(R);
}

void CheetahState::setBasePosition(const Eigen::Vector3d& p) {
    q_.segment<3>(0) = p;
}

void CheetahState::setBaseVelocity(const Eigen::Vector3d& v) {
    dq_.segment<3>(0) = v;
}

Eigen::Vector3d CheetahState::getBodyVelocity() const { 
    Eigen::Vector3d v_world = dq_.segment<3>(0);
    Eigen::Matrix3d Rwb = this->getRotation();
    return Rwb.transpose() * v_world;
}

// Extract each DOF position by name
double CheetahState::x() const { return q_(0); }
double CheetahState::y() const { return q_(1); }
double CheetahState::z() const { return q_(2); }
double CheetahState::yaw() const { return q_(3); }
double CheetahState::pitch() const { return q_(4); }
double CheetahState::roll() const { return q_(5); }
double CheetahState::rightFrontMotor1() const { return q_(6); }
double CheetahState::rightFrontMotor2() const { return q_(7); }
double CheetahState::rightFrontMotor3() const { return q_(8); }
double CheetahState::leftFrontMotor1() const { return q_(9); }
double CheetahState::leftFrontMotor2() const { return q_(10); }
double CheetahState::leftFrontMotor3() const { return q_(11); }
double CheetahState::rightHindMotor1() const { return q_(12); }
double CheetahState::rightHindMotor2() const { return q_(13); }
double CheetahState::rightHindMotor3() const { return q_(14); }
double CheetahState::leftHindMotor1() const { return q_(15); }
double CheetahState::leftHindMotor2() const { return q_(16); }
double CheetahState::leftHindMotor3() const { return q_(17); }

// Extract each DOF velocity by name
double CheetahState::dx() const { return dq_(0); }
double CheetahState::dy() const { return dq_(1); }
double CheetahState::dz() const { return dq_(2); }
double CheetahState::dyaw() const { return dq_(3); }
double CheetahState::dpitch() const { return dq_(4); }
double CheetahState::droll() const { return dq_(5); }
double CheetahState::drightFrontMotor1() const { return dq_(6); }
double CheetahState::drightFrontMotor2() const { return dq_(7); }
double CheetahState::drightFrontMotor3() const { return dq_(8); }
double CheetahState::dleftFrontMotor1() const { return dq_(9); }
double CheetahState::dleftFrontMotor2() const { return dq_(10); }
double CheetahState::dleftFrontMotor3() const { return dq_(11); }
double CheetahState::drightHindMotor1() const { return dq_(12); }
double CheetahState::drightHindMotor2() const { return dq_(13); }
double CheetahState::drightHindMotor3() const { return dq_(14); }
double CheetahState::dleftHindMotor1() const { return dq_(15); }
double CheetahState::dleftHindMotor2() const { return dq_(16); }
double CheetahState::dleftHindMotor3() const { return dq_(17); }


// Print out state information
std::ostream& operator<<(std::ostream& os, const  CheetahState& obj) {
    os << "q: [";
    for (int i=0; i<obj.q_.rows()-1; ++i) {
        os << obj.q_(i) << ", ";
    } 
    os << obj.q_(obj.q_.rows()-1) << "]\n";

    os << "dq: [";
    for (int i=0; i<obj.dq_.rows()-1; ++i) {
        os << obj.dq_(i) << ", ";
    } 
    os << obj.dq_(obj.dq_.rows()-1) << "]";
}
