#include "system/husky_state.hpp"

namespace husky_inekf{

// Default Constructor
HuskyState::HuskyState() {
    this->clear();
}

void HuskyState::setJointState(
    const std::shared_ptr<husky_inekf::JointStateMeasurement> next_joint_state) {

    const auto joint_state_data = next_joint_state;
    // Set wheel velocity and position in sensor frame
    q_.block<3, 1>(0,0) = joint_state_data.get()->getJointPosition();   // wheel position
    dq_.block<3,1>(0,0) = joint_state_data.get()->getJointVelocity();   // wheel velocity

    return;
}


// Set q and dq to 0
void HuskyState::clear() {
    q_ = Eigen::Matrix<double,10,1>::Zero();
    dq_ = Eigen::Matrix<double,10,1>::Zero();
    // GRF_ = Eigen::Matrix<double,4,1>::Zero();
    return; 
}


// Get q and dq
Eigen::Matrix<double,10,1>  HuskyState::q() const { return q_; }
Eigen::Matrix<double,10,1>  HuskyState::dq() const { return dq_; }

// Get base position
Eigen::Vector3d  HuskyState::getPosition() const { return q_.segment<3>(0); }

// Get base quaternion
Eigen::Quaternion<double>  HuskyState::getQuaternion() const { return Eigen::Quaternion<double>(this->getRotation()); }

// Get rotation matrix
Eigen::Matrix3d  HuskyState::getRotation() const { return Euler2Rotation(this->getEulerAngles()); }

// Extract Euler angles and rates
Eigen::Vector3d HuskyState::getEulerAngles() const { return q_.segment<3>(3); }
Eigen::Vector3d HuskyState::getEulerRates() const { return dq_.segment<3>(3); }

Eigen::Vector3d HuskyState::getAngularVelocity() const { 
    return EulerRates2AngularVelocity(this->getEulerAngles(), this->getEulerRates()); 
}

// Extract encoder positions
Eigen::Matrix<double, 4, 1> HuskyState::getEncoderPositions() const{
    return q_.segment<4>(6); //<! take 4 elements start from idx = 6
}

Eigen::Matrix<double,4,1> HuskyState::getEncoderVelocities() const {
    return dq_.segment<4>(6); //<! take 4 elements start from idx = 6
}


void HuskyState::setBaseRotation(const Eigen::Matrix3d& R) {
    q_.segment<3>(3) = Rotation2Euler(R);
}

void HuskyState::setBasePosition(const Eigen::Vector3d& p) {
    q_.segment<3>(0) = p;
}

void HuskyState::setBaseVelocity(const Eigen::Vector3d& v) {
    dq_.segment<3>(0) = v;
}

void HuskyState::setImuBias(const Eigen::VectorXd& bias){
    imu_bias_ = bias;
}

Eigen::Vector3d HuskyState::getBodyVelocity() const { 
    Eigen::Vector3d v_world = dq_.segment<3>(0);
    Eigen::Matrix3d Rwb = this->getRotation();
    return Rwb.transpose() * v_world;
}

Eigen::Vector3d HuskyState::getWorldVelocity() const{
    return dq_.segment<3>(0);
}

Eigen::VectorXd HuskyState::getImuBias() const{
    return imu_bias_;
}

// Extract each DOF position by name
double HuskyState::x() const { return q_(0); }
double HuskyState::y() const { return q_(1); }
double HuskyState::z() const { return q_(2); }
double HuskyState::yaw() const { return q_(3); }
double HuskyState::pitch() const { return q_(4); }
double HuskyState::roll() const { return q_(5); }
// Unused currently
double HuskyState::leftFrontMotor() const { return q_(6); }
double HuskyState::rightFrontMotor() const { return q_(7); }
double HuskyState::leftHindMotor() const { return q_(8); }
double HuskyState::rightHindMotor() const { return q_(9); }

// Extract each DOF velocity by name
double HuskyState::dx() const { return dq_(0); }
double HuskyState::dy() const { return dq_(1); }
double HuskyState::dz() const { return dq_(2); }
double HuskyState::dyaw() const { return dq_(3); }
double HuskyState::dpitch() const { return dq_(4); }
double HuskyState::droll() const { return dq_(5); }
// Unused currently
double HuskyState::dleftFrontMotor() const { return dq_(6); }
double HuskyState::drightFrontMotor() const { return dq_(7); }
double HuskyState::dleftHindMotor() const { return dq_(8); }
double HuskyState::drightHindMotor() const { return dq_(9); }


// Print out state information
std::ostream& operator<<(std::ostream& os, const  HuskyState& obj) {
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
    return os;
}

} // end namespace husky_inekf