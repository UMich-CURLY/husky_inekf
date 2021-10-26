#include "estimator/body_estimator.hpp"

#include "kin/H_Body_to_FrontLeftFoot.h"
#include "kin/H_Body_to_FrontRightFoot.h"
#include "kin/H_Body_to_HindLeftFoot.h"
#include "kin/H_Body_to_HindRightFoot.h"
#include "kin/Jp_Body_to_FrontLeftFoot.h"
#include "kin/Jp_Body_to_FrontRightFoot.h"
#include "kin/Jp_Body_to_HindLeftFoot.h"
#include "kin/Jp_Body_to_HindRightFoot.h"

BodyEstimator::BodyEstimator(lcm::LCM* lcm) :
    lcm_(lcm), t_prev_(0), imu_prev_(Eigen::Matrix<double,6,1>::Zero()) {

    // Create private node handle
    ros::NodeHandle nh("~");
    // Set debug output
    nh.param<bool>("/settings/estimator_enable_debug", estimator_debug_enabled_, false);
    // Enable visualization publisher if requested
    nh.param<std::string>("/settings/estimator_lcm_pose_channel", LCM_POSE_CHANNEL, "CHEETAH_POSE_CHANNEL");
    nh.param<bool>("/settings/estimator_publish_visualization_markers", 
        lcm_publish_visualization_markers_, false);
    // Settings
    nh.param<bool>("/settings/estimator_static_bias_initialization", static_bias_initialization_, false);

    inekf::NoiseParams params;
    double std;
    if (nh.getParam("/noise/gyroscope_std", std)) { 
        params.setGyroscopeNoise(std);
    }
    if (nh.getParam("/noise/accelerometer_std", std)) { 
        params.setAccelerometerNoise(std);
    }
    if (nh.getParam("/noise/gyroscope_bias_std", std)) { 
        params.setGyroscopeBiasNoise(std);
    }
    if (nh.getParam("/noise/accelerometer_bias_std", std)) { 
        params.setAccelerometerBiasNoise(std);
    }
    if (nh.getParam("/noise/contact_std", std)) { 
        params.setContactNoise(std);
    }
    filter_.setNoiseParams(params);
    std::cout << "Noise parameters are initialized to: \n";
    std::cout << filter_.getNoiseParams() << std::endl;
}


bool BodyEstimator::biasInitialized() { return bias_initialized_; }
bool BodyEstimator::enabled() { return enabled_; }
void BodyEstimator::enableFilter() { enabled_ = true; }

void BodyEstimator::update(cheetah_lcm_packet_t& cheetah_data, CheetahState& state) {
    // Initialize bias from initial robot condition
    if (!bias_initialized_) {
        initBias(cheetah_data);
    }

    // Extract out current IMU data [w;a]
    Eigen::Matrix<double,6,1> imu;
    imu << cheetah_data.imu.get()->angular_velocity.x,
           cheetah_data.imu.get()->angular_velocity.y, 
           cheetah_data.imu.get()->angular_velocity.z,
           cheetah_data.imu.get()->linear_acceleration.x, 
           cheetah_data.imu.get()->linear_acceleration.y , 
           cheetah_data.imu.get()->linear_acceleration.z;
    double t = cheetah_data.getTime();

    // Propagate state based on IMU and contact data
    double dt = t - t_prev_;
    if(estimator_debug_enabled_){
        ROS_INFO("Tprev %0.6lf T %0.6lf dt %0.6lf \n", t_prev_, t, dt);
    }
    
    if (dt > 0)
        filter_.Propagate(imu_prev_, dt); 

    correctKinematics(state);

    ///TODO: Check if imu strapdown model is correct
    inekf::RobotState estimate = filter_.getState();
    Eigen::Vector3d i_p_ib; i_p_ib << 0, 0, 0;
    Eigen::Vector3d w = imu.head<3>() - estimate.getGyroscopeBias(); // Angular velocity without bias
    Eigen::Matrix3d R = estimate.getRotation(); // no extra rotation needed
    Eigen::Vector3d p = estimate.getPosition() + R*i_p_ib;
    Eigen::Vector3d v = estimate.getVelocity() + R*inekf::skew(w)*i_p_ib;
    state.setBaseRotation(R);
    state.setBasePosition(p);
    state.setBaseVelocity(v); 

    // Store previous imu data
    t_prev_ = t;
    imu_prev_ = imu;
    seq_ = cheetah_data.imu.get()->header.seq;

    if (estimator_debug_enabled_) {
        ROS_INFO("IMU Propagation Complete: linacceleation x: %0.6f y: %.06f z: %0.6f \n", 
            cheetah_data.imu.get()->linear_acceleration.x,
            cheetah_data.imu.get()->linear_acceleration.y,
            cheetah_data.imu.get()->linear_acceleration.z);
    }
}

// Assumes state contacts have been updated
void BodyEstimator::setContacts(CheetahState& state) {
    // Update contact information
    const uint8_t CONTACT_THRESHOLD = 1;
    std::vector<std::pair<int,bool>> contacts;
    contacts.push_back(std::pair<int,bool> (0, state.getRightFrontContact())); 
    contacts.push_back(std::pair<int,bool> (1, state.getLeftFrontContact())); 
    contacts.push_back(std::pair<int,bool> (2, state.getRightHindContact())); 
    contacts.push_back(std::pair<int,bool> (3, state.getLeftHindContact())); 

    filter_.setContacts(contacts); // Set new contact states
}

// Assumes state encoders have been updated
void BodyEstimator::correctKinematics(CheetahState& state) {
    // Correct state based on kinematics measurements (probably in cheetah inekf ros)
    Eigen::Matrix<double,12,1> encoders = state.getEncoderPositions();

    Eigen::Matrix4d H_FL = H_Body_to_FrontLeftFoot(encoders); 
    Eigen::Matrix4d H_FR = H_Body_to_FrontRightFoot(encoders);
    Eigen::Matrix4d H_HL = H_Body_to_HindLeftFoot(encoders); 
    Eigen::Matrix4d H_HR = H_Body_to_HindRightFoot(encoders);
    Eigen::Matrix<double,3,12> JpFL = Jp_Body_to_FrontLeftFoot(encoders);
    Eigen::Matrix<double,3,12> JpFR = Jp_Body_to_FrontRightFoot(encoders);
    Eigen::Matrix<double,3,12> JpHL = Jp_Body_to_HindLeftFoot(encoders);
    Eigen::Matrix<double,3,12> JpHR = Jp_Body_to_HindRightFoot(encoders);
    Eigen::Matrix<double,6,6> covFL = Eigen::Matrix<double,6,6>::Identity();
    Eigen::Matrix<double,6,6> covFR = Eigen::Matrix<double,6,6>::Identity();
    Eigen::Matrix<double,6,6> covHL = Eigen::Matrix<double,6,6>::Identity();
    Eigen::Matrix<double,6,6> covHR = Eigen::Matrix<double,6,6>::Identity();
    covFL.block<3,3>(3,3) = JpFL*encoder_cov_*JpFL.transpose() + prior_kinematics_cov_;
    covFR.block<3,3>(3,3) = JpFR*encoder_cov_*JpFR.transpose() + prior_kinematics_cov_;
    covHL.block<3,3>(3,3) = JpHL*encoder_cov_*JpHL.transpose() + prior_kinematics_cov_;
    covHR.block<3,3>(3,3) = JpHR*encoder_cov_*JpHR.transpose() + prior_kinematics_cov_;
    inekf::Kinematics rightFrontFoot(0, H_FR, covFR);
    inekf::Kinematics leftFrontFoot(1, H_FL, covFL);
    inekf::Kinematics rightHindFoot(2, H_HR, covHR);
    inekf::Kinematics leftHindFoot(3, H_HL, covHL);
    inekf::vectorKinematics kinematics;
    kinematics.push_back(rightFrontFoot);
    kinematics.push_back(leftFrontFoot);
    kinematics.push_back(rightHindFoot);
    kinematics.push_back(leftHindFoot);  

    filter_.CorrectKinematics(kinematics);
    if (estimator_debug_enabled_) {
        auto position = filter_.getState().getPosition();
        ROS_INFO("Kinematics correction complete x: %0.6f y: %0.6f z: %0.6f\n", 
                position[0],
                position[1], 
                position[2]);
    }
    if (lcm_publish_visualization_markers_) {
        publishPose(t_prev_, "/cheetah/imu", seq_);
    }
}

// Publish current pose over lcm & and save to ros
void BodyEstimator::publishPose(double time, std::string map_frame_id, uint32_t seq) {
    cheetah_inekf_lcm::pose_t pose;
    pose.seq = seq;
    pose.stamp = time;
    pose.frame_id = map_frame_id;

    // Get inekf pose estimate
    inekf::RobotState estimate = filter_.getState();
    Eigen::Vector3d p = estimate.getPosition();

    // Publish pose in LCM
    // std::cout << "Issue before read " << std::endl;
    pose.body[0] = p(0); pose.body[1] = p(1); pose.body[2] = p(2);
    lcm_->publish(LCM_POSE_CHANNEL, &pose);
}

void BodyEstimator::initBias(cheetah_lcm_packet_t& cheetah_data) {
    if (!static_bias_initialization_) {
        bias_initialized_ = true;
        return;
    }
    // Initialize bias based on imu orientation and static assumption
    if (bias_init_vec_.size() < 250) {
        Eigen::Vector3d w, a;
        w << cheetah_data.imu.get()->angular_velocity.x, 
             cheetah_data.imu.get()->angular_velocity.y, 
             cheetah_data.imu.get()->angular_velocity.z;
        a << cheetah_data.imu.get()->linear_acceleration.x,
             cheetah_data.imu.get()->linear_acceleration.y,
             cheetah_data.imu.get()->linear_acceleration.z;
        Eigen::Quaternion<double> quat(cheetah_data.imu.get()->orientation.w, 
                                       cheetah_data.imu.get()->orientation.x,
                                       cheetah_data.imu.get()->orientation.y,
                                       cheetah_data.imu.get()->orientation.z); 
        Eigen::Matrix3d R = quat.toRotationMatrix();
        Eigen::Vector3d g; g << 0,0,-9.81;
        a = (R.transpose()*(R*a + g)).eval();
        Eigen::Matrix<double,6,1> v; 
        v << w(0),w(1),w(2),a(0),a(1),a(2);
        bias_init_vec_.push_back(v); // Store imu data with gravity removed
    } else {
        // Compute average bias of stored data
        Eigen::Matrix<double,6,1> avg = Eigen::Matrix<double,6,1>::Zero();
        for (int i=0; i<bias_init_vec_.size(); ++i) {
            avg = (avg + bias_init_vec_[i]).eval();
        }
        avg = (avg/bias_init_vec_.size()).eval();
        std::cout << "IMU bias initialized to: " << avg.transpose() << std::endl;
        bg0_ = avg.head<3>();
        ba0_ = avg.tail<3>();
        bias_initialized_ = true;
    }
}

void BodyEstimator::initState(const double t, const cheetah_lcm_packet_t& cheetah_data, const CheetahState& state) {
    // Clear filter
    filter_.clear();

    // Initialize state mean
    Eigen::Quaternion<double> quat(cheetah_data.imu.get()->orientation.w, 
                                   cheetah_data.imu.get()->orientation.x,
                                   cheetah_data.imu.get()->orientation.y,
                                   cheetah_data.imu.get()->orientation.z); 
    Eigen::Matrix3d R0 = quat.toRotationMatrix(); // Initialize based on VectorNav estimate
    Eigen::Vector3d v0 = R0*state.getKinematicVelocity(); // initial velocity
    // Eigen::Vector3d v0 = {0.0,0.0,0.0};
    Eigen::Vector3d p0 = {0.0, 0.0, 0.0}; // initial position, we set imu frame as world frame
    // .

    R0 = Eigen::Matrix3d::Identity();
    inekf::RobotState initial_state; 
    initial_state.setRotation(R0);
    initial_state.setVelocity(v0);
    initial_state.setPosition(p0);
    initial_state.setGyroscopeBias(bg0_);
    initial_state.setAccelerometerBias(ba0_);

    // Initialize state covariance
    initial_state.setRotationCovariance(0.03*Eigen::Matrix3d::Identity());
    initial_state.setVelocityCovariance(0.01*Eigen::Matrix3d::Identity());
    initial_state.setPositionCovariance(0.00001*Eigen::Matrix3d::Identity());
    initial_state.setGyroscopeBiasCovariance(0.0001*Eigen::Matrix3d::Identity());
    initial_state.setAccelerometerBiasCovariance(0.0025*Eigen::Matrix3d::Identity());

    filter_.setState(initial_state);
    std::cout << "Robot's state mean is initialized to: \n";
    std::cout << filter_.getState() << std::endl;
    std::cout << "Robot's state covariance is initialized to: \n";
    std::cout << filter_.getState().getP() << std::endl;

    // Set enabled flag
    t_prev_ = t;
    imu_prev_ << cheetah_data.imu.get()->angular_velocity.x, 
                cheetah_data.imu.get()->angular_velocity.y, 
                cheetah_data.imu.get()->angular_velocity.z;
                cheetah_data.imu.get()->linear_acceleration.x,
                cheetah_data.imu.get()->linear_acceleration.y,
                cheetah_data.imu.get()->linear_acceleration.z;;
    enabled_ = true;
}