#include "estimator/body_estimator.hpp"

namespace husky_inekf{

BodyEstimator::BodyEstimator() :
    t_prev_(0), imu_prev_(Eigen::Matrix<double,6,1>::Zero()) {

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
    
    filter_.setNoiseParams(params);
    std::cout << "Noise parameters are initialized to: \n";
    std::cout << filter_.getNoiseParams() << std::endl;
}


bool BodyEstimator::biasInitialized() { return bias_initialized_; }
bool BodyEstimator::enabled() { return enabled_; }
void BodyEstimator::enableFilter() { enabled_ = true; }

void BodyEstimator::propagateIMU(const ImuMeasurement<double>& imu_packet_in, HuskyState& state) {
    // Initialize bias from initial robot condition
    if (!bias_initialized_) {
        initBias(imu_packet_in);
    }

    // Extract out current IMU data [w;a]
    Eigen::Matrix<double,6,1> imu;
    imu << imu_packet_in.angular_velocity.x,
           imu_packet_in.angular_velocity.y, 
           imu_packet_in.angular_velocity.z,
           imu_packet_in.linear_acceleration.x, 
           imu_packet_in.linear_acceleration.y , 
           imu_packet_in.linear_acceleration.z;
    double t = imu_packet_in.getTime();

    // Propagate state based on IMU and contact data
    double dt = t - t_prev_;
    if(estimator_debug_enabled_){
        ROS_INFO("Tprev %0.6lf T %0.6lf dt %0.6lf \n", t_prev_, t, dt);
    }
    
    if (dt > 0)
        filter_.Propagate(imu_prev_, dt); 

    // correctKinematics(state);

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
    state.setTime(t);

    // Store previous imu data
    t_prev_ = t;
    imu_prev_ = imu;
    seq_ = imu_packet_in.header.seq;

    if (estimator_debug_enabled_) {
        ROS_INFO("IMU Propagation Complete: linacceleation x: %0.6f y: %.06f z: %0.6f \n", 
            imu_packet_in.linear_acceleration.x,
            imu_packet_in.linear_acceleration.y,
            imu_packet_in.linear_acceleration.z);
    }
}


// correctvelocity 
void BodyEstimator::correctVelocity(const JointStateMeasurement& joint_state_packet_in, HuskyState& state){

    Eigen::Vector3d measured_velocity = joint_state_packet_in.getBodyLinearVelocity();
    filter_.CorrectVelocity(measured_velocity, velocity_cov_);
}


// Publish current pose over lcm & and save to ros
// void BodyEstimator::publishPose(double time, std::string map_frame_id, uint32_t seq) {
//     // cheetah_inekf_lcm::pose_t pose;
//     pose.seq = seq;
//     pose.stamp = time;
//     pose.frame_id = map_frame_id;

//     // Get inekf pose estimate
//     inekf::RobotState estimate = filter_.getState();
//     Eigen::Vector3d p = estimate.getPosition();

//     // Publish pose in LCM
//     // std::cout << "Issue before read " << std::endl;
//     pose.body[0] = p(0); pose.body[1] = p(1); pose.body[2] = p(2);
//     // lcm_->publish(LCM_POSE_CHANNEL, &pose);
// }

void BodyEstimator::initBias(const ImuMeasurement<double>& imu_packet_in) {
    if (!static_bias_initialization_) {
        bias_initialized_ = true;
        return;
    }
    // Initialize bias based on imu orientation and static assumption
    if (bias_init_vec_.size() < 250) {
        Eigen::Vector3d w, a;
        w << imu_packet_in.angular_velocity.x, 
             imu_packet_in.angular_velocity.y, 
             imu_packet_in.angular_velocity.z;
        a << imu_packet_in.linear_acceleration.x,
             imu_packet_in.linear_acceleration.y,
             imu_packet_in.linear_acceleration.z;
        Eigen::Quaternion<double> quat(imu_packet_in.orientation.w, 
                                       imu_packet_in.orientation.x,
                                       imu_packet_in.orientation.y,
                                       imu_packet_in.orientation.z); 
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

void BodyEstimator::initState(const ImuMeasurement<double>& imu_packet_in, 
                        const JointStateMeasurement& joint_state_packet_in, HuskyState& state) {
    // Clear filter
    filter_.clear();

    // Initialize state mean
    Eigen::Quaternion<double> quat(imu_packet_in.orientation.w, 
                                   imu_packet_in.orientation.x,
                                   imu_packet_in.orientation.y,
                                   imu_packet_in.orientation.z); 
    Eigen::Matrix3d R0 = quat.toRotationMatrix(); // Initialize based on VectorNav estimate


    Eigen::Vector3d v0_body = joint_state_packet_in.getBodyLinearVelocity();
    Eigen::Vector3d v0 = R0*v0_body; // initial velocity

    // Eigen::Vector3d v0 = {0.0,0.0,0.0};
    Eigen::Vector3d p0 = {0.0, 0.0, 0.0}; // initial position, we set imu frame as world frame

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
    t_prev_ = imu_packet_in.getTime();
    imu_prev_ << imu_packet_in.angular_velocity.x, 
                imu_packet_in.angular_velocity.y, 
                imu_packet_in.angular_velocity.z;
                imu_packet_in.linear_acceleration.x,
                imu_packet_in.linear_acceleration.y,
                imu_packet_in.linear_acceleration.z;;
    enabled_ = true;
}

} // end husky_inekf namespace