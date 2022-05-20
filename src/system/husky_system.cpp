#include "system/husky_system.hpp"
// Thread safe locking
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/timer/timer.hpp>

#include <vector>
#include <numeric>

HuskySystem::HuskySystem(ros::NodeHandle* nh, husky_inekf::husky_data_t* husky_data_buffer): 
    nh_(nh), husky_data_buffer_(husky_data_buffer), pose_publisher_node_(nh), new_pose_ready_(false) {

    // Initialize inekf pose file printouts
    nh_->param<std::string>("/settings/system_inekf_pose_filename", file_name_, 
        "husky_inekf_pose.txt");
    nh_->param<std::string>("/settings/system_inekf_vel_est_file_name", vel_est_file_name_, 
        "vel_est.txt");
    nh_->param<std::string>("/settings/system_inekf_bias_est_file_name", bias_est_file_name_, 
        "bias_est.txt");
    nh_->param<std::string>("/settings/system_inekf_vel_input_file_name", vel_input_file_name_, 
        "vel_input.txt");
    nh_->param<std::string>("/settings/system_inekf_imu_file_name", imu_file_name_, 
        "imu.txt");

    outfile_.open(file_name_,std::ofstream::out);
    vel_est_outfile_.open(vel_est_file_name_, std::ofstream::out);
    bias_est_outfile_.open(bias_est_file_name_, std::ofstream::out);
    vel_input_outfile_.open(vel_input_file_name_, std::ofstream::out);
    imu_outfile_.open(imu_file_name_,std::ofstream::out);

    outfile_.precision(20);
    vel_est_outfile_.precision(20);
    bias_est_outfile_.precision(20);
    vel_input_outfile_.precision(20);
    imu_outfile_.precision(20);

    // set velocity update methods
    nh_->param<bool>("/settings/enable_wheel_velocity_update", enable_wheel_vel_, true);
    nh_->param<bool>("/settings/enable_camera_velocity_update", enable_camera_vel_, false);
    nh_->param<bool>("/settings/enable_gps_velocity_update", enable_gps_vel_, false);

    // Initialize pose publishing if requested
    nh_->param<bool>("/settings/system_enable_pose_publisher", enable_pose_publisher_, false);
    nh_->param<bool>("/settings/system_enable_pose_logger", enable_pose_logger_, false);
    nh_->param<bool>("/settings/system_enable_debug_logger", enable_debug_logger_, false);
    nh_->param<int>("/settings/system_log_pose_skip", log_pose_skip_, 100);

    last_imu_time_ = 0;
    skip_count_ = 0;
}

HuskySystem::~HuskySystem(){
    std::cout << "Ready to close Husky system" << std::endl;
    outfile_.close();
    vel_est_outfile_.close();
    bias_est_outfile_.close();
    vel_input_outfile_.close();
    imu_outfile_.close();
}

void HuskySystem::step() {

    // if the estimator is initialized
    if (estimator_.enabled()){
        
        // if IMU measurement exists we do prediction
        if(updateNextIMU()){
            // std::cout<<"propagate time: "<<ros::Time::now()<<std::endl;
            estimator_.propagateIMU(*(imu_packet_.get()),state_);
            if(enable_debug_logger_){
                imu_outfile_ << imu_packet_->getTime() << " " 
                << imu_packet_->angular_velocity.x << " " 
                << imu_packet_->angular_velocity.y << " " 
                << imu_packet_->angular_velocity.z << " " 
                << imu_packet_->linear_acceleration.x << " " 
                << imu_packet_->linear_acceleration.y  << " " 
                << imu_packet_->linear_acceleration.z << std::endl<<std::flush; 
            }
            new_pose_ready_ = true;
        }

        // update using body velocity from wheel encoders
        if (enable_wheel_vel_ && updateNextWheelVelocity()) {
            estimator_.correctVelocity(*(wheel_velocity_packet_.get()),state_);
            new_pose_ready_ = true;


            // record 
            auto v_in = wheel_velocity_packet_->getLinearVelocity();
            if(enable_debug_logger_){
                vel_input_outfile_ << wheel_velocity_packet_->getTime() << " " << v_in(0) << " " << v_in(1) << " " << v_in(2) << std::endl<<std::flush;
            }
        }

        // update using camera velocity
        if (enable_camera_vel_ && updateNextCameraVelocity()) {
            estimator_.correctVelocity(*(camera_velocity_packet_.get()),state_);
            new_pose_ready_ = true;

            // record 
            auto v_in = camera_velocity_packet_->getLinearVelocity();
            if(enable_debug_logger_){
                vel_input_outfile_ << camera_velocity_packet_->getTime() << " " << v_in(0) << " " << v_in(1) << " " << v_in(2) << std::endl<<std::flush;
            }
        }
        
        // update using gps velocity
        if (enable_gps_vel_ && updateNextGPSVelocity()) {
            estimator_.correctVelocity(*(gps_velocity_packet_.get()),state_);
            new_pose_ready_ = true;

            // record 
            auto v_in = gps_velocity_packet_->getLinearVelocity();
            if(enable_debug_logger_){
                vel_input_outfile_ << gps_velocity_packet_->getTime() << " " << v_in(0) << " " << v_in(1) << " " << v_in(2) << std::endl<<std::flush;
            }
        }

        if (enable_pose_publisher_ && new_pose_ready_) {
            pose_publisher_node_.posePublish(state_);
        }        

        if (enable_pose_logger_ && new_pose_ready_){
            logPoseTxt(state_);
        }

        new_pose_ready_ = false;

    }
    // initialization
    else{
        if (estimator_.biasInitialized()) {
            // wait until we receive imu msg
            while(!updateNextIMU()){};
            
            if(enable_wheel_vel_){
                while(!updateNextWheelVelocity()){}
                estimator_.initState(*(imu_packet_.get()), *(wheel_velocity_packet_.get()), state_);
            }
            else if(enable_camera_vel_){
                while(!updateNextCameraVelocity()){}
                estimator_.initState(*(imu_packet_.get()), *(camera_velocity_packet_.get()), state_);
            }
            else if(enable_gps_vel_){
                while(!updateNextGPSVelocity()){}
                estimator_.initState(*(imu_packet_.get()), *(gps_velocity_packet_.get()), state_);
            }
            
            
            estimator_.enableFilter();
            husky_data_buffer_->wheel_velocity_q = {};
            husky_data_buffer_->camera_velocity_q = {};
            husky_data_buffer_->gps_velocity_q = {};
            
            std::cout<<"State initialized."<<std::endl;
        } else {
            while(!updateNextIMU()){};
            estimator_.initBias(*(imu_packet_.get()));
        }
    }

}

void HuskySystem::logPoseTxt(const husky_inekf::HuskyState& state_) {
    if (skip_count_ == 0) {
        // ROS_INFO_STREAM("write new pose\n");
        double t = state_.getTime();

        // log pose tum style
        outfile_ << t << " "<< state_.x()<<" "<< state_.y() << " "<<state_.z() << " "<<state_.getQuaternion().x()\
        <<" "<< state_.getQuaternion().y() <<" "<< state_.getQuaternion().z() <<" "<< state_.getQuaternion().w() <<std::endl<<std::flush;    
        
        // log estimated velocity
        auto vel_est = state_.getWorldVelocity();
        vel_est_outfile_ << t << " " << vel_est(0) << " " << vel_est(1) << " " << vel_est(2)<<std::endl<<std::flush;

        // log estimated bias
        auto bias_est = state_.getImuBias();
        bias_est_outfile_ << t << " " << bias_est(0) << " " << bias_est(1) << " " << bias_est(2) << " " << bias_est(3)\
                            << " " << bias_est(4) << " " << bias_est(5) << std::endl<<std::flush;
        

        skip_count_ = log_pose_skip_;
    }
    else {
        skip_count_--;
        // std::cout<<"skipping--"<<std::endl;
    }
    
    
}


// Private Functions
bool HuskySystem::updateNextIMU() {
    std::lock_guard<std::mutex> lock(husky_data_buffer_->imu_mutex);
    if (!husky_data_buffer_->imu_q.empty()) {

        if(husky_data_buffer_->imu_q.size()>1){
            ROS_INFO_STREAM("Filter not running in real-time!");
            ROS_INFO_STREAM("IMU queue size: " <<  husky_data_buffer_->imu_q.size());
        }
        imu_packet_ = husky_data_buffer_->imu_q.front();
        husky_data_buffer_->imu_q.pop();
        // Update Husky State
        state_.setImu(imu_packet_);

        return true;
    }
    // ROS_INFO_STREAM("!!!imu not received!!!");
    return false;
}

bool HuskySystem::updateNextJointState() {
    std::lock_guard<std::mutex> lock(husky_data_buffer_->joint_state_mutex);
    if (!husky_data_buffer_->joint_state_q.empty()) {

        if(husky_data_buffer_->joint_state_q.size()>1){
            ROS_INFO_STREAM("Filter not running in real-time!");
            ROS_INFO_STREAM("Joint state queue size: " <<  husky_data_buffer_->joint_state_q.size());
        }

        joint_state_packet_ = husky_data_buffer_->joint_state_q.front();
        husky_data_buffer_->joint_state_q.pop();

        // Update Husky State
        state_.setJointState(joint_state_packet_);

        return true;
    }
    return false;
}



bool HuskySystem::updateNextWheelVelocity() {
    std::lock_guard<std::mutex> lock(husky_data_buffer_->wheel_vel_mutex);
    
    if (!husky_data_buffer_->wheel_velocity_q.empty()) {

        if(husky_data_buffer_->wheel_velocity_q.size()>1){
            ROS_INFO_STREAM("Filter not running in real-time!");
            ROS_INFO_STREAM("Velocity queue size: " <<  husky_data_buffer_->wheel_velocity_q.size());
        }

        wheel_velocity_packet_ = husky_data_buffer_->wheel_velocity_q.front();
        husky_data_buffer_->wheel_velocity_q.pop();

        return true;
    }
    // std::cout<<"velocity q empty... "<<std::endl;
    return false;
}

bool HuskySystem::updateNextCameraVelocity() {
    std::lock_guard<std::mutex> lock(husky_data_buffer_->cam_vel_mutex);
    
    if (!husky_data_buffer_->camera_velocity_q.empty()) {

        if(husky_data_buffer_->camera_velocity_q.size()>1){
            ROS_INFO_STREAM("Filter not running in real-time!");
            ROS_INFO_STREAM("Velocity queue size: " <<  husky_data_buffer_->camera_velocity_q.size());
        }

        camera_velocity_packet_ = husky_data_buffer_->camera_velocity_q.front();
        husky_data_buffer_->camera_velocity_q.pop();

        return true;
    }
    // std::cout<<"velocity q empty... "<<std::endl;
    return false;
}

bool HuskySystem::updateNextGPSVelocity() {
    std::lock_guard<std::mutex> lock(husky_data_buffer_->gps_vel_mutex);
    
    if (!husky_data_buffer_->gps_velocity_q.empty()) {

        if(husky_data_buffer_->gps_velocity_q.size()>1){
            ROS_INFO_STREAM("Filter not running in real-time!");
            ROS_INFO_STREAM("Velocity queue size: " <<  husky_data_buffer_->gps_velocity_q.size());
        }

        gps_velocity_packet_ = husky_data_buffer_->gps_velocity_q.front();
        husky_data_buffer_->gps_velocity_q.pop();

        return true;
    }
    // std::cout<<"velocity q empty... "<<std::endl;
    return false;
}