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
        "husky_inekf_kitti_pose.txt");
    nh_->param<std::string>("/settings/system_inekf_tum_pose_filename", tum_file_name_, 
        "husky_inekf_tum_pose.txt");
    nh_->param<std::string>("/settings/system_inekf_vel_est_file_name", vel_est_file_name_, 
        "vel_est.txt");
    nh_->param<std::string>("/settings/system_inekf_bias_est_file_name", bias_est_file_name_, 
        "bias_est.txt");
    nh_->param<std::string>("/settings/system_inekf_vel_input_file_name", vel_input_file_name_, 
        "vel_input.txt");
    nh_->param<std::string>("/settings/system_inekf_imu_file_name", imu_file_name_, 
        "imu.txt");

    outfile_.open(file_name_,std::ofstream::out);
    tum_outfile_.open(tum_file_name_, std::ofstream::out);
    vel_est_outfile_.open(vel_est_file_name_, std::ofstream::out);
    bias_est_outfile_.open(bias_est_file_name_, std::ofstream::out);
    vel_input_outfile_.open(vel_input_file_name_, std::ofstream::out);
    imu_outfile_.open(imu_file_name_,std::ofstream::out);

    outfile_.precision(20);
    tum_outfile_.precision(20);
    vel_est_outfile_.precision(20);
    bias_est_outfile_.precision(20);
    vel_input_outfile_.precision(20);
    imu_outfile_.precision(20);

    // Initialize pose publishing if requested
    nh_->param<bool>("/settings/system_enable_pose_publisher", enable_pose_publisher_, false);
    nh_->param<bool>("/settings/system_enable_pose_logger", enable_pose_logger_, false);
    nh_->param<bool>("/settings/system_enable_debug_logger", enable_debug_logger_, false);
    nh_->param<int>("/settings/system_log_pose_skip", log_pose_skip_, 100);
    skip_count_ = 0;
    // nh_->param<int>("/settings/system_velocity_type", velocity_type_, 0);

    last_imu_time_ = 0;
    skip_count_ = 0;
}

HuskySystem::~HuskySystem(){
    std::cout << "Ready to close Husky system" << std::endl;
    outfile_.close();
    tum_outfile_.close();
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

        if (updateNextVelocity()) {
            estimator_.correctVelocity(*(velocity_packet_.get()),state_);

            // record 
            auto v_in = velocity_packet_->getLinearVelocity();
            if(enable_debug_logger_){
                vel_input_outfile_ << velocity_packet_->getTime() << " " << v_in(0) << " " << v_in(1) << " " << v_in(2) << std::endl<<std::flush;
            }
            
            new_pose_ready_ = true;
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
            
            while(!updateNextVelocity()){}
            
            estimator_.initState(*(imu_packet_.get()), *(velocity_packet_.get()), state_);
            
            estimator_.enableFilter();
            husky_data_buffer_->velocity_q = {};
            
            std::cout<<"State initialized."<<std::endl;
        } else {
            while(!updateNextIMU()){};
            estimator_.initBias(*(imu_packet_.get()));
        }
    }

}

void HuskySystem::logPoseTxt(const husky_inekf::HuskyState& state_) {
    // std::cout<<"before counting"<<std::endl;
    if (skip_count_ == 0) {
        // ROS_INFO_STREAM("write new pose\n");
        double t = state_.getTime();


        // log pose kitti style
        outfile_ << "1 0 0 "<< state_.x() <<" 0 1 0 "<< state_.y() <<" 0 0 1 "<< state_.z() <<std::endl<<std::flush;

        // log pose tum style
        tum_outfile_ << t << " "<< state_.x()<<" "<< state_.y() << " "<<state_.z() << " "<<state_.getQuaternion().x()\
        <<" "<< state_.getQuaternion().y() <<" "<< state_.getQuaternion().z() <<" "<< state_.getQuaternion().w() <<std::endl<<std::flush;    

        // log estimated velocity
        auto vel_est = state_.getWorldVelocity();
        vel_est_outfile_ << t << " " << vel_est(0) << " " << vel_est(1) << " " << vel_est(2)<<std::endl<<std::flush;

        // log estimated bias
        auto bias_est = state_.getImuBias();
        bias_est_outfile_ << t << " " << bias_est(0) << " " << bias_est(1) << " " << bias_est(2) << " " << bias_est(3)\
                            << " " << bias_est(4) << " " << bias_est(5) << std::endl<<std::flush;
        

        skip_count_ = log_pose_skip_;
        // tum_outfile.close();
    }
    else {
        skip_count_--;
        // std::cout<<"skipping--"<<std::endl;
    }
    
    
}


// Private Functions
bool HuskySystem::updateNextIMU() {
    // husky_data_buffer_->imu_mutex.lock();
    std::lock_guard<std::mutex> lock(husky_data_buffer_->imu_mutex);
    if (!husky_data_buffer_->imu_q.empty()) {

        if(husky_data_buffer_->imu_q.size()>1){
            ROS_INFO_STREAM("Filter not running in real-time!");
            ROS_INFO_STREAM("IMU queue size: " <<  husky_data_buffer_->imu_q.size());

            // std::cout<< std::setprecision(20) << husky_data_buffer_->imu_q.front()->getTime()<<std::endl;
            // std::cout<< std::setprecision(20) << husky_data_buffer_->imu_q.back()->getTime()<<std::endl;
        }
        imu_packet_ = husky_data_buffer_->imu_q.front();
        husky_data_buffer_->imu_q.pop();
        // husky_data_buffer_->imu_mutex.unlock();
        // Update Husky State
        state_.setImu(imu_packet_);

        // std::cout<<"IMU time diff: "<<imu_packet_->getTime() - last_imu_time_<<std::endl;
        // last_imu_time_ = imu_packet_->getTime();

        return true;
    }
    // ROS_INFO_STREAM("!!!imu not received!!!");
    // husky_data_buffer_->imu_mutex.unlock();
    return false;
}

bool HuskySystem::updateNextJointState() {
    // husky_data_buffer_->joint_state_mutex.lock();
    std::lock_guard<std::mutex> lock(husky_data_buffer_->joint_state_mutex);
    if (!husky_data_buffer_->joint_state_q.empty()) {

        if(husky_data_buffer_->joint_state_q.size()>1){
            ROS_INFO_STREAM("Filter not running in real-time!");
            ROS_INFO_STREAM("Joint state queue size: " <<  husky_data_buffer_->joint_state_q.size());
        }

        joint_state_packet_ = husky_data_buffer_->joint_state_q.front();
        husky_data_buffer_->joint_state_q.pop();
        // drop everything older than the top measurement on the stack 
        // husky_data_buffer_->joint_state_q.clear();

        // Update Husky State
        state_.setJointState(joint_state_packet_);

        return true;
    }
    return false;
}



bool HuskySystem::updateNextVelocity() {
    std::lock_guard<std::mutex> lock(husky_data_buffer_->velocity_mutex);
    
    if (!husky_data_buffer_->velocity_q.empty()) {

        if(husky_data_buffer_->velocity_q.size()>1){
            ROS_INFO_STREAM("Filter not running in real-time!");
            ROS_INFO_STREAM("Velocity queue size: " <<  husky_data_buffer_->velocity_q.size());
        }

        velocity_packet_ = husky_data_buffer_->velocity_q.front();
        husky_data_buffer_->velocity_q.pop();

        // drop everything older than the top measurement on the stack 
        // husky_data_buffer_->joint_state_q.clear();

        // Update Husky State
        // state_.setVelocity(velocity_packet_);

        return true;
    }
    // std::cout<<"velocity q empty... "<<std::endl;
    return false;
}