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
        "/media/jetson256g/data/inekf_result/husky_inekf_pose.txt");
    nh_->param<std::string>("/settings/system_inekf_tum_pose_filename", tum_file_name_, 
        "/media/jetson256g/data/inekf_result/husky_inekf_tum_pose.txt");

    
    outfile_.open(file_name_,std::ofstream::out);
    tum_outfile_.open(tum_file_name_, std::ofstream::out);

    outfile_.precision(20);
    tum_outfile_.precision(20);
    // Initialize pose publishing if requested
    nh_->param<bool>("/settings/system_enable_pose_publisher", enable_pose_publisher_, false);
    nh_->param<bool>("/settings/system_enable_pose_logger", enable_pose_logger_, false);
    nh_->param<int>("/settings/system_log_pose_skip", log_pose_skip_, 100);

}

HuskySystem::~HuskySystem(){
    outfile_.close();
    tum_outfile_.close();
}

void HuskySystem::step() {

    // boost::timer::auto_cpu_timer t;
    // if the estimator is initialized
    if (estimator_.enabled()){

        // if IMU measurement exists we do prediction
        if(updateNextIMU()){
            estimator_.propagateIMU(*(imu_packet_.get()),state_);
            new_pose_ready_ = true;
        }

        // if velocity measurement exsits we do correction
        if(updateNextJointState()){
            estimator_.correctVelocity(*(joint_state_packet_.get()),state_);
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
            // wait until we receive joint state msg
            while(!updateNextJointState()){};
            
            estimator_.initState(*(imu_packet_.get()), *(joint_state_packet_.get()), state_);
            estimator_.enableFilter();
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
        // std::ofstream outfile(file_name_,std::ofstream::out | std::ofstream::app );
        outfile_ << "1 0 0 "<< state_.x() <<" 0 1 0 "<< state_.y() <<" 0 0 1 "<< state_.z() <<std::endl<<std::flush;
        // outfile.close();
        // tum style
        // std::ofstream tum_outfile(tum_file_name_,std::ofstream::out | std::ofstream::app );
        tum_outfile_ << state_.getTime() << " "<< state_.x()<<" "<< state_.y() << " "<<state_.z() << " "<<state_.getQuaternion().x()\
        <<" "<< state_.getQuaternion().y() <<" "<< state_.getQuaternion().z() <<" "<< state_.getQuaternion().w() <<std::endl<<std::flush;    
        skip_count_ = log_pose_skip_;
    }
    else {
        skip_count_--;
    }
    
    // tum_outfile.close();
    
}

// Private Functions
bool HuskySystem::updateNextIMU() {
    // husky_data_buffer_->imu_mutex.lock();
    std::lock_guard<std::mutex> lock(husky_data_buffer_->imu_mutex);
    if (!husky_data_buffer_->imu_q.empty()) {

        if(husky_data_buffer_->imu_q.size()>1){
            ROS_INFO_STREAM("Filter not running in real-time!");
            ROS_INFO_STREAM("IMU queue size: " <<  husky_data_buffer_->imu_q.size());
        }
        imu_packet_ = husky_data_buffer_->imu_q.front();
        husky_data_buffer_->imu_q.pop();
        // husky_data_buffer_->imu_mutex.unlock();
        // Update Husky State
        state_.setImu(imu_packet_);

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

        joint_state_packet_ = husky_data_buffer_->joint_state_q.back();
        // drop everything older than the top measurement on the stack 
        husky_data_buffer_->joint_state_q.clear();

        // Update Husky State
        state_.setJointState(joint_state_packet_);

        return true;
    }
    return false;
}