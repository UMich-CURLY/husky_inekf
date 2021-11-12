#include "system/husky_system.hpp"
// Thread safe locking
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/circular_buffer.hpp>

#include <vector>
#include <numeric>

HuskySystem::HuskySystem(ros::NodeHandle* nh, husky_inekf::husky_data_t* husky_data_buffer): 
    nh_(nh), husky_data_buffer_(husky_data_buffer), pose_publisher_node_(nh) {
    // Initialize inekf pose file printouts
    nh_->param<std::string>("/settings/system_inekf_pose_filename", file_name_, 
        "/media/jetson256g/data/inekf_result/husky_inekf_pose.txt");
    nh_->param<std::string>("/settings/system_inekf_tum_pose_filename", tum_file_name_, 
        "/media/jetson256g/data/inekf_result/husky_inekf_tum_pose.txt");

    std::ofstream outfile(file_name_);
    std::ofstream tum_outfile(tum_file_name_);
    outfile.close();
    tum_outfile.close();

    // Initialize pose publishing if requested
    nh_->param<bool>("/settings/system_enable_pose_publisher", enable_pose_publisher_, false);

}

void HuskySystem::step() {
    
    // if the estimator is initialized
    if (estimator_.enabled()){

        // if IMU measurement exists we do prediction
        if(updateNextIMU()){
            estimator_.propagateIMU(*(imu_packet_.get()),state_);
        }

        // if velocity measurement exsits we do correction
        if(updateNextJointState()){
            estimator_.correctVelocity(*(joint_state_packet_.get()),state_);
        }

        if (enable_pose_publisher_) {
            pose_publisher_node_.posePublish(state_);
            poseCallback(state_);
        }        

    }
    // initialization
    else{

        std::cout << "Initialized initState" << std::endl;
        if (estimator_.biasInitialized()) {
            // wait until we receive imu msg
            while(!updateNextIMU()){};
            // wait until we receive joint state msg
            while(!updateNextJointState()){};
            
            estimator_.initState(*(imu_packet_.get()), *(joint_state_packet_.get()), state_);
            estimator_.enableFilter();
        } else {
            
            while(!updateNextIMU()){};
            estimator_.initBias(*(imu_packet_.get()));
        }
    }

}

void HuskySystem::poseCallback(const husky_inekf::HuskyState& state_) {
    if (file_name_.size() > 0) {
        // ROS_INFO_STREAM("write new pose\n");
        std::ofstream outfile(file_name_,std::ofstream::out | std::ofstream::app );
        outfile << "1 0 0 "<< state_.x() <<" 0 1 0 "<< state_.y() <<" 0 0 1 "<< state_.z() <<std::endl<<std::flush;
        outfile.close();
        // tum style
        std::ofstream tum_outfile(tum_file_name_,std::ofstream::out | std::ofstream::app );
        tum_outfile << state_.getTime() << " "<< state_.x()<<" "<< state_.y() << " "<<state_.z() << " "<<state_.getQuaternion().x()\
        <<" "<< state_.getQuaternion().y() <<" "<< state_.getQuaternion().z() <<" "<< state_.getQuaternion().w() <<std::endl<<std::flush;
        
        tum_outfile.close();
    }
}

// Private Functions

bool HuskySystem::updateNextIMU() {
    husky_data_buffer_->imu_mutex.lock();

    if (!husky_data_buffer_->imu_q.empty()) {
        imu_packet_ = husky_data_buffer_->imu_q.front();
        husky_data_buffer_->imu_q.pop();
        return true;
    }
    return false;
}

bool HuskySystem::updateNextJointState() {
    husky_data_buffer_->joint_state_mutex.lock();
    
    if (!husky_data_buffer_->joint_state_q.empty()) {
        joint_state_packet_ = husky_data_buffer_->joint_state_q.front();
        husky_data_buffer_->joint_state_q.pop();
        return true;
    }
    return false;
}