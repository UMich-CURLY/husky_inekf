#include "system/husky_system.hpp"
// Thread safe locking
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/circular_buffer.hpp>

#include <vector>
#include <numeric>

HuskySystem::HuskySystem(ros::NodeHandle* nh, husky_inekf_data::husky_data_t* husky_data_buffer): 
    nh_(nh), ts_(0.05, 0.05), cheetah_buffer_(cheetah_buffer), cdata_mtx_(cdata_mtx), estimator_(lcm), pose_publisher_node_(nh) {
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
            estimator_.update(imu_packet_,state_);
        }

        // if velocity measurement exsits we do correction
        if(updateNextJointState()){
            estimator_.correctVelocity(joint_state_packet_,state_);
        }

        if (enable_pose_publisher_) {
                pose_publisher_node_.posePublish(state_);
                poseCallback(state_);
        }        

    }
    // initialization
    else{

        // wait until we receive imu msg
        while(!updateNextIMU()){};
        // wait until we receive joint state msg
        while(!updateNextJointState()){};

        std::cout << "Initialized initState" << std::endl;
        if (estimator_.biasInitialized()) {
            estimator_.initState(imu_packet_, joint_state_packet_, state_);
            estimator_.enableFilter();
        } else {
            estimator_.initBias(imu_packet_);
        }
    }

}

void HuskySystem::poseCallback(const HuskyState& state_) {
    if (file_name_.size() > 0) {
        // ROS_INFO_STREAM("write new pose\n");
        std::ofstream outfile(file_name_,std::ofstream::out | std::ofstream::app );
        outfile << "1 0 0 "<< state_.x() <<" 0 1 0 "<< state_.y() <<" 0 0 1 "<< state_.z() <<std::endl<<std::flush;
        outfile.close();
        // tum style
        std::ofstream tum_outfile(tum_file_name_,std::ofstream::out | std::ofstream::app );
        tum_outfile << cheetah_packet_.getTime() << " "<< state_.x()<<" "<< state_.y() << " "<<state_.z() << " "<<state_.getQuaternion().x()\
        <<" "<< state_.getQuaternion().y() <<" "<< state_.getQuaternion().z() <<" "<< state_.getQuaternion().w() <<std::endl<<std::flush;
        
        tum_outfile.close();
    }
}

// Private Functions

// bool HuskySystem::updateNextPacket() {
//     //Copy data to be handled in queues (lock/unlock)
//     bool hasUpdated = false;
//     cdata_mtx_->lock();
//     if (!cheetah_buffer_->timestamp_q.empty() &&
//         !cheetah_buffer_->imu_q.empty() &&
//         !cheetah_buffer_->joint_state_q.empty() &&
//         !cheetah_buffer_->contact_q.empty()) 
//     {
//         hasUpdated = true;
//         double timestamp = cheetah_buffer_->timestamp_q.front();
//         cheetah_packet_.setTime(timestamp);
//         cheetah_packet_.imu = cheetah_buffer_->imu_q.front();
//         cheetah_packet_.joint_state = cheetah_buffer_->joint_state_q.front();
//         cheetah_packet_.contact = cheetah_buffer_->contact_q.front();

//         cheetah_buffer_->timestamp_q.pop();
//         cheetah_buffer_->imu_q.pop();
//         cheetah_buffer_->joint_state_q.pop();
//         cheetah_buffer_->contact_q.pop();
//     }
//     cdata_mtx_->unlock();

//     return hasUpdated;
// }
