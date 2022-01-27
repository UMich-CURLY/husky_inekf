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
    // nh_->param<int>("/settings/system_velocity_type", velocity_type_, 0);

    last_imu_time_ = 0;
}

HuskySystem::~HuskySystem(){
    outfile_.close();
    tum_outfile_.close();
}

void HuskySystem::step() {

    // boost::timer::auto_cpu_timer t;
    // boost::timer::cpu_timer timer;
    // if the estimator is initialized
    
    if (estimator_.enabled()){
        
        // if IMU measurement exists we do prediction
        if(updateNextIMU()){
            // std::cout<<"propagate time: "<<ros::Time::now()<<std::endl;
            estimator_.propagateIMU(*(imu_packet_.get()),state_);
            new_pose_ready_ = true;
        }

        if (updateNextJointState() && updateNextVelocity()) {
            estimator_.correctVelocity(*(velocity_packet_.get()),state_);
            // std::cout << "new_pose_ready_" << std::endl;
            new_pose_ready_ = true;
        }

        // if(updateNextJointState()){
        //     estimator_.correctVelocity(*(joint_state_packet_.get()),state_);
        //     new_pose_ready_ = true;
        // }

        // if(velocity_type_ == 0){
        //     // if velocity measurement exsits we do correction
        //     if(updateNextJointState()){
        //         estimator_.correctVelocity(*(joint_state_packet_.get()),state_);
        //         new_pose_ready_ = true;
        //     }
        // }
        // else if(velocity_type_ == 1){
        //     if(updateNextVelocity()){
        //         estimator_.correctVelocity(*(velocity_packet_.get()),state_);
        //         new_pose_ready_ = true;
        //     }
        // }
        // else if(velocity_type_ == 3){
        //     if(updateNextVelocity()){
        //         estimator_.correctVelocity(*(velocity_packet_.get()),state_);
        //         new_pose_ready_ = true;
        //     }
        //     if(updateNextJointState()){
        //         estimator_.correctVelocity(*(joint_state_packet_.get()),state_);
        //         new_pose_ready_ = true;
        //     }
        // }
        

        if (enable_pose_publisher_ && new_pose_ready_) {
            pose_publisher_node_.posePublish(state_);
            // std::cout << timer.format() << '\n';
            // std::cout<<"publish pose: "<<ros::Time::now()<<std::endl;
            // std::cout<<"-------------"<<std::endl;
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
            // husky_data_buffer_->joint_state_q = {};
            husky_data_buffer_->velocity_q = {};

            // if(velocity_type_ == 0){
            //     // wait until we receive joint state msg
            //     while(!updateNextJointState()){};
            //     estimator_.initState(*(imu_packet_.get()), *(joint_state_packet_.get()), state_);
            //     husky_data_buffer_->joint_state_q = {};
            // }
            // else if(velocity_type_ == 1){
            //     // wait until we receive joint state msg

            //     while(!updateNextVelocity()){};
            //     estimator_.initState(*(imu_packet_.get()), *(velocity_packet_.get()), state_);
            //     husky_data_buffer_->velocity_q = {};
            // }
            // else if(velocity_type_ == 3){
            //     // wait until we receive joint state msg
            //     while(!updateNextJointState()){};
            //     estimator_.initState(*(imu_packet_.get()), *(joint_state_packet_.get()), state_);
            //     husky_data_buffer_->joint_state_q = {};
            //     husky_data_buffer_->velocity_q = {};
            // }
            
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
    return false;
}