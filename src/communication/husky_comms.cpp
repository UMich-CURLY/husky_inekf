#include "communication/husky_comms.h"


#include <boost/timer/timer.hpp>

HuskyComms::HuskyComms( ros::NodeHandle* nh, husky_inekf::husky_data_t* husky_data_buffer)
                        : nh_(nh), husky_data_buffer_(husky_data_buffer)
{
    std::string imu_topic, joint_topic;

    nh_->param<std::string>("/settings/imu_topic", imu_topic, "/gx5_0/imu/data");
    nh_->param<std::string>("/settings/joint_topic", joint_topic, "/joint_states");

    std::cout<<"husky comms nh namespace: "<<ros::this_node::getNamespace()<<std::endl;

    std::cout<<"subscribing to: "<<imu_topic<<", and "<<joint_topic<<std::endl;
    // Initialize subscribers with queue size of 1000
    imu_sub_ = nh_->subscribe(imu_topic, 1000, &HuskyComms::imuCallback, this);
    joint_sub_ = nh_->subscribe(joint_topic, 1000, &HuskyComms::jointStateCallback, this);


    // start the subscribing thread
    subscribing_thread_ = std::thread([this]{this->sub();});
}

HuskyComms::~HuskyComms()
{
    subscribing_thread_.join();
}


void HuskyComms::sub(){
    // ros::MultiThreadedSpinner spinner(2); // Use 2 threads
    // spinner.spin();
    while(ros::ok()){
        ros::spinOnce();
    }
}

// Note no transformation mat needed between imu and odometry for husky
void HuskyComms::imuCallback(const sensor_msgs::Imu& imu_msg) 
{
    // std::cout<<"getting imu at time: "<<ros::Time::now()<<std::endl;
    // boost::timer::auto_cpu_timer t;
    auto imu_ptr = 
        std::make_shared<husky_inekf::ImuMeasurement<double> >(imu_msg);
        

    std::lock_guard<std::mutex> lock(husky_data_buffer_->imu_mutex);
    // if(husky_data_buffer_->imu_q.size()>0){
    //     std::cout<< std::setprecision(20)<<"Queue already have imu with time: "<<husky_data_buffer_->imu_q.front()->getTime()<<std::endl;
    //     std::cout<< std::setprecision(20)<<"Pushing second imu with time: "<<imu_ptr->getTime()<<std::endl;
    //     std::cout<<"-----------------"<<std::endl;
    // }
    husky_data_buffer_->imu_q.push(imu_ptr);
}


void HuskyComms::jointStateCallback(const sensor_msgs::JointState& joint_msg)
{   
    auto joint_ptr =
        std::make_shared<husky_inekf::JointStateMeasurement>(joint_msg, 4);

    std::lock_guard<std::mutex> lock(husky_data_buffer_->joint_state_mutex);
    husky_data_buffer_->joint_state_q.push(joint_ptr);
    // std::cout<<"joint state size: "<<husky_data_buffer_->joint_state_q.size()<<std::endl;
}
