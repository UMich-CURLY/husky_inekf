#include "communication/husky_comms.h"


#include <boost/timer/timer.hpp>

HuskyComms::HuskyComms( ros::NodeHandle* nh, husky_inekf::husky_data_t* husky_data_buffer)
                        : nh_(nh), husky_data_buffer_(husky_data_buffer)
{
    std::string imu_topic, joint_topic, velocity_topic;
    int velocity_type;

    nh_->param<std::string>("/settings/imu_topic", imu_topic, "/gx5_0/imu/data");
    nh_->param<std::string>("/settings/joint_topic", joint_topic, "/joint_states");

    // Velocity topic:
    nh_->param<std::string>("/settings/velocity_topic", velocity_topic, "/gps/vel");

    // Velocity Type:
    nh_->param<int>("/settings/velocity_type", velocity_type, 1);

    // Odom To IMU frame:
    nh_->param<std::vector<double>>("settings/translation", translation_imu, std::vector<double>({0, 0, 0, 0}));
    nh_->param<std::vector<double>>("settings/rotation", rotation_imu, std::vector<double>({0, 0, 0, 1}));

    std::cout<<"husky comms nh namespace: "<<ros::this_node::getNamespace()<<std::endl;

    std::cout<<"subscribing to: "<< imu_topic << joint_topic<<", and "<<velocity_topic << std::endl;
    // Initialize subscribers with queue size of 1000
    imu_sub_ = nh_->subscribe(imu_topic, 1000, &HuskyComms::imuCallback, this);

    switch(velocity_type) {
        case 1: // JOINT_STATE
        {
            vel_sub_ = nh_->subscribe(velocity_topic, 1000, &HuskyComms::jointStateVelocityCallback, this);
        }
            break;
        case 2: // GPS_VELOCITY
        {
            joint_sub_ = nh_->subscribe(joint_topic, 1000, &HuskyComms::jointStateCallback, this);
            vel_sub_ = nh_->subscribe(velocity_topic, 1000, &HuskyComms::GPSvelocityCallback, this);
        }
            break;
        case 3: // CAMERA_ODOM
        {
            joint_sub_ = nh_->subscribe(joint_topic, 1000, &HuskyComms::jointStateCallback, this);
            vel_sub_ = nh_->subscribe(velocity_topic, 1000, &HuskyComms::CameraOdomCallBack, this);
        }
            break;
        default:
            std::cout << "Invalid velocity measurement" << std::endl;
    }

    // start the subscribing thread
    subscribing_thread_ = std::thread([this]{this->sub();});
}

HuskyComms::~HuskyComms()
{
    subscribing_thread_.join();
}


void HuskyComms::sub(){
    // ros::MultiThreadedSpinner spinner(4); // Use 2 threads
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


void HuskyComms::jointStateVelocityCallback(const sensor_msgs::JointState& joint_msg)
{   
    if (joint_msg.velocity.size() == 4) {
        auto joint_state_ptr = std::make_shared<husky_inekf::JointStateMeasurement>(joint_msg, 4);

        // set velocity message:
        geometry_msgs::TwistStamped vel_msg;
        vel_msg.header = joint_msg.header;

        vel_msg.twist.linear.x = joint_state_ptr->getBodyLinearVelocity()(0);
        vel_msg.twist.linear.y = joint_state_ptr->getBodyLinearVelocity()(1);
        vel_msg.twist.linear.z = joint_state_ptr->getBodyLinearVelocity()(2);

        vel_msg.twist.angular.x = joint_state_ptr->getBodyAngularVelocity()(0);
        vel_msg.twist.angular.y = joint_state_ptr->getBodyAngularVelocity()(1);
        vel_msg.twist.angular.z = joint_state_ptr->getBodyAngularVelocity()(2);
        auto vel_ptr = std::make_shared<husky_inekf::VelocityMeasurement>(vel_msg);


        std::lock_guard<std::mutex> lock2(husky_data_buffer_->joint_state_mutex);
        husky_data_buffer_->joint_state_q.push(joint_state_ptr);

        std::lock_guard<std::mutex> lock(husky_data_buffer_->velocity_mutex);
        husky_data_buffer_ -> velocity_q.push(vel_ptr);
        
    }
}


void HuskyComms::GPSvelocityCallback(const geometry_msgs::TwistStamped& vel_msg){
    auto vel_ptr = std::make_shared<husky_inekf::VelocityMeasurement>(vel_msg);
    std::lock_guard<std::mutex> lock(husky_data_buffer_->velocity_mutex);
    husky_data_buffer_ -> velocity_q.push(vel_ptr);
}

void HuskyComms::CameraOdomCallBack(const nav_msgs::Odometry& camera_odom_msg) {
    auto camera_odom_ptr = std::make_shared<husky_inekf::CameraOdomMeasurement>(camera_odom_msg, translation_imu, rotation_imu);
    
    // We need two odometry data to calculate the velocity
    if (husky_data_buffer_ -> camera_odom_q.empty()) {
        husky_data_buffer_ -> camera_odom_q.push(camera_odom_ptr);
        return;
    }

    auto prev_transformation = 
            husky_data_buffer_->camera_odom_q.front().get()->getTransformation();
    double prev_time = husky_data_buffer_->camera_odom_q.front().get()->getTime();

    husky_data_buffer_->camera_odom_q.pop();

    // Insert the current camera odometry:
    husky_data_buffer_ -> camera_odom_q.push(camera_odom_ptr);
    auto curr_transformation = camera_odom_ptr->getTransformation();
    double curr_time = camera_odom_ptr->getTime();

    double time_diff = curr_time - prev_time;

    Eigen::Matrix4d transformation = prev_transformation.inverse() * curr_transformation;
    Eigen::Matrix4d twist_se3 = transformation.log();

    geometry_msgs::TwistStamped vel_msg;

    vel_msg.header = camera_odom_msg.header;

    vel_msg.twist.linear.x = twist_se3(0, 3) / time_diff;
    vel_msg.twist.linear.y = twist_se3(1, 3) / time_diff;
    vel_msg.twist.linear.z = twist_se3(2, 3) / time_diff;

    vel_msg.twist.angular.x = twist_se3(2, 1) / time_diff;
    vel_msg.twist.angular.y = -twist_se3(2, 0) / time_diff;
    vel_msg.twist.angular.z = twist_se3(1, 0) / time_diff;
    
    auto vel_ptr = std::make_shared<husky_inekf::VelocityMeasurement>(vel_msg);

    std::lock_guard<std::mutex> lock(husky_data_buffer_->velocity_mutex);
    husky_data_buffer_ -> velocity_q.push(vel_ptr);
}
