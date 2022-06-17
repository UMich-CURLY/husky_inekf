#include "communication/husky_comms.h"


#include <boost/timer/timer.hpp>

HuskyComms::HuskyComms( ros::NodeHandle* nh, husky_inekf::husky_data_t* husky_data_buffer)
                        : nh_(nh), husky_data_buffer_(husky_data_buffer)
{
    std::string imu_topic, joint_topic, wheel_vel_topic, camera_vel_topic, gps_vel_topic;
    
    nh_->param<std::string>("/settings/imu_topic", imu_topic, "/gx5_0/imu/data");
    nh_->param<std::string>("/settings/joint_topic", joint_topic, "/joint_states");

    nh_->param<double>("/settings/wheel_radius",wheel_radius_,0.1651);
    nh_->param<double>("/settings/vehicle_track_width",vehicle_track_width_,0.555);
    nh_->param<double>("/settings/vehicle_length",vehicle_length_,0.540);

    // Velocity Type:
    nh_->param<bool>("/settings/enable_wheel_velocity_update", enable_wheel_vel_, true);
    nh_->param<std::string>("/settings/wheel_velocity_topic", wheel_vel_topic, "/joint_states");
    nh_->param<bool>("/settings/enable_camera_velocity_update", enable_camera_vel_, false);
    nh_->param<std::string>("/settings/camera_velocity_topic", camera_vel_topic, "/zed_node/odom");
    nh_->param<bool>("/settings/enable_gps_velocity_update", enable_gps_vel_, false);
    nh_->param<std::string>("/settings/gps_velocity_topic", gps_vel_topic, "/gps/vel");

    // TODO: Check usage
    // Odom To IMU frame:

    std::vector<double> translation_cam_body;
    std::vector<double> rotation_cam_body;
    cam_to_body_ = Eigen::Matrix4d::Identity();

    nh_->param<std::vector<double>>("settings/translation_cam_body", translation_cam_body, std::vector<double>({0, 0, 0}));
    nh_->param<std::vector<double>>("settings/rotation_cam_body", rotation_cam_body, std::vector<double>({1, 0, 0, 0}));

    Eigen::Quaternion<double> orientation_quat(rotation_cam_body[0],
                                                rotation_cam_body[1],
                                                rotation_cam_body[2],
                                                rotation_cam_body[3]);
    
    cam_to_body_.block<3,3>(0,0) = orientation_quat.toRotationMatrix();
    cam_to_body_.block<3,1>(0,3) = Eigen::Vector3d({translation_cam_body[0], translation_cam_body[1], translation_cam_body[2]});
    
    // Rotation from IMU to body frame. Body frame definition: X forward, Y left, Z up.
    nh_->param<std::vector<double>>("settings/rotation_imu_body", rotation_imu_body_, std::vector<double>({0, 0.7071, -0.7071, 0}));

    std::cout<<"subscribing to: "<< imu_topic << joint_topic<<std::endl;

    // Initialize subscribers with queue size of 1000
    imu_sub_ = nh_->subscribe(imu_topic, 1000, &HuskyComms::imuCallback, this);

    // velocity from wheel encoders. msg_type: sensor_msgs::JointState
    if(enable_wheel_vel_){
        wheel_vel_sub_ = nh_->subscribe(wheel_vel_topic, 1000, &HuskyComms::jointStateVelocityCallback, this);
    }
    else{
        joint_sub_ = nh_->subscribe(joint_topic, 1000, &HuskyComms::jointStateCallback, this);
    }

    // velocity from camera odometry. msg_type: nav_msgs::Odometry
    if(enable_camera_vel_){
        cam_vel_sub_ = nh_->subscribe(camera_vel_topic, 1000, &HuskyComms::CameraOdomCallBack, this);
    }

    // velocity from GPS. msg_type: geometry_msgs::TwistStamped
    if(enable_gps_vel_){
        gps_vel_sub_ = nh_->subscribe(gps_vel_topic, 1000, &HuskyComms::GPSvelocityCallback, this);
    }

    // start the subscribing thread
    subscribing_thread_ = std::thread([this]{this->sub();});
}

HuskyComms::~HuskyComms()
{
    subscribing_thread_.join();
}


void HuskyComms::sub(){
    while(ros::ok()){
        ros::spinOnce();
    }
}

// Note no transformation mat needed between imu and odometry for husky
void HuskyComms::imuCallback(const sensor_msgs::Imu& imu_msg) 
{
    auto imu_ptr = 
        std::make_shared<husky_inekf::ImuMeasurement<double> >(imu_msg, rotation_imu_body_);

    std::lock_guard<std::mutex> lock(husky_data_buffer_->imu_mutex);
    husky_data_buffer_->imu_q.push(imu_ptr);
}


void HuskyComms::jointStateCallback(const sensor_msgs::JointState& joint_msg)
{   
    auto joint_ptr =
        std::make_shared<husky_inekf::JointStateMeasurement>(joint_msg, 4, wheel_radius_, vehicle_track_width_, vehicle_length_);

    std::lock_guard<std::mutex> lock(husky_data_buffer_->joint_state_mutex);
    husky_data_buffer_->joint_state_q.push(joint_ptr);
}


void HuskyComms::jointStateVelocityCallback(const sensor_msgs::JointState& joint_msg)
{   
    auto joint_state_ptr = std::make_shared<husky_inekf::JointStateMeasurement>(joint_msg, 4, wheel_radius_, vehicle_track_width_, vehicle_length_);

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

    std::lock_guard<std::mutex> lock(husky_data_buffer_->wheel_vel_mutex);
    husky_data_buffer_ -> wheel_velocity_q.push(vel_ptr);
        
}


void HuskyComms::GPSvelocityCallback(const geometry_msgs::TwistStamped& vel_msg){
    auto vel_ptr = std::make_shared<husky_inekf::VelocityMeasurement>(vel_msg);
    std::lock_guard<std::mutex> lock(husky_data_buffer_->gps_vel_mutex);
    husky_data_buffer_ -> gps_velocity_q.push(vel_ptr);
}

void HuskyComms::CameraOdomCallBack(const nav_msgs::Odometry& camera_odom_msg) {
    auto camera_odom_ptr = std::make_shared<husky_inekf::CameraOdomMeasurement>(camera_odom_msg);
    
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

    Eigen::Matrix4d transformation = cam_to_body_.inverse() * prev_transformation.inverse() * curr_transformation * cam_to_body_;
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

    std::lock_guard<std::mutex> lock(husky_data_buffer_->cam_vel_mutex);
    husky_data_buffer_ -> camera_velocity_q.push(vel_ptr);
}
