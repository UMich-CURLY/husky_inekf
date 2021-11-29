#include "communication/pose_publisher_node.hpp"

PosePublisherNode::PosePublisherNode(ros::NodeHandle* n) : n_(n) {
    // Create private node handle
    ros::NodeHandle nh("~");
    // std::string pose_csv_file, init_rot_file;
    std::string pose_topic, pose_frame;

    nh.param<std::string>("pose_topic", pose_topic, "/husky/inekf_estimation/pose");
    nh.param<std::string>("pose_frame", pose_frame, "/odom");
    nh.param<double>("publish_rate", publish_rate_, 1000); 
    nh.param<int>("pose_skip", pose_skip_, 0); 
    first_pose_ = {0, 0, 0};
    // first_pose_ = pose_from_csv_.front();
    std::cout<<"first pose is: "<<first_pose_[0]<<", "<<first_pose_[1]<<", "<<first_pose_[2]<<std::endl;
    pose_frame_ = pose_frame;
    
    pose_pub_ = n_->advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_topic, 1000);
    // this->pose_publishing_thread_ = std::thread([this]{this->posePublishingThread();});
}

PosePublisherNode::~PosePublisherNode() {}

// Publishes pose and path
void PosePublisherNode::posePublish(const husky_inekf::HuskyState& state_) {

    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.seq = pose_seq_;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = pose_frame_;
    pose_msg.pose.pose.position.x = state_.x() - first_pose_[0];
    pose_msg.pose.pose.position.y = state_.y() - first_pose_[1];
    pose_msg.pose.pose.position.z = state_.z() - first_pose_[2];
    pose_msg.pose.pose.orientation.w = state_.getQuaternion().w();
    pose_msg.pose.pose.orientation.x = state_.getQuaternion().x();
    pose_msg.pose.pose.orientation.y = state_.getQuaternion().y();  
    pose_msg.pose.pose.orientation.z = state_.getQuaternion().z();
    // std::cout<<"publishing: "<<pose_msg.pose.pose.position.x<<", "<<pose_msg.pose.pose.position.y<<", "<<pose_msg.pose.pose.position.z<<std::endl;
    pose_pub_.publish(pose_msg);
    
    pose_seq_++;
    
    
    // geometry_msgs::PoseStamped pose_stamped;
    // pose_stamped.header = pose_msg.header;
    // pose_stamped.pose = pose_msg.pose.pose;
    // std::lock_guard<std::mutex> lock(poses_mutex_);
    // poses_.push_back(pose_stamped);
}

// Publishes path
void PosePublisherNode::pathPublish() {
    std::lock_guard<std::mutex> lock(poses_mutex_);

    nav_msgs::Path path_msg;
    path_msg.header.seq = path_seq_;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = pose_frame_;
    path_msg.poses = poses_;
    // std::cout<<"publishing current path: "<<path_msg.poses.back().pose.position.x<<", "<<path_msg.poses.back().pose.position.y<<", "<<path_msg.poses.back().pose.position.z<<std::endl;
    
    path_pub_.publish(path_msg);
    path_seq_++;
}


// Pose message callback
// void PosePublisherNode::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
//     if ((int)msg->header.seq%pose_skip_!=0) { return; }
//     geometry_msgs::PoseStamped pose;
//     pose.header = msg->header;
//     pose.pose = msg->pose.pose;
//     std::lock_guard<std::mutex> lock(poses_mutex_);
//     poses_.push_back(pose);
// }


// Path publishing thread
// void PosePublisherNode::posePublishingThread(){
//     // Loop and publish data
//     ros::Rate loop_rate(publish_rate_);
//     while(ros::ok()){
//         posePublish();
//         loop_rate.sleep();
//     }
// }

// int main(int argc, char **argv) {
//     // Initialize ROS node
//     ros::init(argc, argv, "pose_publisher");  
//     ros::NodeHandle n;
//     PosePublisherNode pose_publisher_node(n);
//     ros::spin();
//     return 0;
// }
