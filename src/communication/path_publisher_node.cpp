/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   pose_to_path_node.cpp
 *  @author Ross Hartley
 *  @brief  Subscribes to pose and publish a path
 *  @date   March 20, 2019
 **/

#include "ros/ros.h"
#include <string>
#include <mutex>
#include <fstream>
#include <thread>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <nav_msgs/Path.h>

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

class PathPublisherNode {
    public:
        PathPublisherNode(ros::NodeHandle n) : n_(n) {
            // Create private node handle
            ros::NodeHandle nh("~");
            std::string pose_topic, path_topic;
            nh.param<std::string>("/settings/pose_topic", pose_topic, "/husky/inekf_estimation/pose");
            nh.param<std::string>("/settings/path_topic", path_topic, "/husky/inekf_estimation/path");
            nh.param<double>("/settings/publish_rate", publish_rate_, 1000); 
            nh.param<int>("/settings/pose_skip", pose_skip_, 100); 

            std::cout<<"pose_topic: "<<pose_topic<<", path_topic: "<<path_topic<<std::endl;
            std::cout<<"path publish rate: "<<publish_rate_<<std::endl;

            // Find pose frame from first message
            geometry_msgs::PoseWithCovarianceStampedConstPtr pose_msg = 
                ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>(pose_topic, n_);
            pose_frame_ = pose_msg->header.frame_id;
            // Subscribe and advertise
            pose_sub_ = n_.subscribe(pose_topic, 1000, &PathPublisherNode::poseCallback, this);
            path_pub_ = n_.advertise<nav_msgs::Path>(path_topic, 10);

            // if (file_name_.size() > 1){
            //   std::ofstream outfile(file_name_);
            //   std::ofstream tum_outfile(tum_file_name_);
            //   ROS_INFO_STREAM("write traj to file "<<file_name_);
            // tum_outfile.close();
            // outfile.close();
            // }
            
            this->path_publishing_thread_ = std::thread([this]{this->pathPublishingThread();});
        }

    private:
        ros::NodeHandle n_;
        ros::Subscriber pose_sub_;
        ros::Publisher path_pub_;
        std::string pose_frame_;
        std::string file_name_;
        std::string tum_file_name_;
        uint32_t seq_ = 0;
        double publish_rate_;
        int pose_skip_;
        std::vector<geometry_msgs::PoseStamped> poses_;
        std::mutex poses_mutex_;
        std::thread path_publishing_thread_;

        // Pose message callback
        void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){

            if ((int)msg->header.seq%pose_skip_!=0) { return; }
            geometry_msgs::PoseStamped pose;
            pose.header = msg->header;
            pose.pose = msg->pose.pose;
            // std::cout<<"subscribing to pose: "<<pose.pose.position.x<<", "<<pose.pose.position.y<<", "<<pose.pose.position.z<<std::endl;
            
            std::lock_guard<std::mutex> lock(poses_mutex_);
            
            poses_.push_back(pose);
        }

        void poseStampedCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
            // if ((int)msg->header.seq%pose_skip_!=0) { return; }
            geometry_msgs::PoseStamped pose;
            pose.header = msg->header;
            pose.pose = msg->pose;
            // std::cout<<"publishing: "<<pose.pose.position.x<<", "<<pose.pose.position.y<<", "<<pose.pose.position.z<<std::endl;
            std::lock_guard<std::mutex> lock(poses_mutex_);
            poses_.push_back(pose);
        }


        // Publishes path
        void pathPublish() {
            std::lock_guard<std::mutex> lock(poses_mutex_);
            nav_msgs::Path path_msg;
            path_msg.header.seq = seq_;
            path_msg.header.stamp = ros::Time::now();
            path_msg.header.frame_id = pose_frame_;
            path_msg.poses = poses_;
            // std::cout<<"publishing current path: "<<path_msg.poses.back().pose.position.x<<", "<<path_msg.poses.back().pose.position.y<<", "<<path_msg.poses.back().pose.position.z<<std::endl;
            
            path_pub_.publish(path_msg);
            seq_++;
        }

        // Path publishing thread
        void pathPublishingThread(){
            // Loop and publish data
            ros::Rate loop_rate(publish_rate_);
            while(ros::ok()){
                pathPublish();
                loop_rate.sleep();
            }
        }
};


int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "path_publisher");  
    ros::NodeHandle n;
    PathPublisherNode path_publisher_node(n);
    ros::spin();
    return 0;
}
