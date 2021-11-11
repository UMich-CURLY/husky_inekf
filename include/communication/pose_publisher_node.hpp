#include "ros/ros.h"
#include <string>
#include <sstream> 
#include <fstream>
#include <array>
#include <vector>
#include <queue>
#include <mutex>
#include <thread>
#include "system/husky_state.hpp"
#include <Eigen/Dense>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <nav_msgs/Path.h>

class PosePublisherNode{
    public:
        PosePublisherNode(ros::NodeHandle* n);
        ~PosePublisherNode();

        // Publishes pose
        void posePublish(const husky_inekf::HuskyState& state_);

    private:
        ros::NodeHandle* n_;
        ros::Publisher pose_pub_;
        std::string pose_frame_;
        uint32_t seq_ = 0;
        double publish_rate_;
        int pose_skip_;
        std::queue<std::array<float,3>> pose_from_csv_;
        std::array<float,3> first_pose_;
        std::vector<geometry_msgs::PoseStamped> poses_;
        std::mutex poses_mutex_;
        std::thread pose_publishing_thread_;
};

