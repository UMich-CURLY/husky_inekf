#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "utils/husky_data.hpp"

#include <string>

class HuskyComms {
    public:
        HuskyComms(ros::NodeHandle nh, husky_inekf::husky_data_t* husky_data_buffer);

    private:
        void imuCallback(const sensor_msgs::Imu& imu_msg);
        void odomCallback(const nav_msgs::Odometry& odom_msg);
        
        ros::NodeHandle nh_;
        ros::Subscriber imu_sub_;
        ros::Subscriber odom_sub_;

        husky_inekf::husky_data_t* husky_data_buffer_;
};