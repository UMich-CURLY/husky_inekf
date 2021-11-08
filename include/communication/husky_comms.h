/* ROS interfaces */
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"

/* Husky Data Structures For Internal Program */
#include "utils/husky_data.hpp"
#include "utils/imu.hpp"
#include "utils/joint_state.hpp"

#include <string>

class HuskyComms {
    public:
        HuskyComms(ros::NodeHandle nh, husky_inekf_data::husky_data_t* husky_data_buffer);

    private:
        void imuCallback(const sensor_msgs::Imu& imu_msg);
        void jointStateCallback(const sensor_msgs::JointState& joint_msg);
        
        ros::NodeHandle nh_;
        ros::Subscriber imu_sub_;
        ros::Subscriber joint_sub_;

        husky_inekf_data::husky_data_t* husky_data_buffer_;
};