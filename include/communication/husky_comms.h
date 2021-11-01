#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <string>
#include <mutex>
#include <thread>

class HuskyComms {
    public:
        HuskyComms(ros::NodeHandle n);

    private:
        void imuCallback(const sensor_msgs::Imu& msg);

        void odomCallback(const nav_msgs::Odometry& msg);
        
        ros::NodeHandle n_;
        message_filters::Subscriber<sensor_msgs::Imu> imu_sub_;
        message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, 
                                                    nav_msgs::Odometry> TimeSyncPolicy;
        typedef message_filters::Synchronizer<TimeSyncPolicy> Sync;
        boost::shared_ptr<Sync> sync_;
    
};