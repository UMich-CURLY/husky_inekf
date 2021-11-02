
// STL
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <thread>
#include <chrono>
#include <fstream>
#include <string>
#include <memory>
#include <iostream>

// Internal Libraries
#include "utils/husky_data.hpp"
#include "communication/husky_comms.h"

// External Libraries
#include "ros/ros.h"
#include "InEKF.h"
// Boost
#include <boost/algorithm/string.hpp>
// Threading
#include <boost/thread/condition.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/circular_buffer.hpp>

int main(int argc, char **argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "husky_estimator");
    ros::NodeHandle nh;

    // Threading
    husky_inekf::husky_data_t husky_data_buffer;

    // Set noise parameters
    inekf::NoiseParams params;

    HuskyComms husky_comms(nh, &husky_data_buffer);

    // Initialize CheetahSystem
    // CheetahSystem *system = new CheetahSystem(&lcm, &nh, &cdata_mtx, &cheetah_input_data);
    // // system->setEstimator(std::make_shared<BodyEstimator>());

    // // //TODO: Listen/Respond Loop
    // bool received_data = true;
    // while (lcm.handle() == 0 && ros::ok())
    // {
    //     system->step();
    //     /// TODO: publish to ros

    //     ros::spinOnce();
    // }

    return 0;
}
