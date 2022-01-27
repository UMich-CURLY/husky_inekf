/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   Measurement.h
 *  @author Ross Hartley
 *  @brief  Header file for Measurement class
 *  @date   September 27, 2018
 **/

#ifndef MEASUREMENT_H
#define MEASUREMENT_H 
#include <Eigen/Dense>
#include <string>
#include "ros/ros.h"
// #include "inekf_msgs/ContactArray.h"
// #include "inekf_msgs/KinematicsArray.h"
// #include "inekf_msgs/LandmarkArray.h"
#include "std_msgs/Header.h"
#include "core/InEKF.h"
#include "tf/transform_listener.h"

enum MeasurementType {
                      EMPTY, 
                      IMU,
                      JOINT_STATE, 
                      GPS_VELOCITY, 
                      CAMERA_ODOM,
                      VELOCITY 
                    };

class Measurement {
    struct MeasurementHeader {
        uint64_t seq;
        double stamp;
        std::string frame_id;
        
        MeasurementHeader() {
        }

        MeasurementHeader(const std_msgs::Header& header_in) {
            seq = (uint64_t) header_in.seq;
            stamp = header_in.stamp.sec + header_in.stamp.nsec / 1000000000.0;
            frame_id = header_in.frame_id;
        }
    };

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Measurement();
        virtual ~Measurement() = default;

        MeasurementHeader header;
        void setHeader(const std_msgs::Header& header_in);


        double getTime() const;
        MeasurementType getType();

        friend std::ostream& operator<<(std::ostream& os, const Measurement& m);  

    protected:
        MeasurementType type_;
};

struct MeasurementCompare {
  bool operator()(Measurement& lhs, Measurement& rhs) const {
    return lhs.getTime() > rhs.getTime();
  }
};

#endif 
