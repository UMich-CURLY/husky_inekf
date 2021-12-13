/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   Measurement.cpp
 *  @author Ross Hartley
 *  @brief  Source file for Measurement class
 *  @date   September 27, 2018
 **/

#include "utils/measurement.h"
using namespace std;
using namespace inekf;

// Construct Empty Measurement
Measurement::Measurement() {
    header.stamp = 0;
    type_ = EMPTY;
}

void Measurement::setHeader(const std_msgs::Header& header_in){
          header.seq = (uint64_t) header_in.seq;
          header.stamp = header_in.stamp.sec + header_in.stamp.nsec / 1000000000.0;
          header.frame_id = header_in.frame_id;
};

// Getters
double Measurement::getTime() const {   
    return header.stamp; 
}
MeasurementType Measurement::getType() { return type_; }

// Print measurement
ostream& operator<<(ostream& os, const Measurement& m) {
    string type_str;
    switch (m.type_) {
        case IMU :
            type_str = "IMU";
            break;
        default:
            type_str = "Unknown";
    }
    os << "Measurement type: " << type_str << endl;
    return os;
}
