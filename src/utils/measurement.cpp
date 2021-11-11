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
// Getters
double Measurement::getTime() {   
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
