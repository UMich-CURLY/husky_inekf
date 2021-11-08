#ifndef UTILS_H
#define UTILS_H

#define _USE_MATH_DEFINES
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <algorithm>

// Clamp a to be within lim1 and lim2
double clamp(double a, double lim1, double lim2);
// Scales input to be between (0 to 1) based on input limits
double scaleFactor(double f, double tl, double tu);
// Converts a rotation matrix to Euler angles (RzRyRx) convention
Eigen::Vector3d Rotation2Euler(const Eigen::Matrix3d& R);
// Computes an iterative inverse kinematics solution
// Eigen::VectorXd IterativeInverseKinematics(std::function<Eigen::VectorXd(const Eigen::VectorXd&)> FK_func, std::function<Eigen::MatrixXd(const Eigen::VectorXd&)> J_FK_func, const Eigen::VectorXd& x_target, const Eigen::VectorXd& theta_init);
// Computes an iterative inverse kinematics solution with holonomic constraints
// Eigen::VectorXd IterativeConstrainedInverseKinematics(std::function<Eigen::VectorXd(const Eigen::VectorXd&)> y_func, std::function<Eigen::MatrixXd(const Eigen::VectorXd&)> J_y_func, std::function<Eigen::VectorXd(const Eigen::VectorXd&)> hol_func, std::function<Eigen::MatrixXd(const Eigen::VectorXd&)> J_hol_func, const Eigen::VectorXd& y_target, const Eigen::VectorXd& q_init);
// Convert from euler angles to rotation matrix using the ZYX convention 
Eigen::Matrix3d Euler2Rotation(const Eigen::Vector3d& euler);
// Convert from angular velocity to euler rates using the ZYX convention 
Eigen::Vector3d AngularVelocity2EulerRates(const Eigen::Vector3d& euler, const Eigen::Vector3d& w);
// Convert from to euler rates to angular velocity using the ZYX convention 
Eigen::Vector3d EulerRates2AngularVelocity(const Eigen::Vector3d& euler, const Eigen::Vector3d& euler_rates);
// Computes skew-symmetric matrix from Vector3d
Eigen::Matrix3d skew(const Eigen::Vector3d& v);

#endif // UTILS_H