#include "utils/utils.hpp"

// Clamp a to be within lim1 and lim2
double clamp(double a, double lim1, double lim2){
    // Find which limit is min and max
	double a_min = std::min(lim1, lim2);
	double a_max = std::max(lim1, lim2);

	// Clamp value between limits
	return std::max(std::min(a, a_max), a_min);
}

// Scales input to be between (0 to 1) based on input limits
double scaleFactor(double f, double tl, double tu){
    return (clamp(f, tl, tu) - tl)/(tu - tl);
}

// Converts a rotation matrix to Euler angles (RzRyRx) convention
Eigen::Vector3d Rotation2Euler(const Eigen::Matrix3d& R) {
/* Extract Euler angles in (RzRyRx) convention
	https://www.geometrictools.com/Documentation/EulerAngles.pdf
	Author:   Ross Hartley
	Date:     06/23/2017
*/
	Eigen::Vector3d q;
	// Singularity at qy = +/- pi/2
	double qx, qy, qz;
	if (R(2,0) < 1) {
		if (R(2,0) > -1) {
			// Unique solution
			qx = std::atan2(R(2,1),R(2,2));
			qy = std::asin(-R(2,0));
			qz = std::atan2(R(1,0),R(0,0));
		} else {
			// Not a unique solution
			qx = 0;
			qy = M_PI/2.0;
			qz = -std::atan2(-R(1,2),R(1,1));
		}
	} else {
		// Not a unique solution
		qx = 0;
		qy = -M_PI/2.0;
		qz = atan2(-R(1,2),R(1,1));
	}
	q << qz, qy, qx;
	return q;
}

// Convert from euler angles to rotation matrix using the ZYX convention 
Eigen::Matrix3d Euler2Rotation(const Eigen::Vector3d& euler) {
	Eigen::AngleAxisd Rz(euler(0), Eigen::Vector3d::UnitZ());
	Eigen::AngleAxisd Ry(euler(1), Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd Rx(euler(2), Eigen::Vector3d::UnitX());
	Eigen::Quaternion<double> q = Rz*Ry*Rx;
	return q.toRotationMatrix();
}

// Convert from to euler rates to angular velocity using the ZYX convention 
Eigen::Vector3d EulerRates2AngularVelocity(const Eigen::Vector3d& euler, const Eigen::Vector3d& euler_rates) {
	Eigen::Vector3d angularVelocity;
	double dqx = euler_rates(2);
	double dqy = euler_rates(1);
	double dqz = euler_rates(0);
	double qx = euler(2);
	double qy = euler(1);
	double t2 = sin(qx);
	double t3 = cos(qx);
	double t4 = cos(qy);
	angularVelocity << dqx-dqz*sin(qy), dqy*t3+dqz*t2*t4, -dqy*t2+dqz*t3*t4;
	return angularVelocity;
}

// Convert from angular velocity to euler rates using the ZYX convention 
Eigen::Vector3d  AngularVelocity2EulerRates(const Eigen::Vector3d& euler, const Eigen::Vector3d& w) {
	Eigen::Vector3d eulerRates;
    double t2 = cos(euler(2));
    double t3 = sin(euler(2));
    double t4 = cos(euler(1));
    double t5 = 1.0/t4;
    double t6 = sin(euler(1));
    eulerRates << t5*(t3*w(1)+t2*w(2)), t2*w(1)-t3*w(2), t5*(t4*w(0)+t3*t6*w(1)+t2*t6*w(2));
    return eulerRates;
}

// Computes skew-symmetric matrix from Vector3d
Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
	Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
	A(0,1) = -v(2);
	A(1,0) = v(2);
	A(0,2) = v(1);
	A(2,0) = -v(1);
	A(1,2) = -v(0);
	A(2,1) = v(0);
	return A;
}