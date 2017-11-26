#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.
const float PI2 = 2 * M_PI;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}


void KalmanFilter::Predict() {

	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const Eigen::VectorXd &z) {
	/**
  TODO:
	 * update the state by using Kalman Filter equations
	 */
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const Eigen::VectorXd &z) {
	/**
  TODO:
	 * update the state by using Extended Kalman Filter equations
	 */
	VectorXd z_pred = ProcessRM(x_);
	VectorXd y = z - z_pred;

	/*Normalize phi in y vector*/
	// normalize the angle between -pi to pi
	while(y(1) > M_PI){
		y(1) -= PI2;
	}

	while(y(1) < -M_PI){
		y(1) += PI2;
	}

	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

VectorXd  KalmanFilter::ProcessRM(const Eigen::VectorXd& x){

	const float px = x[0];
	const float py = x[1];
	const float vx = x[2];
	const float vy = x[3];

	const float rho = sqrt(px * px + py * py);
	const float phi = atan2(py, px);

	 if(rho < 0.000001)
	    rho = 0.000001;

	const float rho_dot = (px * vx + py * vy) / rho;
	VectorXd result(3);
	result << rho, phi, rho_dot;
	return result;
}
