#include "kalman_filter.h"
#include <iostream>
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;


/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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
   x_ = F_*x_; //section 8 in lesson 5
   MatrixXd Ft = F_.transpose(); // section 9 in lesson 5
   P_ = F_ *P_ *Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {

  // Section 7 of lesson 5
  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;
  MatrixXd I = MatrixXd(4,4);
  I << 1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1;
  //new state
  x_ = x_ + (K * y);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * //TODO: update the state by using Extended Kalman Filter equations
   */
  // Section 14 of lesson 5
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  float rho = sqrt(px*px + py*py);
  float phi = atan2(py, px);
  float rho_dot = (px*vx + py*vy)/rho;
  VectorXd z_pred = VectorXd(3);
  z_pred << rho, phi, rho_dot;

  VectorXd y = z - z_pred;

  // Wraps angle if it's greater than PI or -PI
  if (y(1)>M_PI){
    y(1) = fmod(y(1),M_PI);
  }
  else if(y(1)<-M_PI){
    y(1) = fmod(y(1),-M_PI);
  }

  //Section 7 of lesson 5
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;
  MatrixXd I = MatrixXd(4,4);
  I << 1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1;
  //new state
  x_ = x_ + (K * y);
  P_ = (I - K * H_) * P_;
}
