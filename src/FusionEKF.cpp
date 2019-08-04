#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0.0,
              0.0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0.0, 0.0,
              0.0, 0.0009, 0.0,
              0.0, 0.0, 0.09;

  //Laser measurement matrix
  H_laser_ << 1.0, 0, 0, 0,
        0, 1.0, 0, 0;
  // TODO Finish F_,P_
  //4x4 state transition matrix
  ekf_.F_ = MatrixXd(4,4);

  // 4x4 Covariance matrix
  ekf_.P_ = MatrixXd(4,4);

  ekf_.Q_ = MatrixXd(4,4);

    // TODO Finish noise
  // Acceleration noise components
  noise_ax = 9.0;
  noise_ay = 9.0;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */


    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4); //
    ekf_.x_(2) = 5.199937e+00;
    ekf_.x_(3) = 0;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float rho_dot = measurement_pack.raw_measurements_[2];
      ekf_.x_(0) = rho * cos(phi);
      ekf_.x_(1) = rho * sin(phi);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      ekf_.x_(0) = measurement_pack.raw_measurements_[0];
      ekf_.x_(1) = measurement_pack.raw_measurements_[1];
    }

    ekf_.F_ << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;
    previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
   float dt = (measurement_pack.timestamp_/ 1.0e6 - previous_timestamp_/ 1.0e6); //dt in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;


  // Section 8 0f lesson 5: F matrix uses dt
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // lesson 10: Set the process covariance matrix Q
  ekf_.Q_ << dt_4/4.0*noise_ax, 0, dt_3/2.0*noise_ax, 0,
            0, dt_4/4.0*noise_ay, 0, dt_3/2.0*noise_ay,
            dt_3/2.0*noise_ax, 0, dt_2*noise_ax, 0,
            0, dt_3/2.0*noise_ay, 0, dt_2*noise_ay;

  ekf_.Predict();
  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar HJ_ using tools
    Tools tools;
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    //Update H Radar
//    cout << "Update H_Radar = " << ekf_.H_ << endl;
  } else {
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
