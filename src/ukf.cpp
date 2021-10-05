#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // is initialized to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;

  // initial state vector
  x_ = Eigen::VectorXd(5);

  // initial covariance matrix
  P_ = Eigen::MatrixXd(5, 5);

  // TODO: Set the noise values
  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30; // rule of thumb divided expected max acceleration by 2

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;
  
  ////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////
  // Provided by sensor manufacturer
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  // end provided by sensor manufacturer
  ////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////

  n_x_ = x_.size(); // get size of state vector
  n_aug_ = n_x_ + 2; // get size of augmented state vector
  lambda_ = 3 - n_aug_; // define spreading parameter
  // initialize params for predicting mean and covariance with augmented state vector
  // TODO: make more efficient
  weights_ = Eigen::VectorXd(2 * n_aug_ + 1); // define weights vector size
  weights_(0) = lambda_ / (lambda_ + n_aug_); 
  double weight_n = 0.5 / (lambda_ + n_aug_); // save some computation time
  for (int i = 1; i < 2 * n_aug_ + 1; i++) {
    weights_(i) = weight_n;
  }
  
  // initialize augmented state vector
  Xsig_pred_ = Eigen::MatrixXd(n_x_, 2 * n_aug_ + 1);

  // initialize measurement noise covariance matrix for radar
  R_radar_ = Eigen::MatrixXd(3, 3);
  R_radar_ << std_radr_ * std_radr_, 0, 0,
              0, std_radphi_ * std_radphi_, 0,
              0, 0, std_radrd_ * std_radrd_;

  // initialize measurement noise covariance matrix for laser
  R_laser_ = Eigen::MatrixXd(2, 2);
  R_laser_ << std_laspx_ * std_laspx_, 0,
              0, std_laspy_ * std_laspy_;

}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  
  // if not initialized, initialize
  if (!is_initialized_){
    P_ = Eigen::MatrixXd::Identity(n_x_, n_x_); // initialize state covariance matrix
    double px;  // initial state x position
    double py;  // initial state y position  
    double v;  // initial state velocity 
    double yaw = 0;  // initial state yaw angle
    double yay_rate = 0;  // initial state yaw angle rate

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      // convert radar from polar to cartesian coordinates
      double rho = meas_package.raw_measurements_[0]; // range
      double phi = meas_package.raw_measurements_[1]; // bearing angle
      double rho_dot = meas_package.raw_measurements_[2]; // range rate
      double px = rho * cos(phi);
      double py = rho * sin(phi);
      double vx = rho_dot * cos(phi);
      double vy = rho_dot * sin(phi);
      double v = sqrt(vx * vx + vy * vy);

    } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      // initialize state with laser measurement
      px  = meas_package.raw_measurements_[0]; // x position
      py = meas_package.raw_measurements_[1]; // y position
      v = 0; // velocity is 0 because we don't know it yet
    }
    // insert variables into state vector
    x_ << px, py, v, yaw, yay_rate;
    // set initialized to true
    is_initialized_ = true;
    time_us_ = meas_package.timestamp_; //  measure time
    return; 

  } else { // initialized
    // compute delta time
    double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
    time_us_ = meas_package.timestamp_; //  measure time
    // predict state
    Prediction(dt);
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      // update state with radar measurement
      UpdateRadar(meas_package);
    } else  if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      // update state with laser measurement
      UpdateLidar(meas_package);
    }
  }
}

void UKF::Prediction(double delta_t) {

  // create augmented matrices
  // TODO: Move these to smart pointers
  Eigen::VectorXd x_aug = Eigen::VectorXd(n_aug_); // augmented state mean vector
  Eigen::MatrixXd P_aug = Eigen::MatrixXd(n_aug_, n_aug_); // augmented state covariance matrix
  Eigen::MatrixXd Xsig_aug = Eigen::MatrixXd(n_aug_, 2 * n_aug_ + 1); // augmented sigma points matrix

  // create augmented mean state vector
  x_aug.fill(0.0);
  x_aug.head(5) = x_; // first 5 elements fill with x_
  x_aug(5) = 0; // initial value for longitudinal acceleration noise
  x_aug(6) = 0; // initial value for yaw acceleration noise

  // create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_; // fill top left with P_
  P_aug(5, 5) = std_a_ * std_a_; // longitudinal acceleration noise
  P_aug(6, 6) = std_yawdd_ * std_yawdd_; // yaw angle acceleration noise

  // create augmented sigma points matrix
  Eigen::MatrixXd sqrt_P_aug = P_aug.llt().matrixL(); // square root of P_aug
  double sqrt_lambda = sqrt(lambda_ + n_aug_); 
  Xsig_aug.col(0) = x_aug; // first column is the mean state
  for (int i = 0; i < n_aug_; i++) {
    Xsig_aug.col(i + 1) = x_aug + sqrt_lambda * sqrt_P_aug.col(i); 
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt_lambda * sqrt_P_aug.col(i);
  }

  ////// Predict sigma points //////
  for (int i = 0; i < 2 * n_aug_ + 1; i++){
    // extract state values for better readability
    double px = Xsig_aug(0, i);
    double py = Xsig_aug(1, i);
    double v = Xsig_aug(2, i);
    double yaw = Xsig_aug(3, i);
    double yaw_accel = Xsig_aug(4, i);
    double nu_accel = Xsig_aug(5, i);
    double nu_yaw_accel = Xsig_aug(6, i);

    double px_p; // predicted x position
    double py_p; // predicted y position

    // avoid division by zero
    if (fabs(yaw_accel) > 0.001) {
      px_p = px + v / yaw_accel * (sin(yaw + yaw_accel * delta_t) - sin(yaw));
      py_p = py + v / yaw_accel * (-cos(yaw + yaw_accel * delta_t) + cos(yaw));
    } else {
      px_p = px + v * cos(yaw) * delta_t;
      py_p = py + v * sin(yaw) * delta_t;
    }

    // TODO: more efficient method?
    px_p = px_p + 0.5 * delta_t * delta_t * cos(yaw) * nu_accel; // add acceleration noise to predicted x position
    py_p = py_p + 0.5 * delta_t * delta_t * sin(yaw) * nu_accel; // add acceleration noise to predicted y position
    double v_p = v; 
    v_p = v_p + nu_accel * delta_t; // add acceleration noise to predicted velocity
    double yaw_p = yaw; 
    yaw_p = yaw_p + 0.5 * delta_t * delta_t * nu_yaw_accel; // add yaw acceleration noise to predicted yaw angle
    double yaw_accel_p = yaw_accel; 
    yaw_accel_p = yaw_accel_p + delta_t * nu_yaw_accel; // add yaw acceleration noise to predicted yaw acceleration

    // write predicted sigma point into right column
    Xsig_pred_(0, i) = px_p;
    Xsig_pred_(1, i) = py_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = yaw_p;
    Xsig_pred_(4, i) = yaw_accel_p;
  }

  //// predict mean and covariance /////
  // predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  // // predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    Eigen::VectorXd x_diff = Xsig_pred_.col(i) - x_; // state difference
    NormalizeAngle(&x_diff(3)); // pass yaw angle by reference
    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }
} // end of Prediction function

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
}

void UKF::NormalizeAngle(double *angle) {
  while (*angle > M_PI) 
    *angle -= 2.0 * M_PI;
  while (*angle < -M_PI) 
    *angle += 2.0 * M_PI;
}