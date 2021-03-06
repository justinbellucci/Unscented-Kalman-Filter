#include "ukf.h"
#include <Eigen/Dense>
#include <iostream>


UKF::UKF() {

  // if this is false, laser measurements will be ignored (except during init)
  use_lidar_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // is initialized to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;

  // initial state vector
  x_ = Eigen::VectorXd(5);

  // initial covariance matrix
  P_ = Eigen::MatrixXd(5, 5);
  P_  << 1, 0, 0, 0, 0,
         0, 1, 0, 0, 0,
         0, 0, 1, 0, 0,
         0, 0, 0, 0.05, 0,
         0, 0, 0, 0, 0.05;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  // std_a_ = 0.9; 
  std_a_ = 5.0;

  // Process noise standard deviation yaw acceleration in rad/s^2
  // std_yawdd_ = 0.4;
  std_yawdd_ = 1.7;

  ///////////////////////////////////////////////////////////////
  /////////////// Provided by sensor manufacturer ///////////////

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
  
  ///////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////

  n_x_ = 5; // state vector dimension
  n_aug_ = n_x_ + 2; // augmented state vector dimension
  lambda_ = 3 - n_aug_; // spreading parameter

  weights_ = Eigen::VectorXd(2 * n_aug_ + 1); 
  weights_(0) = lambda_ / (lambda_ + n_aug_); 
  double weight_n = 0.5 / (lambda_ + n_aug_); 
  for (int i = 1; i < 2 * n_aug_ + 1; i++) {
    weights_(i) = weight_n;
  }
  
  // predicted sigma points matrix
  X_sig_pred_ = Eigen::MatrixXd(n_x_, 2 * n_aug_ + 1);

  // measurement noise covariance matrix for radar
  R_radar_ << std_radr_ * std_radr_, 0, 0,
              0, std_radphi_ * std_radphi_, 0,
              0, 0, std_radrd_ * std_radrd_;

  // measurement noise covariance matrix for lidar
  R_lidar_ << std_laspx_ * std_laspx_, 0,
              0, std_laspy_ * std_laspy_;

  // Initialize NIS values
  NIS_radar_ = 0;
  NIS_radar_filename_ = "NIS_radar_log.txt";
  NIS_lidar_ = 0;
  NIS_lidar_filename_ = "NIS_lidar_log.txt";
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage& meas_package) {
  
  if (!is_initialized_){
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
      px  = meas_package.raw_measurements_[0]; 
      py = meas_package.raw_measurements_[1]; 
      v = 0; // velocity is 0 because we don't know it yet
    }
    
    x_ << px, py, v, yaw, yay_rate;

    is_initialized_ = true;
    time_us_ = meas_package.timestamp_; 
    // std::cout << "Initialization complete" << std::endl;
    return; 
  }

  // compute delta time
  double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_; 

  Prediction(dt);
  
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
    // update state with radar measurement
    n_z_ = 3; // number of radar measurements
    Z_sig_ = Eigen::MatrixXd::Zero(n_z_, 2 * n_aug_ + 1); 
    z_pred_ = Eigen::VectorXd::Zero(n_z_); 
    S_ = Eigen::MatrixXd::Zero(n_z_, n_z_); 
    UpdateRadar(meas_package);
  } else  if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_lidar_) {
    // update state with laser measurement
    n_z_ = 2; 
    Z_sig_ = Eigen::MatrixXd::Zero(n_z_, 2 * n_aug_ + 1); 
    z_pred_ = Eigen::VectorXd::Zero(n_z_); 
    S_ = Eigen::MatrixXd::Zero(n_z_, n_z_); 
    UpdateLidar(meas_package);
  }
} // end ProcessMeasurement

void UKF::Prediction(double& delta_t) {

  //////////////////////////////////////////////////////////////
  /////////////// Create Augmented Sigma Points ////////////////

  // create augmented mean state vector
  Eigen::VectorXd x_aug = Eigen::VectorXd::Zero(n_aug_); 
  x_aug.head(5) = x_; 
  x_aug(5) = 0; // longitudinal acceleration noise
  x_aug(6) = 0; // yaw acceleration noise

  // create augmented covariance matrix
  Eigen::MatrixXd P_aug = Eigen::MatrixXd::Zero(n_aug_, n_aug_); 
  P_aug.topLeftCorner(n_x_, n_x_) = P_; 
  P_aug(5, 5) = std_a_ * std_a_; // longitudinal acceleration noise
  P_aug(6, 6) = std_yawdd_ * std_yawdd_; // yaw angle acceleration noise

  // create augmented sigma points matrix
  Eigen::MatrixXd Xsig_aug = Eigen::MatrixXd(n_aug_, 2 * n_aug_ + 1); 
  Eigen::MatrixXd sqrt_P_aug = P_aug.llt().matrixL(); 
  double sqrt_lambda = sqrt(lambda_ + n_aug_); 
  Xsig_aug.col(0) = x_aug; // first column is the mean state
  for (int i = 0; i < n_aug_; i++) {
    Xsig_aug.col(i + 1) = x_aug + sqrt_lambda * sqrt_P_aug.col(i); 
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt_lambda * sqrt_P_aug.col(i);
  }

  //////////////////////////////////////////////////////////////
  ////////////////////// Predict Sigma Points //////////////////

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
      py_p = py + v / yaw_accel * (cos(yaw) - cos(yaw + yaw_accel * delta_t));
    } else {
      px_p = px + v * cos(yaw) * delta_t;
      py_p = py + v * sin(yaw) * delta_t;
    }

    double v_p = v;
    double yaw_p = yaw + yaw_accel * delta_t;
    double yaw_accel_p = yaw_accel; 

    // add acceleration noise to predicted state
    px_p = px_p + 0.5 * delta_t * delta_t * cos(yaw) * nu_accel; 
    py_p = py_p + 0.5 * delta_t * delta_t * sin(yaw) * nu_accel; 
    v_p = v_p + nu_accel * delta_t; 

    // add yaw acceleration noise to predicted state
    yaw_p = yaw_p + 0.5 * delta_t * delta_t * nu_yaw_accel; 
    yaw_accel_p = yaw_accel_p + delta_t * nu_yaw_accel; 

    // write predicted sigma point into right column
    X_sig_pred_(0, i) = px_p;
    X_sig_pred_(1, i) = py_p;
    X_sig_pred_(2, i) = v_p;
    X_sig_pred_(3, i) = yaw_p;
    X_sig_pred_(4, i) = yaw_accel_p;
  }

  //////////////////////////////////////////////////////////////
  //////////////// Predict Mean and Covariance /////////////////

  // predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    x_ = x_ + weights_(i) * X_sig_pred_.col(i);
  }

  // predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    // residual
    Eigen::VectorXd x_diff = X_sig_pred_.col(i) - x_; 
    NormalizeAngle(&x_diff(3)); 

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }
} // end of Prediction function

void UKF::UpdateLidar(MeasurementPackage& meas_package) {

  //////////////////////////////////////////////////////////////
  ////////////////////// Predict Measurement ///////////////////

  // predicted sigma points for measurement space
  Z_sig_ = X_sig_pred_.block(0, 0, n_z_, 2 * n_aug_ + 1); 
  
  // mean predicted measurement
  z_pred_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    z_pred_ = z_pred_ + weights_(i) * Z_sig_.col(i);
  }

  // predicted measurement covariance matrix S
  S_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    // residual
    Eigen::VectorXd z_diff = Z_sig_.col(i) - z_pred_;

    S_ += weights_(i) * z_diff * z_diff.transpose();
  }
  S_ = S_ + R_lidar_;

  //////////////////////////////////////////////////////////////
  ////////////////////// Update State //////////////////////////

  Eigen::VectorXd z = meas_package.raw_measurements_;

  // calculate cross correlation Tc
  Eigen::MatrixXd Tc_lidar_ = Eigen::MatrixXd::Zero(n_x_, n_z_);

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    // residual
    Eigen::VectorXd z_diff = Z_sig_.col(i) - z_pred_;
    // state difference
    Eigen::VectorXd x_diff = X_sig_pred_.col(i) - x_;

    Tc_lidar_ += weights_(i) * x_diff * z_diff.transpose();
  }

  // calculate Kalman gain K
  Eigen::MatrixXd K = Tc_lidar_ * S_.inverse();

  // update state mean and covariance matrix
  Eigen::VectorXd z_diff = z - z_pred_;

  x_ = x_ + K * z_diff;
  P_ = P_ - K * S_ * K.transpose();

  // NIS
  NIS_lidar_ = z_diff.transpose() * S_.inverse() * z_diff;
  RecordNIS(NIS_lidar_, NIS_lidar_filename_);

} // end of UpdateLidar function


void UKF::UpdateRadar(MeasurementPackage &meas_package) {
  
  //////////////////////////////////////////////////////////////
  ////////////////////// Predict Measurement ///////////////////

  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    // extract values for better readability
    double p_x = X_sig_pred_(0, i);
    double p_y = X_sig_pred_(1, i);
    double v = X_sig_pred_(2, i);
    double yaw = X_sig_pred_(3, i);

    double v1 = cos(yaw) * v;
    double v2 = sin(yaw) * v;

    // measurement model
    Z_sig_(0, i) = sqrt(p_x * p_x + p_y * p_y);     // rho in m
    Z_sig_(1, i) = atan2(p_y, p_x);                 // phi in rad

    // avoid division by zero
    if (Z_sig_(0, i) < 0.001) {
      Z_sig_(2, i) = (p_x * v1 + p_y * v2) / 0.001; // rho_dot in m/s
    } else {
      Z_sig_(2, i) = (p_x * v1 + p_y * v2) / Z_sig_(0, i);
    }
  }

  // mean predicted measurement
  z_pred_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    z_pred_ = z_pred_ + weights_(i) * Z_sig_.col(i);
  }

  // predicted measurement covariance matrix S
  S_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    // residual
    Eigen::VectorXd z_diff = Z_sig_.col(i) - z_pred_;
    NormalizeAngle(&z_diff(1)); 

    S_ += weights_(i) * z_diff * z_diff.transpose();
  }

  S_ = S_ + R_radar_;

  //////////////////////////////////////////////////////////////
  ////////////////////// Update State //////////////////////////

  Eigen::Vector3d z = meas_package.raw_measurements_;

  // calculate cross correlation Tc
  Eigen::MatrixXd Tc_radar = Eigen::MatrixXd::Zero(n_x_, n_z_);

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    // residual
    Eigen::VectorXd z_diff = Z_sig_.col(i) - z_pred_;
    NormalizeAngle(&z_diff(1));

    // state difference
    Eigen::VectorXd x_diff = X_sig_pred_.col(i) - x_;
    NormalizeAngle(&x_diff(3));

    Tc_radar += weights_(i) * x_diff * z_diff.transpose();
  }

  // calculate Kalman gain K
  Eigen::MatrixXd K = Tc_radar * S_.inverse();

  // update state mean and covariance matrix
  Eigen::VectorXd z_diff = z - z_pred_;
  NormalizeAngle(&z_diff(1)); 

  x_ = x_ + K * z_diff;
  P_ = P_ - K * S_ * K.transpose();

  // NIS
  NIS_radar_ = z_diff.transpose() * S_.inverse() * z_diff;
  RecordNIS(NIS_radar_, NIS_radar_filename_);
  
} // end of UpdateRadar function

void UKF::NormalizeAngle(double* angle) {
  while (*angle > M_PI) 
    *angle -= 2.0 * M_PI;
  while (*angle < -M_PI) 
    *angle += 2.0 * M_PI;
}

void UKF::RecordNIS(double& NIS, std::string& filename) {
  std::ofstream log;
  log.open(filename, std::ios_base::app);
  log << NIS << std::endl;
}