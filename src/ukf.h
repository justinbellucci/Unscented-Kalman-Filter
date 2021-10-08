#ifndef UKF_H
#define UKF_H

#include "Eigen/Dense"
#include "measurement_package.h"
#include <fstream>
#include <iostream>
#include <filesystem>
#include <string>
class UKF {
  public:
    
    // constructor
    UKF();

    // destructor
    virtual ~UKF();

    // ProcessMeasurement
    //  @param meas_package The latest measurement data of either radar or laser
    void ProcessMeasurement(MeasurementPackage &meas_package);

    // Predicts sigma points, the state, and the state covariance
    // matrix
    // @param delta_t Time between k and k+1 in s
    void Prediction(double &delta_t);

    // Updates the state and the state covariance matrix using a laser measurement
    // @param meas_package The measurement at k+1
    void UpdateLidar(MeasurementPackage &meas_package);

    // Updates the state and the state covariance matrix using a radar measurement
    // @param meas_package The measurement at k+1
    void UpdateRadar(MeasurementPackage &meas_package);

    // process first measurement if not already done
    bool is_initialized_;

    // if false, lidar measurements will be ignored (except for init)
    bool use_lidar_;

    // if false, radar measurements will be ignored (except for init)
    bool use_radar_;

    // state vector: [px py velocity yaw_angle yaw_rate] in SI units and rad
    Eigen::VectorXd x_;

    // state covariance matrix
    Eigen::MatrixXd P_;

    // predicted sigma points matrix
    Eigen::MatrixXd X_sig_pred_;

    // time when the state is true, in us
    long long time_us_;

    // Process noise standard deviation longitudinal acceleration in m/s^2
    double std_a_;

    // Process noise standard deviation yaw acceleration in rad/s^2
    double std_yawdd_;

    // Laser measurement noise standard deviation position1 in m
    double std_laspx_;

    // Laser measurement noise standard deviation position2 in m
    double std_laspy_;

    // Radar measurement noise standard deviation radius in m
    double std_radr_;

    // Radar measurement noise standard deviation angle in rad
    double std_radphi_;

    // Radar measurement noise standard deviation radius change in m/s
    double std_radrd_ ;

    // Weights of sigma points
    Eigen::VectorXd weights_;

    // State dimension
    int n_x_;

    // Augmented state dimension
    int n_aug_;

    // Sigma point spreading parameter
    double lambda_;

  private:
    
    // measurement noise covariance matrix - radar 
    Eigen::Matrix3d R_radar_;

    // measurement noise covariance matrix - lidar
    Eigen::Matrix2d R_lidar_;

    // measurement vector
    Eigen::Vector3d z_; 

    // predicted mesurement sigma points matrix
    Eigen::MatrixXd Z_sig_;

    // predicted mesurement mean
    Eigen::VectorXd z_pred_;

    // innovation covariance matrix S 
    Eigen::MatrixXd S_;

    // dimension of measurement vector
    int n_z_;

    // radar cross correlation matrix
    Eigen::MatrixXd Tc_radar_;

    // lidar cross correlation matrix
    Eigen::MatrixXd Tc_lidar_;

    // NIS for radar
    double NIS_radar_;
    std::string NIS_radar_filename_;
    
    // NIS for lidar
    double NIS_lidar_;
    std::string NIS_lidar_filename_;

    // Normalize the angle to be between -2pi and 2pi
    // @param angle difference between the predicted and state yaw angle
    void NormalizeAngle(double *angle);

    // record NIS values to a text file
    // @param filename name of the file to write to
    void RecordNIS(double& NIS, std::string& filename);
};

#endif  // UKF_H