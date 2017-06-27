#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [posX posY vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;
  ///* Augmented state vector: [posx posy vel_abs yaw_angle yaw_rate nu_a nu_yawdd] in SI units and rad
  VectorXd x_aug_;


  ///* mean predicted measurement
  VectorXd z_pred_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* Augmented state covariance matrix
  MatrixXd P_aug_;

  ///* Radar Measurement Noise Covariance Matrix
  MatrixXd R_radar_;

  ///* Lidar Measurement Noise Covariance Matrix
  MatrixXd R_lidar_;

  ///* generated sigma points matrix
  MatrixXd Xsig_gen_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* An Identity Matrix
  MatrixXd I_;

  ///* Lidar Measurement Matrix
  MatrixXd H_lidar_;

  ///* Measurement Sigma Matrix
  MatrixXd Zsig_;

  ///* time when the state is true, in us
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Radar measurement Dimension
  int n_z_radar_;

  ///* Lidar measurement Dimension
  int n_z_lidar_;

  ///* Sigma point spreading parameter
  double lambda_;

  ///* NIS Radar
  double NIS_radar_;

  ///* NIS Lidar
  double NIS_lidar_;

  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);
/**
     * ProcessMeasurement
     * @param none
     */
    ///* returns artesian  state vector: [posX posY velX velY] in SI units and rad
    VectorXd getCartesian(void);

private:
  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double dt);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);

  /**
   * Generates Sigma Points
   * @param none
   */
    void GenerateSigmaPoints(void);

  /**
     * Predict Sigma Points
     * @param dt delta time
     */
    void PredictSigmaPoints(double dt);

  /**
     * Predict Mean and Covariance
     * @param none
     */
    void PredictMeanCovariance(void);

    long long previous_timestamp_;
};

#endif /* UKF_H */
