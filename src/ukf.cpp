#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;






// TODO: check that you need P_aug (size 7 in calculating predicted measurements and doing measurement update)







/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {

    n_x_    = 5;

    n_aug_  = 7;  // 5 States + 2 Noise

    n_z_radar_    = 3; // Radar Sensor Measurements

    n_z_lidar_    = 2; // Lidar Measurements

    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;

    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;

    // initial state vector
    x_ = VectorXd(n_x_);
    x_aug_ = VectorXd(n_aug_);
    // mean predicted measurement
    z_pred_ = VectorXd(n_z_radar_);

    // initial covariance matrix
    P_ = MatrixXd(n_x_, n_x_);

    P_aug_ = MatrixXd(n_aug_, n_aug_);

    // Initial sigma point matrix
    Xsig_gen_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);

    // Initial Augmented sigma point matrix
    Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

    // Radar Measurement Covariance Matrix
    R_radar_ = MatrixXd(n_z_radar_, n_z_radar_);

    // Lidar Measurement Covariance Matrix
    R_lidar_ = MatrixXd(n_z_lidar_, n_z_lidar_);

    // Lidar Measurement Matrix
    H_lidar_ = MatrixXd(n_z_lidar_, n_x_);

    // create matrix for sigma points in measurement space
    Zsig_ = MatrixXd(n_z_radar_, n_z_lidar_ * n_aug_ + 1);

    // Weights of sigma points
    weights_ = VectorXd(2*n_aug_+1);

    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 1.5;

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 0.5;

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

    // sigma lambda
    lambda_ = 3 - n_aug_;

    // Initialize the identity matrix
    I_ = MatrixXd::Identity(n_x_,n_x_);

    // Measurement Noise Covariance Matrix - radar
    R_radar_ << std_radr_ * std_radr_,      0,                          0,
                0,                          std_radphi_ * std_radphi_,  0,
                0,                          0,                          std_radrd_ * std_radrd_;

    //measurement covariance matrix - lidar
    R_lidar_ << std_laspx_ * std_laspx_,    0,
                0,                          std_laspy_ * std_laspy_;

    // Initial process Covariance Matrix

    P_.fill(0.0);
    P_ <<   .1, 0, 0, 0, 0,
            0, .1, 0, 0, 0,
            0, 0, .5, 0, 0,
            0, 0, 0, .1, 0,
            0, 0, 0, 0, .1;
    // Initial Augmented Covariance Values


    // Lidar Measurement Matrix
    H_lidar_ <<  1, 0, 0, 0, 0,
                 0, 1, 0, 0, 0;

    NIS_radar_ = 0.0;
    NIS_lidar_ = 0.0;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
    /**
    TODO:

    Complete this function! Make sure you switch between lidar and radar
    measurements.
    */
    // Initialize
    if (!is_initialized_) {
        // first measurement
        cout << "UKF: " << endl;
        x_.fill(0.0);
        VectorXd measurement = meas_package.raw_measurements_;

        if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
            /**
            Convert radar from polar to cartesian coordinates and initialize state.
            */

            auto r    = measurement(0);
            auto phi  = measurement(1);
            auto rdot = measurement(2);
            x_(0) = r * cos(phi);
            x_(1) = r * sin(phi);
            x_(2) = rdot;
            x_(3) = 0;

        } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
            /**
            Initialize state.
            */

            x_(0) = measurement(0);
            x_(1) = measurement(1);

        }



        // set weights
        weights_(0) = lambda_ / (lambda_ + n_aug_);
        for (int i = 1; i < 2 * n_aug_ + 1; i++) {  // 2n+1 weights
            weights_(i)  = 0.5 / (n_aug_ + lambda_);
        }

        previous_timestamp_ = meas_package.timestamp_;
        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }

    double dt = (meas_package.timestamp_ - previous_timestamp_)/ 1000000.0;
    previous_timestamp_ = meas_package.timestamp_;

    /* Prediction Step - check that sufficient time has elapsed */
    auto epsilon = 0.001; // 1 ms
    if ( dt > epsilon )
    {
        Prediction(dt);
        if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) { UpdateRadar(meas_package);}
        else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) { UpdateLidar(meas_package);}
    }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} dt the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double dt) {

    GenerateSigmaPoints();

    PredictSigmaPoints(dt);

    PredictMeanCovariance();

}

void UKF::GenerateSigmaPoints() {

    x_aug_.fill(0.0);
    x_aug_.head(n_x_) = x_;

    P_aug_.fill(0.0);
    P_aug_.topLeftCorner(5, 5) = P_;
    P_aug_(5, 5) = std_a_ * std_a_;
    P_aug_(6, 6) = std_yawdd_ * std_yawdd_;

    // Equation (15)
    // create square root matrix of covariance matrix
    MatrixXd L =  P_aug_.llt().matrixL();
    L = L * sqrt(lambda_ + n_aug_);
    // create sigma points
    Xsig_gen_.col(0) = x_aug_;

    for (int i = 0; i < n_aug_; i++) {
        Xsig_gen_.col(i + 1)            = x_aug_ +  L.col(i);
        Xsig_gen_.col(i + 1 + n_aug_)   = x_aug_ -  L.col(i);
    }
}


void UKF::PredictSigmaPoints(double dt) {

    // Time Update
    //predict sigma points
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {

        // extract values for better readability
        auto p_x        = Xsig_gen_(0, i);
        auto p_y        = Xsig_gen_(1, i);
        auto v          = Xsig_gen_(2, i);
        auto yaw        = Xsig_gen_(3, i);
        auto yawd       = Xsig_gen_(4, i);
        auto nu_a       = Xsig_gen_(5, i);
        auto nu_yawdd   = Xsig_gen_(6, i);

        // predicted state values
        double px_p, py_p;

        auto cosyaw = cos(yaw);
        auto sinyaw = sin(yaw);

        // avoid division by zero
        if (fabs(yawd) > 0.001) {
            px_p = p_x + v / yawd * (sin(yaw + yawd * dt) - sinyaw);
            py_p = p_y + v / yawd * (cosyaw - cos(yaw + yawd * dt));
        } else {
            px_p = p_x + v * dt * cosyaw;
            py_p = p_y + v * dt * sinyaw;
        }

        auto v_p    = v;
        auto yaw_p  = yaw + yawd * dt;
        auto yawd_p = yawd;

        // add noise
        auto temp = 0.5 * nu_a * dt * dt;
        px_p = px_p + temp * cosyaw;
        py_p = py_p + temp * sinyaw;
        v_p  = v_p + nu_a * dt;

        yaw_p  = yaw_p + 0.5 * nu_yawdd * dt * dt;
        yawd_p = yawd_p + nu_yawdd * dt;

        // write predicted sigma point into right column
        Xsig_pred_(0, i) = px_p;
        Xsig_pred_(1, i) = py_p;
        Xsig_pred_(2, i) = v_p;
        Xsig_pred_(3, i) = yaw_p;
        Xsig_pred_(4, i) = yawd_p;
        //Xsig_pred_(5, i) = nu_a;
        //Xsig_pred_(6, i) = nu_yawdd;

    }
}

void UKF::PredictMeanCovariance() {

    //predicted state mean
    // Equation (17)
    x_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
        x_ = x_ + weights_(i) * Xsig_pred_.col(i);
    }

    //std::cout<<" X predicted mean: " << x_ << std::endl;
    //predicted state covariance matrix - remember in UKF, P is incorporated into generated sigma pts.
    // Equation (18)
    P_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;

        // angle normalization
        while (x_diff(3) > M_PI) x_diff(3) -= 2. * M_PI;
        while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;

        P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
    }

}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {

/**
    TODO:

    Complete this function! Use lidar data to update the belief about the object's
    position. Modify the state vector, x_, and covariance, P_.

    You'll also need to calculate the lidar NIS.
    */

    ///* Lidar measurement is linear, so we'll just use a standard Kalman Filter to update
    /// Using the predicted covariance matrix with sigma points however. TODO: Check if this is OK

    VectorXd z_ = meas_package.raw_measurements_;
    MatrixXd Ht = H_lidar_.transpose();
    VectorXd y  = z_ - H_lidar_ * x_;
    MatrixXd S_ = H_lidar_ * P_ * Ht + R_lidar_;
    MatrixXd K_ = P_ * Ht * S_.inverse();

    x_ = x_ + K_ * y;
    P_ = (I_ - K_ * H_lidar_) * P_;

    NIS_lidar_ = y.transpose() * S_ * y;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
    /**
    TODO:

    Complete this function! Use radar data to update the belief about the object's
    position. Modify the state vector, x_, and covariance, P_.

    You'll also need to calculate the radar NIS.
    */

    VectorXd z_ = meas_package.raw_measurements_;

    // transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

        // extract values for better readibility
        double p_x = Xsig_pred_(0, i);
        double p_y = Xsig_pred_(1, i);
        double v   = Xsig_pred_(2, i);
        double yaw = Xsig_pred_(3, i);

        double v1 = cos(yaw) * v;
        double v2 = sin(yaw) * v;

        // measurement model
        Zsig_(0, i) = sqrt(p_x * p_x + p_y * p_y);   //r
        Zsig_(1, i) = atan2(p_y, p_x);               //phi
        Zsig_(2, i) = (p_x * v1 + p_y * v2) / sqrt(p_x * p_x + p_y * p_y); //r_dot
    }

    z_pred_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        z_pred_ = z_pred_ + weights_(i) * Zsig_.col(i);
    }

    //measurement covariance matrix and cross correlation matrix
    MatrixXd Pyy_ = MatrixXd(n_z_radar_, n_z_radar_);
    MatrixXd Pxy_ = MatrixXd(n_x_, n_z_radar_);
    //Cross correlation Matrix Tc
    Pyy_.fill(0.0);
    Pxy_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  // 2n+1 simga points

        // residual
        VectorXd z_diff = Zsig_.col(i) - z_pred_;

        // angle normalization
        while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
        while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;

        Pyy_ = Pyy_ + weights_(i) * z_diff * z_diff.transpose();

        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        //angle normalization
        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

        Pxy_ = Pxy_ + weights_(i) * x_diff * z_diff.transpose();
    }

    Pyy_ = Pyy_ + R_radar_;

    // Kalman gain K;
    MatrixXd K_ = Pxy_ * Pyy_.inverse();

    // residual
    VectorXd z_diff = z_ - z_pred_;

    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // update state mean and covariance matrix
    x_ = x_ + K_ * z_diff;
    P_ = P_ - K_*Pyy_*K_.transpose();


    NIS_radar_ = z_diff.transpose() * Pyy_ * z_diff;

}

VectorXd UKF::getCartesian(void) {

    VectorXd x_cartesian_(4);

    x_cartesian_(0) = x_(0);
    x_cartesian_(1) = x_(1);
    x_cartesian_(2) = x_(2) * cos(x_(3));
    x_cartesian_(3) = x_(2) * sin(x_(3));

    return x_cartesian_;
}




