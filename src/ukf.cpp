#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

#define EPSILON 0.001

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
    
    // for initializing based on first measurement
    is_initialized_ = false;
    
    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;
    
    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;
    
    //set state dimension
    n_x_ = 5;
    
    //set augmented dimension
    n_aug_ = 7;
    
    //define spreading parameter
    lambda_ = 3 - n_aug_;
    
    // initial state vector
    x_ = VectorXd(n_x_);
    
    // initial covariance matrix
    P_ = MatrixXd(n_x_, n_x_);
    
    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 1.5;
    
    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 0.3;
    
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
    
    // Predicted sigma points
    Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
    
    // Weights
    weights_ = VectorXd(2 * n_aug_ + 1);
    double weight_0 = lambda_ / (lambda_ + n_aug_);
    weights_(0) = weight_0;
    for (int i = 1; i < 2 * n_aug_ + 1; i++) {
        weights_(i) = 0.5 / (n_aug_ + lambda_);
    }
    
    // Initialize P
    P_ = 0.1 * MatrixXd::Identity(n_x_, n_x_);
    
    // Measurement noise covariance matrices initialization
    R_radar_ = MatrixXd(3, 3);
    R_radar_ << std_radr_*std_radr_, 0, 0,
                0, std_radphi_*std_radphi_, 0,
                0, 0, std_radrd_*std_radrd_;
    
    R_lidar_ = MatrixXd(2, 2);
    R_lidar_ << std_laspx_*std_laspx_, 0,
                0, std_laspy_*std_laspy_;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
    // As per CTRV Model, x_ is [px, py, vel, ang, ang_rate]
    if (!is_initialized_) {
        /**
         * Initialize the state x_ with the first measurement.
         */
        if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
            /**
             Convert radar from polar to cartesian coordinates and initialize state.
             */
            float rho     = meas_package.raw_measurements_(0);
            float phi    = meas_package.raw_measurements_(1);
            float rho_dot = meas_package.raw_measurements_(2);
            // Coordinates convertion from polar to cartesian
            float px = rho * cos(phi);
            float py = rho * sin(phi);
            float vx = rho_dot * cos(phi);
            float vy = rho_dot * sin(phi);
            float v  = sqrt(vx * vx + vy * vy);
            x_ << px, py, v, 0, 0;
        }
        else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
            float px = meas_package.raw_measurements_(0);
            float py = meas_package.raw_measurements_(1);
            if (fabs(px) < EPSILON and fabs(py) < EPSILON) {
                px = EPSILON;
                py = EPSILON;
            }
            x_ << px, py, 0, 0, 1;
        }
        previous_timestamp_ = meas_package.timestamp_;
        // done initializing
        is_initialized_ = true;
        return;
    }
    //compute the time elapsed between the current and previous measurements
    //dt - expressed in seconds
    float dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = meas_package.timestamp_;
    
    // Predict
    Prediction(dt);
    
    //Update
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR and use_radar_) {
        UpdateRadar(meas_package);
    } else if (meas_package.sensor_type_ == MeasurementPackage::LASER and use_laser_) {
        UpdateLidar(meas_package);
    }
    return;
}

void UKF::SigmaPointPrediction(double delta_t) {
    //create augmented mean vector
    VectorXd x_aug = VectorXd(n_aug_);
    
    //create augmented state covariance
    MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
    
    //create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
    
    //create augmented mean state
    x_aug.head(n_x_) = x_;
    x_aug(n_x_) = 0;
    x_aug(n_x_ + 1) = 0;
    
    //create augmented covariance matrix
    P_aug.fill(0.0);
    P_aug.topLeftCorner(n_x_, n_x_) = P_;
    P_aug(n_x_, n_x_) = std_a_*std_a_;
    P_aug(n_x_ + 1, n_x_ + 1) = std_yawdd_*std_yawdd_;
    
    //create square root matrix
    MatrixXd L = P_aug.llt().matrixL();
    
    //create augmented sigma points
    Xsig_aug.col(0)  = x_aug;
    for (int i = 0; i< n_aug_; i++) {
        Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
        Xsig_aug.col(i+1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
    }
    
    //predict sigma points
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        //extract values for better readability
        double p_x = Xsig_aug(0,i);
        double p_y = Xsig_aug(1,i);
        double v = Xsig_aug(2,i);
        double yaw = Xsig_aug(3,i);
        double yawd = Xsig_aug(4,i);
        double nu_a = Xsig_aug(5,i);
        double nu_yawdd = Xsig_aug(6,i);
        
        //predicted state values
        double px_p, py_p;
        
        //avoid division by zero
        if (fabs(yawd) > EPSILON) {
            px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
            py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
        }
        else {
            px_p = p_x + v*delta_t*cos(yaw);
            py_p = p_y + v*delta_t*sin(yaw);
        }
        
        double v_p = v;
        double yaw_p = yaw + yawd*delta_t;
        double yawd_p = yawd;
        
        //add noise
        px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
        py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
        v_p = v_p + nu_a*delta_t;
        
        yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
        yawd_p = yawd_p + nu_yawdd*delta_t;
        
        //write predicted sigma point into right column
        Xsig_pred_(0,i) = px_p;
        Xsig_pred_(1,i) = py_p;
        Xsig_pred_(2,i) = v_p;
        Xsig_pred_(3,i) = yaw_p;
        Xsig_pred_(4,i) = yawd_p;
    }
    return;
}

/**
 *  Angle normalization to [-Pi, Pi]
 */
double UKF::NormalizeAngle(double ang) {
    double retval = ang;
    while (retval > M_PI) retval -= 2. * M_PI;
    while (retval < -M_PI) retval += 2. * M_PI;
    return retval;
}

void UKF::PredictMeanAndCovariance() {
    x_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        x_ = x_ + weights_(i) * Xsig_pred_.col(i);
    }
    
    P_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        //angle normalization
        x_diff(3) = NormalizeAngle(x_diff(3));
        P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
    }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
    /**
     * Estimate the object's location. Modify the state vector, x_.
     * Predict sigma points, the state, and the state covariance matrix.
     */
    SigmaPointPrediction(delta_t);
    PredictMeanAndCovariance();
}

/**
 * Common update function.
 */
void UKF::UpdateCommon(MeasurementPackage meas_package, MatrixXd Zsig, int n_z) {
    // Mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }
    //measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z, n_z);
    S.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        // Residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        // Angle normalization
        z_diff(1) = NormalizeAngle(z_diff(1));
        S = S + weights_(i) * z_diff * z_diff.transpose();
    }
    // Add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z, n_z);
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        R = R_radar_;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
        R = R_lidar_;
    }
    S = S + R;
    
    // Create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);
    // Calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
            // Angle normalization
            z_diff(1) = NormalizeAngle(z_diff(1));
        }
        // State difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        // Angle normalization
        x_diff(3) = NormalizeAngle(x_diff(3));
        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }
    // Measurements
    VectorXd z = meas_package.raw_measurements_;
    //Kalman gain K;
    MatrixXd K = Tc * S.inverse();
    // Residual
    VectorXd z_diff = z - z_pred;
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        // Angle normalization
        z_diff(1) = NormalizeAngle(z_diff(1));
    }
    // Update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K * S * K.transpose();
    // Calculate NIS
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        NIS_radar_ = z.transpose() * S.inverse() * z;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
        NIS_laser_ = z.transpose() * S.inverse() * z;
    }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
    /**
     * Use lidar data to update the belief about the object's
     * position. Modify the state vector, x_, and covariance, P_.
     * Also need to calculate the lidar NIS.
     */
    int n_z = 2;
    //create matrix for sigma points in measurement space
    MatrixXd Zsig = Xsig_pred_.block(0, 0, n_z, 2 * n_aug_ + 1);
    UpdateCommon(meas_package, Zsig, n_z);
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
    /**
     * Use radar data to update the belief about the object's
     * position. Modify the state vector, x_, and covariance, P_.
     * Also need to calculate the radar NIS.
     */
    int n_z = 3;
    //create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
    
    //transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        // extract values for better readibility
        double p_x = Xsig_pred_(0,i);
        double p_y = Xsig_pred_(1,i);
        double v  = Xsig_pred_(2,i);
        double yaw = Xsig_pred_(3,i);
        
        double v1 = cos(yaw)*v;
        double v2 = sin(yaw)*v;
        
        // measurement model
        Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);          //r
        Zsig(1,i) = atan2(p_y,p_x);                   //phi
        Zsig(2,i) = (p_x*v1 + p_y*v2 ) / Zsig(0,i);   //r_dot
    }
    
    UpdateCommon(meas_package, Zsig, n_z);
}
