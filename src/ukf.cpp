#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

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
    x_.fill(0.0);
    
    // initial covariance matrix
    P_ = MatrixXd(n_x_, n_x_);
    P_.fill(0.0);
    
    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 2;
    
    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 30;
    
    // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.15;
    
    // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.15;
    
    // Radar measurement noise standard deviation radius in m
    std_radr_ = 0.3;
    
    // Radar measurement noise standard deviation angle in rad
    std_radphi_ = 0.0175;
    
    // Radar measurement noise standard deviation radius change in m/s
    std_radrd_ = 0.1;
    
    // Predicted sigma points
    Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
    
    // Weights
    weights_ = VectorXd(2 * n_aug_ + 1);
    double weight_0 = lambda_/(lambda_ + n_aug_);
    weights_(0) = weight_0;
    for (int i = 1; i < 2 * n_aug_ + 1; i++) {
        double weight = 0.5/(n_aug_ + lambda_);
        weights_(i) = weight;
    }
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
    if (!is_initialized_) {
        /**
         * Initialize the state x_ with the first measurement.
         */
        if (meas_package.sensor_type_ == MeasurementPackage::RADAR and use_radar_) {
            /**
             Convert radar from polar to cartesian coordinates and initialize state.
             */
            float ro     = meas_package.raw_measurements_(0);
            float phi    = meas_package.raw_measurements_(1);
            float ro_dot = meas_package.raw_measurements_(2);
            x_ << ro * cos(phi), ro * sin(phi), ro_dot * cos(phi), ro_dot * sin(phi);
        }
        else if (meas_package.sensor_type_ == MeasurementPackage::LASER and use_laser_) {
            x_ << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1), 0, 0;
            if (fabs(x_(0)) < EPSILON1) {
                x_(0) = EPSILON1;
            }
            if (fabs(x_(1)) < EPSILON1) {
                x_(1) = EPSILON1;
            }
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

MatrixXd UKF::AugmentedSigmaPoints() {
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
    return Xsig_aug;
}

void UKF::SigmaPointPrediction(MatrixXd& Xsig_aug, double delta_t) {
    
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
        if (fabs(yawd) > EPSILON1) {
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
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        x_ = x_ + weights_(i) * Xsig_pred_.col(i);
    }
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
    MatrixXd Xsig_aug = AugmentedSigmaPoints();
    SigmaPointPrediction(Xsig_aug, delta_t);
    PredictMeanAndCovariance();
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
    //extract measurement as VectorXd
    VectorXd z = meas_package.raw_measurements_;
    
    int n_z = 2;
    //create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
    
    //transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        // extract values for better readibility
        double p_x = Xsig_pred_(0,i);
        double p_y = Xsig_pred_(1,i);
        
        // measurement model
        Zsig(0,i) = p_x;
        Zsig(1,i) = p_y;
    }
    
    //mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0.0);
    for (int i=0; i < 2 * n_aug_ + 1; i++) {
        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }
    
    //measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z, n_z);
    S.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        //angle normalization
        z_diff(1) = NormalizeAngle(z_diff(1));
        S = S + weights_(i) * z_diff * z_diff.transpose();
    }
    
    //add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z, n_z);
    R << std_laspx_*std_laspx_, 0,
    0, std_laspy_*std_laspy_;
    S = S + R;
    
    //create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);
    
    //calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        //angle normalization
        z_diff(1) = NormalizeAngle(z_diff(1));
        
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        
        //angle normalization
        x_diff(3) = NormalizeAngle(x_diff(3));
        
        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }
    
    //Kalman gain K;
    MatrixXd K = Tc * S.inverse();
    
    //residual
    VectorXd z_diff = z - z_pred;
    
    //angle normalization
    z_diff(1) = NormalizeAngle(z_diff(1));
    
    //calculate NIS
    NIS_laser_ = z_diff.transpose() * S.inverse() * z_diff;
    
    //update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K*S*K.transpose();
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
    
    //extract measurement as VectorXd
    VectorXd z = meas_package.raw_measurements_;
    
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
        Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
        Zsig(1,i) = atan2(p_y,p_x);                                 //phi
        Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
    }
    
    //mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0.0);
    for (int i=0; i < 2 * n_aug_ + 1; i++) {
        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }
    
    //measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z, n_z);
    S.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) { 
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        //angle normalization
        z_diff(1) = NormalizeAngle(z_diff(1));
        S = S + weights_(i) * z_diff * z_diff.transpose();
    }
    
    //add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z, n_z);
    R <<    std_radr_*std_radr_, 0, 0,
    0, std_radphi_*std_radphi_, 0,
    0, 0,std_radrd_*std_radrd_;
    S = S + R;
    
    //create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);
    
    //calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        //angle normalization
        z_diff(1) = NormalizeAngle(z_diff(1));
        
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        
        //angle normalization
        x_diff(3) = NormalizeAngle(x_diff(3));
        
        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }
    
    //Kalman gain K;
    MatrixXd K = Tc * S.inverse();
    
    //residual
    VectorXd z_diff = z - z_pred;
    
    //angle normalization
    z_diff(1) = NormalizeAngle(z_diff(1));
    
    //calculate NIS
    NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
    
    //update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K*S*K.transpose();
}
