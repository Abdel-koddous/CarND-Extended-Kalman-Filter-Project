#include "kalman_filter.h"
#include "tools.h"
#include "FusionEKF.h"
#include <iostream>

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
  /**
   * TODO: predict the state
   */
  
  x_ = F_*x_;
  P_ = F_*P_*F_.transpose() + Q_;

  if ( debug_ ) {
    cout << "prediction step - x = " << x_ << endl;
    cout << "prediction step - P = " << P_ << endl;
  }
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */

  VectorXd error_y = z - H_*x_;
  MatrixXd S = H_*P_*H_.transpose() + R_;
  MatrixXd K = P_*H_.transpose()*S.inverse(); // Kalman gain

  // filter output
  x_ = x_ +  K*error_y;
  P_ = ( MatrixXd::Identity(4, 4) - K*H_ )*P_;

  if (debug_) {
    cout << "error_y = " << error_y << endl;   
    cout << "H_ = " << H_ << endl;
    cout << "S = " << S << endl;
    cout << "K = " << K << endl;
  }
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
                                                  
  // convert to polar space (might be added as a method under Tools class ... I just did!)
  VectorXd x_ploar = tools_.ToPolarSpace(x_);
  
  VectorXd error_y = z - x_ploar;

  // Bring error in phi back to interval [-pi, pi] 
  while (error_y(1) > M_PI){
    error_y(1) -= 2*M_PI;
  }
  while ( error_y(1) < -M_PI){
    error_y(1) += 2*M_PI;
  }

  MatrixXd S = H_*P_*H_.transpose() + R_;
  MatrixXd K = P_*H_.transpose()*S.inverse(); // Kalman gain

  // filter output
  x_ = x_ +  K*error_y;
  P_ = ( MatrixXd::Identity(4, 4) - K*H_ )*P_;



  if (debug_) {
    cout << "z = " << z << endl;   
    cout << "x_polar = " << x_ploar << endl;   
    cout << "error_y = " << error_y << endl;   
    cout << "H_ = " << H_ << endl;
    cout << "S = " << S << endl;
    cout << "K = " << K << endl;
  }

}