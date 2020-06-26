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

  //cout << "x = " << x_ << endl;
  //cout << "P = " << P_ << endl;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  int debug = 0;

  VectorXd error_y = z - H_*x_;
  MatrixXd S = H_*P_*H_.transpose() + R_;
  MatrixXd K = P_*H_.transpose()*S.inverse(); // Kalman gain


  x_ = x_ +  K*error_y;
  P_ = ( MatrixXd::Identity(4, 4) - K*H_ )*P_;

  if (debug){
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
                                                  
  //_cout << "UpdateEKF: " << endl;
  int debug = 0;

  // Use Jacobian in the linearization of h(x')
  // convert to polar space (might be added as a method under Tools class)
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  float rho = sqrt( px*px + py*py );
  float phi = atan2(py, px);
  if (fabs(px) < 0.001){
    cout << "Watch out px is Null" << endl;
  }

  float rho_dot = (px*vx + py*vy)/rho;

  while (phi > 2*M_PI){
    phi -= 2*M_PI;
  }
  while ( phi < -2*M_PI){
    phi += 2*M_PI;
  }

  VectorXd x_ploar(3);
  x_ploar << rho, phi, rho_dot;

  VectorXd error_y = z - x_ploar;


  //cout << "Done updating EKF " << endl;

  MatrixXd S = H_*P_*H_.transpose() + R_;
  MatrixXd K = P_*H_.transpose()*S.inverse(); // Kalman gain


  if (debug){
  cout << "px = " << px << " py = " << py << endl;   
  cout << "z = " << z << endl;   
  cout << "x_polar = " << x_ploar << endl;   
  cout << "error_y = " << error_y << endl;   
  cout << "H_ = " << H_ << endl;
  cout << "S = " << S << endl;
  cout << "K = " << K << endl;
  }

  x_ = x_ +  K*error_y;
  P_ = ( MatrixXd::Identity(4, 4) - K*H_ )*P_;
}