#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  //_cout << "Computing RMSE" << endl;
  VectorXd rmse(4);

  rmse << 0,0,0,0;

  if ( estimations.size() != ground_truth.size() && estimations.size() == 0){
      cout << "Estimations and Ground truth vectors size should be equal and not equal to zero" << endl;
      return rmse;
  }

  for (uint i=0; i < estimations.size(); ++i) {

    VectorXd residual = estimations[i] - ground_truth[i];

    residual = residual.array()*residual.array();

    rmse += residual;

  }

  // calculate the mean of the error
  rmse = rmse*1/estimations.size();

  // calculate the squared root of the accumulated error
  rmse = rmse.array().sqrt();
  //_cout << "Computing RMSE ... DONE " << rmse << endl;
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

  /**
   * TODO:
   * Calculate a Jacobian here.
   */
   // Compute the Jacobian matrix of h(x')
   //_ cout << "Let's compute the Jacobian " << x_state.size() << endl;
   MatrixXd Hj(3, 4);

   float px = x_state(0);
   float py = x_state(1);
   float vx = x_state(2);
   float vy = x_state(3);

   float rho = sqrt( px*px + py*py );
   float rho2 = rho*rho;
   float rho3 = rho*rho2;

   if ( fabs(rho3) < 0.001 ){
      cout << "px and py are Null" << endl;
      return Hj;
   }

   Hj << px/rho, py/rho, 0, 0,
         -py/rho2, px/rho2, 0, 0,
         py*(vx*py - vy*px)/rho3, px*(vy*px - vx*py)/rho3, px/rho, py/rho;

   //_ cout << "Done" << endl;

   return Hj;
}


VectorXd Tools::ToPolarSpace(const Eigen::VectorXd& x_state) {

  VectorXd x_polar(3);

  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float rho = sqrt( px*px + py*py );

  if (fabs(rho) < 0.001){
    cout << "Error -  rho is equal to Zero" << endl;
    return x_polar;
  }

  float phi = atan2(py, px);
  float rho_dot = (px*vx + py*vy)/rho;
    
  x_polar << rho, phi, rho_dot;

  return x_polar;

}
