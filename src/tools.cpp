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
  VectorXd rmse(4);



  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
   // Compute the Jacobian matrix of h(x')
  MatrixXd Hj(3, 4);

  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float rho = sqrt( px*px + py*py );
  float rho2 = rho*rho;
  float rho3 = rho*rho2;

  if (rho == 0){
      cout << "px and py are Null" << endl;
      return Hj;
  }

  Hj << px/rho, py/rho, 0, 0,
        -py/rho2, px/rho2, 0, 0,
        py*(vx*py - vy*px)/rho3, px*(vy*px - vx*py)/rho3, px/rho, py/rho;

   return Hj;
}
