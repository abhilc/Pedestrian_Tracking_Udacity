#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
    VectorXd y = z - H_ * x_;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K =  P_ * Ht * Si;
    int sz = x_.size();
    MatrixXd I = MatrixXd::Identity(sz, sz);

    // new state
    x_ = x_ + (K * y);
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  VectorXd hx(3);
  float px = x_[0], py = x_[1], vx = x_[2], vy = x_[3];
  float px_2 = px*px, py_2 = py*py;
 
  if((px_2 + py_2) < 0.000001)
     return;

  float rho = sqrt(px_2 + py_2), theta = atan2(py, px);
  if(rho < 0.0001)
    rho = 0.0001;
  float rho_dot = (px*vx + py*vy)/rho;

  hx << rho, theta, rho_dot;

  VectorXd y = z - hx;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;
  int sz = x_.size();
  MatrixXd I = MatrixXd::Identity(sz, sz);

  //Correct the angle in the y vector
  while (y(1)>M_PI)
  {
    y(1) -= 2 * M_PI;
  }
  while (y(1)<-M_PI)
  {
    y(1) += 2 * M_PI;
  }

  //new state 
  x_ = x_ + (K * y);
  P_ = (I - K * H_) * P_;

}
