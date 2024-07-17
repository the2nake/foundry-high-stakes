#pragma once

#include <eigen/Dense>

class Filter {
public:
  virtual void predict(int delta_ms) = 0;
  virtual void predict(int delta_ms, Eigen::VectorXd control_input) = 0;
  virtual void update(int delta_ms, Eigen::VectorXd measurement) = 0;

  virtual Eigen::VectorXd get_state() = 0;
  virtual Eigen::MatrixXd get_covariance() = 0;

  virtual void initialise(Eigen::VectorXd state,
                          Eigen::MatrixXd covariance) = 0;

protected:
  Filter() {}
};