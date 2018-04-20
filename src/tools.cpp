#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
	rmse << 0,0,0,0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if(estimations.size() != ground_truth.size()
			|| estimations.size() == 0){
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

	//accumulate squared residuals
	for(unsigned int i=0; i < estimations.size(); ++i){

		VectorXd residual = estimations[i] - ground_truth[i];

		//coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	//calculate the mean
	rmse = rmse/estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);
	//recover state parameters
	double px = x_state(0);
	double py = x_state(1);
	double vx = x_state(2);
	double vy = x_state(3);

	//TODO: YOUR CODE HERE 

	//check division by zero
	if(px<0.0001 & py<0.0001 ){
	    cout<<"CalculateJacobian() - Error - Division by zero";
	} else {
	    double pxpy = px*px+py*py;
	    double pxvyvxpy = vx*py-px*vy;
	    Hj(0,0) = px/sqrt(pxpy);
	    Hj(0,1) = py/sqrt(pxpy);
	    Hj(1,0) = -py/pxpy;
	    Hj(1,1) = px/pxpy;
	    Hj(2,0) = py* (vx*py-px*vy) / pow(pxpy,3/2);
	    Hj(2,1) = px* (vy*px-py*vx) / pow(pxpy,3/2);
	    Hj(2,2) = px/sqrt(pxpy);
	    Hj(2,3) = py/sqrt(pxpy);
	}
  return Hj;
}
