#include <eigen3/Eigen/Core>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <Eigen/Dense>
#include <stdio.h>
#include <iostream>

using namespace Eigen;
class kalman_variables
{
 private:
 	/**
	* The following are kalman filter matrices: 
	(reference Probablistic Robotics THRUN et.al.) - (http://www.probabilistic-robotics.org)

	A - Matrix describing the system dynamics
	B - Input matrix
	H - Matrix describing relation between sensor measurements and states
	Q - Covariance matrix for noise in states 
	R - Covariance matrix for noise in sensor values
 	**/
	const MatrixXd A; 
	const MatrixXd B;
	const MatrixXd H;
	const MatrixXd R;
	const MatrixXd Q;
	MatrixXd xt_1;
	MatrixXd Sigmat_;
	MatrixXd Sigmat_1;
	MatrixXd Rt;
	MatrixXd Ct;
	MatrixXd Sigmat;
	MatrixXd Kt;
	MatrixXd xt;
	MatrixXd zt;
	MatrixXd x_t;
	MatrixXd xt_;
	MatrixXd ut_1;
	MatrixXd yt;
	MatrixXd X;
	MatrixXd Y;
	MatrixXd I;

 public:
 	int n;
 	int m;

 	double dt;
	kalman_variables();
	//~kalman_variables();
	void prior();		//compute prediction
	void posterior();	//compute correction
	void kalman_iter();	//function which iterates kalman over 
	void kalman_print(); //function to print values to screen
	void get_states(); //function to get next states
	
};

