#include "kalman_variables.hpp"

//reference http://www.probabilistic-robotics.org

kalman_variables::kalman_variables()
{
	A(3,3); 
	B(3,3);
	H(3,3);
	R(3,3);
	Q(3,3);
	xt_1(3,1);
	xt_(3,3);
	Sigmat_(3,3);
	Sigmat_1(3,3);
	Rt(3,3);
	Ct(3,3);
	Sigmat(3,3);
	Kt(3,3);
	xt(3,1);
	zt(3,1);
	x_t(3,1);
	ut_1(3,1);
	yt(3,1);
	X(3,3);
	Y(3,3);
	I(3,3);
	//A << 1, dt, 0, 0, 1, dt, 0, 0, 1;
	//B << 1, 0, 0;
	//fill out A,B, other matrices
	//parse file to get states x,z
}


int main()
{
	kalman_variables kalman;
	kalman.n = 3;		// No of states
	kalman.m = 1;		// No of measurements
	kalman.dt = 0.1/30; //Time step for kalman
	int count = 10;     //No of iterations for running kalman

for (int i = 0; i < count; ++i)	
{
	kalman.kalman_iter();
	kalman.kalman_print();
}

	return 0;

}