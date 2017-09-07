#include "kalman_variables.hpp"

//reference http://www.probabilistic-robotics.org

void kalman_variables::prior()
{
	xt_ = A*xt_1 + B*ut_1;
	Sigmat_ = A*Sigmat_1*A.transpose() + Q;

}

void kalman_variables::posterior()
{
	Kt = Sigmat_* H.transpose() * (H*Sigmat_*H.transpose() + R).inverse();
	xt = xt_ + Kt * (zt - H*xt_1);
	Sigmat = (I - Kt*H) * Sigmat_;
}

void kalman_variables::kalman_iter()
{	kalman_variables::get_states();
	kalman_variables::prior();
	kalman_variables::posterior();
}

void kalman_variables::kalman_print()
{ 
	std::cout<< "expected states = " << xt << std::endl;
	std::cout<< "actual states = " << yt << std::endl;
	std::cout<< "error in measurement = " << xt-yt << std::endl;
}



void kalman_variables::get_states()
{
	//get next x, z etc
}

