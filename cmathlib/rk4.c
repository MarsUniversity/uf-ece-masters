
#include "rk4.h"

/*!
This function initializes the data structure.
\param model a pointer to the dynamic equations.
\param dt time step of each iteration.
\param x_size size of the state vector.
\param u_size size of the control effort.
*/
 Runge_Kutta* createRK4(
							 void (*model)( Vector*,
								 						  Vector*,
														  Vector*), //!< model of the dynamics
						 	 double dt, //!< time step
						 	 int x_size, //!< size of the state vector
							 int u_size
						 	 ){
	Runge_Kutta *rk = (Runge_Kutta*)malloc(sizeof( Runge_Kutta));

	rk->x=initVector(x_size,"(rk)x");
	rk->u=initVector(u_size,"(rk)u");
	rk->k1x=initVector(x_size,"(rk)k1x");
	rk->k2x=initVector(x_size,"(rk)k2x");
	rk->k3x=initVector(x_size,"(rk)k3x");
	rk->k4x=initVector(x_size,"(rk)k4x");
	rk->tmp=initVector(x_size,"(rk)tmp");
	rk->tmp2=initVector(x_size,"(rk)tmp2");

	rk->model=model;
	rk->dt=dt;
	rk->size=x_size;

	return rk;
}

/*!
This function performs the integration on the system
using a fourth order, fixed time step runge-kutta
integrateion. The results can be obtained by
getting the Vector x.
\param rk pointer to a Runge_Kutta data structure.
*/
void integrateRK4( Runge_Kutta *rk){
	//k1x=dt*model(Xn);
	rk->model(rk->x,rk->u,rk->tmp);
	vectorMultS(rk->tmp,rk->dt,rk->k1x);

  //k2x=dt*model(Xn+.5*k1x);
  vectorMultS(rk->k1x,.5,rk->tmp);
  vectorAdd(rk->x,rk->tmp,rk->tmp);
  rk->model(rk->tmp,rk->u,rk->tmp2);
  vectorMultS(rk->tmp2,rk->dt,rk->k2x);

  //k3x=dt*model(Xn+.5*k2x);
  vectorMultS(rk->k2x,.5,rk->tmp);
  vectorAdd(rk->x,rk->tmp,rk->tmp);
  rk->model(rk->tmp,rk->u,rk->tmp2);
  vectorMultS(rk->tmp2,rk->dt,rk->k3x);

  //k4x=dt*model(Xn+k3x);
  vectorAdd(rk->x,rk->k3x,rk->tmp);
  rk->model(rk->tmp,rk->u,rk->tmp2);
  vectorMultS(rk->tmp2,rk->dt,rk->k4x);

  //Xn = Xn +(1.0/6.0)*(k1x + 2.0*k2x + 2.0*k3x + k4x );
  vectorMultS(rk->k2x,2.0,rk->tmp);
  vectorMultS(rk->k3x,2.0,rk->tmp2);
  vectorAdd(rk->tmp,rk->tmp2,rk->tmp);
  vectorAdd(rk->tmp,rk->k1x,rk->tmp);
  vectorAdd(rk->tmp,rk->k4x,rk->tmp);
  vectorMultS(rk->tmp,1.0/6.0,rk->tmp);
  vectorAdd(rk->x,rk->tmp,rk->x);
}

/*!
This function frees the memory that was allocated
for the Runge_Kutta data structure.
\param rk a pointer to the Runge_Kutta data structure.
*/
void freeRK4( Runge_Kutta *rk){
	freeVector(rk->k1x);
	freeVector(rk->k2x);
	freeVector(rk->k3x);
	freeVector(rk->k4x);
	freeVector(rk->tmp);
	freeVector(rk->tmp2);
	free(rk);
}
