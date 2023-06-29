
#ifndef KEVINS_RUNGE_KUTTA
#define KEVINS_RUNGE_KUTTA

#include<math.h>
#include "matrix.h"

//! Runge Kutta data structure
typedef struct {
	 Vector *x; //!< state vector
	 Vector *u; //!< control effort
	 Vector *k1x;
	 Vector *k2x;
	 Vector *k3x;
	 Vector *k4x;
	 Vector *tmp;
	 Vector *tmp2;
	void (*model)( Vector*,
								 Vector*,
								 Vector*); //!< model of the dynamics
	double dt; //!< time step
	int size; //!< size of the state vector
} Runge_Kutta;


#ifdef __cplusplus
extern "C" {
#endif

//! setup
 Runge_Kutta* createRK4(
							 void (*model)(
							 		 Vector*,
							 		 Vector*,
							 		 Vector*),
							 double dt,
							 int x_size,
							 int u_size
							 );
//! perform integration
void integrateRK4( Runge_Kutta*);
//! free memory
void freeRK4( Runge_Kutta*);


#ifdef __cplusplus
}
#endif

#endif

