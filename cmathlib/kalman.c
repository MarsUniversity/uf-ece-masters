#include "kalman.h"

/*!
  This function is used to run the kalman filter once
  it is setup.  Based on the flags used in the setup,
  this function will call either a continuous or discrete
  kalman filter.
  \param kf a pointer to a Kalman_Filter structure.
*/
int kalman( Kalman_Filter *kf){

  if(kf->flags & KF_DISCRETE) // discrete filter
    discreteKalmanFilter(kf);
  else if(kf->flags & KF_CONTINUOUS) // continous filter
  	continousKalmanFilter(kf);
  else{
    printf("error\n");
    exit(1);
  }
}

/*!
  This function runs a discrete kalman filter.  If the
  kalman filter uses a continuous model, it will call the
  function C2D which will turn it into a discrete system.
  The basic procedure for a discrete kalman filter is:
  \n xk = Ad*xk+Bd*u;
  \n P = Ad*P*Ad'+Q;
  \n k = P*H'*inv(H*P*H'+R);
  \n xk = xk+k*(z-H*xk-D*u);
  \n P = (I-k*H)*P;
  \param kf a pointer to a Kalman_Filter structure.
  \warning currently this does NOT take into account the direct
  transmition term (D).
*/
void discreteKalmanFilter( Kalman_Filter *kf){

  if(kf->flags & KF_NONLINEAR_MODEL) // linearize system model
    linearizeKalmanFilter(kf);

  if(kf->flags & KF_CONT_MODEL) // cont -> discrete model
    C2D(kf);

	//printMatrix(kf->Ac);printMatrix(kf->Bc);
	//printMatrix(kf->Ad);printMatrix(kf->Bd);

	//matrixClear(kf->Ad);
	//matrixClear(kf->Bd);


  /* xk = Ad*xk+Bd*u; */
  vectorMult(kf->Ad,kf->xk,kf->tmpv[0]);
	// ignore the B thing for now
  //vectorMult(kf->Bd,kf->u,kf->tmpv[1]);
  //vectorAdd(kf->tmpv[0],kf->tmpv[1],kf->xk);
	vectorCopy(kf->tmpv[0],kf->xk);

  /* P = Ad*P*Ad'+Q; */
  matrixMult(kf->Ad,kf->P,kf->tmpm[0]);
  matrixTrans(kf->Ad,kf->AdT);
  matrixMult(kf->tmpm[0],kf->AdT,kf->tmpm[1]);
  matrixAdd(kf->tmpm[1],kf->Q,kf->P);

  /* k = P*H'*inv(H*P*H'+R); */
  matrixMult(kf->H,kf->P,kf->tmpm[0]);
  matrixTrans(kf->H,kf->HT);
  matrixMult(kf->tmpm[0],kf->HT,kf->tmpm[1]);
  matrixAdd(kf->tmpm[1],kf->R,kf->tmpm[1]);
  matrixInv(kf->tmpm[1],kf->tmpm[0]);
  matrixMult(kf->HT,kf->tmpm[1],kf->tmpm[2]);
  matrixMult(kf->P,kf->tmpm[2],kf->k);

	//kf->k->p[0][0] = 1.0;

  /* xk = xk+k*(z-H*xk); */
  vectorMult(kf->H,kf->xk,kf->tmpv[0]);
  vectorSub(kf->z,kf->tmpv[0],kf->tmpv[0]); //printVector(kf->tmpv[0]);
  vectorMult(kf->k,kf->tmpv[0],kf->tmpv[1]);
  vectorAdd(kf->xk,kf->tmpv[1],kf->xk);

  //vectorCopy(kf->tmpv[1],kf->xk); // kill state stuff

#if 0 // standard method
  /* P = (I-k*H)*P; */
  matrixCopy(kf->P,kf->P2);
  matrixMult(kf->k,kf->H,kf->tmpm[0]);
  matrixSub(kf->eye,kf->tmpm[0],kf->tmpm[0]);
  matrixMult(kf->tmpm[0],kf->P2,kf->P);
#else //
  /* P=(I-k*H)*P*(I-k*H)'+KRK'; */
  matrixMult(kf->k,kf->H,kf->tmpm[0]);
  matrixEye(kf->tmpm[1]);
  matrixSub(kf->tmpm[1],kf->tmpm[0],kf->tmpm[0]);  // I-KH
  matrixTrans(kf->tmpm[0],kf->tmpm[1]);
  matrixMult(kf->tmpm[0],kf->P,kf->tmpm[2]);
  matrixMult(kf->tmpm[2],kf->tmpm[1],kf->tmpm[0]); // (I-KH)P(I-KH)'
  matrixMult(kf->k,kf->R,kf->tmpm[1]);             // kR
  matrixTrans(kf->k,kf->tmpm[2]);                  // k'
  matrixMult(kf->tmpm[1],kf->tmpm[2],kf->P);       // kRk'
  matrixAdd(kf->tmpm[0],kf->P,kf->P);              // (I-KH)P(I-KH)'+kRk'
#endif

  vectorCopy(kf->xk,kf->x_est);
}


/*!
  This function turns a continuous time state space (Ac) into a discrete
  state transition matrix (Ad) by the following method:

  \n	Ad = I+A*dt+(1/2)*A^2*dt^2+ ... +(1/k!)*A^k*dt^k
  \n	Bd = dt*(I+A*dt/2!+A^2*dt^2/3!+...)*B

  \param kf a pointer to a Kalman_Filter structure.
  \todo do the discrete Bd part - kevin [fixed bug also]
*/
void C2D( Kalman_Filter *kf){
  int i,j;

  // Make A discrete (Ac -> Ad)
  matrixMultS(kf->Ac,kf->dt,kf->tmpm[0]);  // A*dt
  matrixMult(kf->Ac,kf->Ac,kf->tmpm[1]);  // A^2
  matrixMult(kf->tmpm[1],kf->Ac,kf->tmpm[2]); // A^3
  matrixMultS(kf->tmpm[1],kf->dt*kf->dt/2.0,kf->tmpm[1]);  // A^2*dt^2/2
  matrixAdd(kf->tmpm[0],kf->tmpm[1],kf->tmpm[0]); // A*dt+A^2*dt^2/2
  matrixMultS(kf->tmpm[2],kf->dt*kf->dt*kf->dt/6.0,kf->tmpm[2]);  // A^3*dt^3/6
  matrixAdd(kf->tmpm[0],kf->tmpm[2],kf->tmpm[0]); // A*dt+A^2*dt^2/2+A^3*dt^3/6
  matrixAdd(kf->eye,kf->tmpm[0],kf->Ad);  // I+A*dt+A^2*dt^2/2+A^3*dt^3/6
  //printMatrix(kf->Ad);

  // Make B discrete (Bc -> Bd)
  //printf("g\n");
  //matrixMultS(kf->Bc,kf->dt,kf->Bd); // B*dt - this is a test

  matrixMultS(kf->Bc,kf->dt,kf->tmpB[0]); // B*dt
  matrixMult(kf->Ac,kf->Bc,kf->tmpB[1]); // A*B
  matrixMultS(kf->tmpB[1],kf->dt*kf->dt/2.0,kf->tmpB[1]); // A*B*dt^2/2!
  matrixAdd(kf->tmpB[0],kf->tmpB[1],kf->tmpB[0]); // B*dt+A*B*dt^2/2!
  matrixMult(kf->Ac,kf->Ac,kf->tmpm[0]);  // A^2
  matrixMultS(kf->tmpm[0],kf->dt*kf->dt*kf->dt/6.0,kf->tmpm[0]); // A^2*dt^3/3!
  matrixMult(kf->tmpm[0],kf->Bc,kf->tmpB[1]);
  matrixAdd(kf->tmpB[0],kf->tmpB[1],kf->Bd);
  //printMatrix(kf->Bd);
}

/*!
  Constructor for discrete kalman filter with a continuous nonlinear model
  or a discrete linear model.  The discrete linear model is assumed to be
  in the form:
  \n Xdot = A*X+Bu
  \n Y = HX+Du
  \n and the nonlinear model is assumed to be of the form:
  \n Xdot = f(X,u)
  \n Y = HX+Du

  \note These functions assume a square A and H matrix of (size x size) and
  a B matrix of (size x u_size).

  \todo handle the direct transmition D.

  \warning Please do not feed a continous linear model in here, this would
  be a waste of run time.  Instead, go use matlab (or something) and convert
  your continous linear model into a discrete using the command c2d (or
  any other one).

  \warning I have NOT verified the linear kalman filter stuff really works!!  I
  have only worked with the nonlinear stuff.
*/
Kalman_Filter* createKalmanFilter(
       void (*model)(
	     Vector *input_state,
	     Vector *control_effort,
	     Vector *output_state), //!< nonlinear continuous model dynamics
       Matrix *A,                 //!< discrete state transition matrix
       Matrix *B,                 //!< discrete input matrix
       Matrix *H,                 //!< discrete output matrix
       Matrix *D,                 //!< discrete direct transfer matrix
       Matrix *Q,                 //!< discrete process noise
       Matrix *R,                 //!< discrete measurement noise
       Vector *initState,         //!< initial states for kalman filter
       float dt,                   //!< time step (for linearization)
       int size,                   //!< size of state vector
       int u_size,                 //!< size of control effort vector
       int flags                   //!< Kalman Filter Flags
       ){

  Kalman_Filter *kf = ( Kalman_Filter*)malloc(sizeof( Kalman_Filter));

  kf->flags = flags;
  kf->size = size;
  kf->u_size = u_size;
  kf->model = model;
  kf->dt=dt;

  //--- State Space Stuff -------------------
  kf->Ad=initMatrix(size,size,"Ad"); matrixClear(kf->Ad);    /* discrete state transition matrix */
  kf->AdT=initMatrix(size,size,"AdT"); matrixClear(kf->AdT); /* transpose of Ad */
  kf->Bd=initMatrix(size,u_size,"Bd"); matrixClear(kf->Bd);  /* discrete input matrix */
  kf->H=initMatrix(H->rows,H->cols,"H"); matrixCopy(H,kf->H);
  kf->HT=initMatrix(H->cols,H->rows,"HT"); matrixTrans(H,kf->HT);
  kf->D=initMatrix(D->rows,D->cols,"D"); matrixCopy(D,kf->D);
  kf->u=initVector(u_size,"u");

  //--- Temporary Data Structures -----------
  kf->eye=initMatrixEye(size);
  kf->tmpm = ( Matrix**)malloc(sizeof( Matrix*)*4);
  kf->tmpm[0]=initMatrix(size,size,"tmpm");
  kf->tmpm[1]=initMatrix(size,size,"tmpm2");
  kf->tmpm[2]=initMatrix(size,size,"tmpm3");
  kf->tmpm[3]=initMatrix(size,size,"tmpm4");
  kf->tmpv = ( Vector**)malloc(sizeof( Vector*)*3);
  kf->tmpv[0]=initVector(size,"tmpv");
  kf->tmpv[1]=initVector(size,"tmpv2");
  kf->tmpv[2]=initVector(size,"tmpv3");

  //--- Kalman Gain Stuff --------------------
  kf->P=initMatrix(size,size,"P"); matrixClear(kf->P);        /* error cov */
  kf->P2=initMatrix(size,size,"P2"); matrixClear(kf->P2);     /* P' */
  kf->k=initMatrix(size,size,"K"); matrixClear(kf->k);        /* kalman gain */
  kf->z=initVector(size,"z");  vectorClear(kf->z);            /* noisy data */
  kf->x_est=initVector(size,"x est");  vectorClear(kf->x_est);/* estimated/filtered data */
  kf->xk=initVector(size,"xk"); vectorClear(kf->xk);          /* noisless kalman discrete state */
  kf->Q=initMatrix(Q->rows,Q->cols,"Q"); matrixCopy(Q,kf->Q);
  kf->R=initMatrix(R->rows,R->cols,"R"); matrixCopy(R,kf->R);

  //--- System Type --------------------------
  if(kf->flags & KF_NONLINEAR_MODEL){
    if(model == NULL){
      printf("ERROR: the nonlinear kalman filter MUST have a nonlinear model!\n");
      exit(1);
    }
    if(A != NULL || B != NULL){
      printf("WARNING: the nonlinear kalman filter will ignore your input for the matrix A and B.\n");
    }
    //--- linearization ---
    kf->linstate=initVector(size,"linstate"); vectorClear(kf->linstate); /* linearized state */
    kf->Ac=initMatrix(size,size,"Ac"); matrixClear(kf->Ac);   /* continuous state transition matrix */
    kf->AcT=initMatrix(size,size,"AcT"); matrixClear(kf->AcT);/* transpose of Ac */
    kf->Bc=initMatrix(size,u_size,"Bc"); matrixClear(kf->Bc); /* continuous input matrix */
    kf->ulinstate = initVector(u_size,"u linstate");
    kf->tmpu = initVector(u_size,"tmp u");
    //--- C2D Stuff ---
    kf->tmpB = ( Matrix**)malloc(sizeof( Matrix*)*2);
    kf->tmpB[0]=initMatrix(size,u_size,"tmpB0");
    kf->tmpB[1]=initMatrix(size,u_size,"tmpB1");
  }
  else if(kf->flags & KF_LINEAR_MODEL){
    if(model != NULL){
      printf("WARNING: the linear kalman filter will ignore the model you gave it!\n");
    }
    if(A == NULL || B == NULL){
      printf("ERROR: the linear kalman filter MUST have an input for the discrete matrix A and B.\n");
      exit(1);
    }
    if(kf->flags & KF_DISC_MODEL){
      matrixCopy(A,kf->Ad);
      matrixTrans(kf->Ad,kf->AdT);
      matrixCopy(B,kf->Bd);
    }
    else if(kf->flags & KF_CONT_MODEL){
      //--- linearization ---
      //kf->linstate=initVector(size,"linstate"); vectorClear(kf->linstate); /* linearized state */
      kf->Ac=initMatrix(size,size,"Ac"); matrixCopy(A,kf->Ac);   /* continuous state transition matrix */
      kf->AcT=initMatrix(size,size,"AcT"); matrixTrans(A,kf->AcT);/* transpose of Ac */
      kf->Bc=initMatrix(size,u_size,"Bc"); matrixCopy(B,kf->Bc); /* continuous input matrix */
      //--- C2D Stuff ---
      kf->tmpB = ( Matrix**)malloc(sizeof( Matrix*)*2);
      kf->tmpB[0]=initMatrix(size,u_size,"tmpB0");
      kf->tmpB[1]=initMatrix(size,u_size,"tmpB1");
    }
  }
  else{
    printf("ERROR: the kalman filter MUST be either linear or nonlinear\n");
    exit(1);
  }

  printf("done\n");
  return kf;
}


/*!
  This function linearizes the nonlinear eqations of modtion
  (from the function model) in to Ac.  The matrix Ac is a
  continuous state space matrix, which will need to be made
  discrete using the C2D function if a discrete kalman filter
  is being used.
  \param kf a pointer to a Kalman_Filter structure.
  \todo move ulinstate and tmpu into Kalman_Filter
  for better performance. [done]
*/
void linearizeKalmanFilter( Kalman_Filter *kf){
  int j;

  vectorClear(kf->tmpu);
  for(j=0; j<kf->size;j++){  // linearize the system
    vectorCopy(kf->xk,kf->linstate);
    kf->linstate->p[j] += kf->dt;
    kf->model(kf->linstate,kf->u,kf->tmpv[1]);

    vectorCopy(kf->xk,kf->linstate);
    kf->linstate->p[j] -= kf->dt;
    kf->model(kf->linstate,kf->u,kf->tmpv[2]);

    vectorSub(kf->tmpv[1],kf->tmpv[2],kf->tmpv[0]);
    vectorDivS(kf->tmpv[0],2.0*kf->dt,kf->tmpv[0]);
    matrixFillColV(kf->Ac,j,kf->tmpv[0]);
  }
	//-----------------------------------------------------
/*    vectorClear(kf->tmpu); */
/*    for(j=0; j<kf->size;j++){  // linearize the system */
/*      vectorClear(kf->linstate); */
/*      kf->linstate->p[j] = kf->xk->p[j] + kf->dt; */
/*      kf->model(kf->linstate,kf->tmpu,kf->tmpv[1]); */

/*      kf->linstate->p[j] = kf->xk->p[j] - kf->dt; */
/*      kf->model(kf->linstate,kf->tmpu,kf->tmpv[2]); */

/*      vectorSub(kf->tmpv[1],kf->tmpv[2],kf->tmpv[0]); */
/*      vectorDivS(kf->tmpv[0],2.0*kf->dt,kf->tmpv[0]); */
/*      matrixFillColV(kf->Ac,j,kf->tmpv[0]); */
/*    } */

/*    for(j=0; j<kf->u_size;j++){  // linearize the system */
/*      vectorClear(kf->tmpv[2]); */
/*      vectorClear(kf->ulinstate); */
/*      kf->ulinstate->p[j] = kf->u->p[j] + kf->dt; */
/*      kf->model(kf->tmpv[2],kf->ulinstate,kf->tmpv[0]); */

/*      kf->ulinstate->p[j] = kf->u->p[j] - kf->dt; */
/*      kf->model(kf->tmpv[2],kf->tmpu,kf->tmpv[1]); */

/*      vectorSub(kf->tmpv[0],kf->tmpv[1],kf->tmpv[2]); */
/*      vectorDivS(kf->tmpv[2],2.0*kf->dt,kf->tmpv[2]); */
/*      matrixFillColV(kf->Bc,j,kf->tmpv[2]); */
/*    } */
}


/*!
  Free's the memory allocated by creating a Kalman Filter.
  \param kf a pointer to a Kalman_Filter structure to be freed.
*/
void freeKalmanFilter( Kalman_Filter *kf){
  int i;

  freeMatrix(kf->P);
  freeMatrix(kf->k);
  freeMatrix(kf->eye);
  freeMatrix(kf->AcT);
  freeMatrix(kf->AdT);
  freeMatrix(kf->HT);
  freeMatrix(kf->P2);

  for(i=0;i<4;i++)
    freeMatrix(kf->tmpm[i]);

  freeVector(kf->linstate);
  freeVector(kf->xk);
  freeVector(kf->x_est);
  freeVector(kf->z);
  freeVector(kf->u);
  for(i=0;i<3;i++)
    freeVector(kf->tmpv[i]);

  free(kf);
}




////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
/*!
This function runs a continuous kalman filter using either a
nonlinear or linear model.  The nonlinear model will be linearized
just like the discrete kalman filter.  In the continuous kalman
filter, the error covarience (P) is given by a well known
function called the riccati equation:
\par P = A*P+P*A'-P*H'*inv(R)*H*P+B*Q*B'
This equation has an analytical solution which involves solving
two differential equations:
\par Xdot = AcX+BQB'Z
\par Zdot = H'*inv(R)*H*X-Ac'*Z
\par P=X*inv(Z)
and the kalman gain can be calculated from:
\par K = P*H'*inv(R)
and finally the x_est is:
\par x_est_dot = Ac*x_est+B*u+K(z-H*x_est-D*u)
NOTE: Z is NOT z (noisey measurements) and X is NOT
x_est (the model, noise free state), they are just
variables used to solve the riccati equation.

\todo write this function - kevin
*/

void continousKalmanFilter( Kalman_Filter *kf){
  if(kf->flags & KF_NONLINEAR_MODEL)
  	linearizeKalmanFilter(kf);

  int_P(kf);

  // K = P*H'*inv(R)
  matrixCopy(kf->R,kf->tmpm[0]);
  matrixInv(kf->tmpm[0],kf->tmpm[1]);
  matrixTrans(kf->H,kf->HT);
  matrixMult(kf->P,kf->HT,kf->tmpm[1]);
  matrixMult(kf->tmpm[1],kf->tmpm[0],kf->k);

  int_X(kf);
}


void int_X(Kalman_Filter *kf){
	Vector *k1x = initVector(kf->size,"(kf)k1x");
	Vector *k2x = initVector(kf->size,"(kf)k2x");
	Vector *k3x = initVector(kf->size,"(kf)k3x");
	Vector *k4x = initVector(kf->size,"(kf)k4x");
	Vector *tmp = initVector(kf->size,"(kf)tmp");
	Vector *tmp2 = initVector(kf->size,"(kf)tmp2");

	//k1x=dt*model(Xn);
	xmodel(kf->xk,kf,tmp);
	vectorMultS(tmp,kf->dt,k1x);

  //k2x=dt*model(Xn+.5*k1x);
  vectorMultS(k1x,.5,tmp);
  vectorAdd(kf->xk,tmp,tmp);
  xmodel(tmp,kf,tmp2);
  vectorMultS(tmp2,kf->dt,k2x);

  //k3x=dt*model(Xn+.5*k2x);
  vectorMultS(k2x,.5,tmp);
  vectorAdd(kf->xk,tmp,tmp);
  xmodel(tmp,kf,tmp2);
  vectorMultS(tmp2,kf->dt,k3x);

  //k4x=dt*model(Xn+k3x);
  vectorAdd(kf->xk,k3x,tmp);
  xmodel(tmp,kf,tmp2);
  vectorMultS(tmp2,kf->dt,k4x);

  //Xn = Xn +(1.0/6.0)*(k1x + 2.0*k2x + 2.0*k3x + k4x );
  vectorMultS(k2x,2.0,tmp);
  vectorMultS(k3x,2.0,tmp2);
  vectorAdd(tmp,tmp2,tmp);
  vectorAdd(tmp,k1x,tmp);
  vectorAdd(tmp,k4x,tmp);
  vectorMultS(tmp,1.0/6.0,tmp);
  vectorAdd(kf->xk,tmp,kf->xk);

	vectorCopy(kf->xk,kf->x_est);
}

void int_P(Kalman_Filter *kf){
	Matrix *k1x = initMatrix(kf->size,kf->size,"(kf)k1x m");
	Matrix *k2x = initMatrix(kf->size,kf->size,"(kf)k2x m");
	Matrix *k3x = initMatrix(kf->size,kf->size,"(kf)k3x m");
	Matrix *k4x = initMatrix(kf->size,kf->size,"(kf)k4x m");
	Matrix *tmp = initMatrix(kf->size,kf->size,"(kf)tmp m");
	Matrix *tmp2 = initMatrix(kf->size,kf->size,"(kf)tmp2 m");

	//k1x=dt*model(Xn);
	pmodel(kf->P,kf,tmp);
	matrixMultS(tmp,kf->dt,k1x);

  //k2x=dt*model(Xn+.5*k1x);
  matrixMultS(k1x,.5,tmp);
  matrixAdd(kf->P,tmp,tmp);
  pmodel(tmp,kf,tmp2);
  matrixMultS(tmp2,kf->dt,k2x);

  //k3x=dt*model(Xn+.5*k2x);
  matrixMultS(k2x,.5,tmp);
  matrixAdd(kf->P,tmp,tmp);
  pmodel(tmp,kf,tmp2);
  matrixMultS(tmp2,kf->dt,k3x);

  //k4x=dt*model(Xn+k3x);
  matrixAdd(kf->P,k3x,tmp);
  pmodel(tmp,kf,tmp2);
  matrixMultS(tmp2,kf->dt,k4x);

  //Xn = Xn +(1.0/6.0)*(k1x + 2.0*k2x + 2.0*k3x + k4x );
  matrixMultS(k2x,2.0,tmp);
  matrixMultS(k3x,2.0,tmp2);
  matrixAdd(tmp,tmp2,tmp);
  matrixAdd(tmp,k1x,tmp);
  matrixAdd(tmp,k4x,tmp);
  matrixMultS(tmp,1.0/6.0,tmp);
  matrixAdd(kf->P,tmp,kf->P);
}

/*!
	x is being integrated here.
*/
void xmodel( Vector *x, Kalman_Filter *kf,  Vector *Xdot){
	// Xdot = model(x,u)+K(Z-Hx)
	kf->model(x,kf->u,kf->tmpv[0]);
	vectorMult(kf->H,x,kf->tmpv[1]);
	vectorSub(kf->z,kf->tmpv[1],kf->tmpv[1]);
	vectorMult(kf->k,kf->tmpv[1],kf->tmpv[2]);
	vectorAdd(kf->tmpv[0],kf->tmpv[2],Xdot);
}

/*!
	P is being intedgrated, all others are const
*/
void pmodel( Matrix *P, Kalman_Filter *kf, Matrix *Pdot){
	// Pdot = FP+PF'-PH'inv(R)HP+GQG
	matrixMult(kf->Ac,P,kf->tmpm[0]);
	matrixTrans(kf->Ac,kf->AcT);
	matrixMult(P,kf->AcT,kf->tmpm[1]);
	matrixAdd(kf->tmpm[0],kf->tmpm[1],kf->tmpm[0]);
	matrixTrans(kf->H,kf->HT);
	matrixMult(P,kf->HT,kf->tmpm[1]);
	matrixSub(kf->tmpm[0],kf->tmpm[1],kf->tmpm[0]);
	matrixCopy(kf->R,kf->tmpm[1]);
	matrixInv(kf->tmpm[1],kf->tmpm[2]);
	matrixMult(kf->tmpm[0],kf->tmpm[1],kf->tmpm[2]);
	matrixMult(P,kf->H,kf->tmpm[1]);
	matrixMult(kf->tmpm[2],kf->tmpm[1],kf->tmpm[0]);
	matrixAdd(kf->tmpm[0],kf->Q,Pdot);
}


////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
