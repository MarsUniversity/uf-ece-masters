
#include "nav_filter.h"
#include "navigation.h"
#include <iostream>

using namespace std;

/*!
  This function initializes the kalman filter for the
  navigation system.
*/
void initKF(cDKF &kf){
  float w_iee[3]={0.0,0.0,NAV_W_IE};
  cMatrix Q(KF_STATE_SIZE,KF_STATE_SIZE,"Q");
  cMatrix R(KF_STATE_SIZE,KF_STATE_SIZE,"R");
  cMatrix A(KF_STATE_SIZE,KF_STATE_SIZE,"A KF");
  cMatrix B(KF_STATE_SIZE,KF_CONTROL_SIZE,"B KF");
  cMatrix H(KF_STATE_SIZE,KF_STATE_SIZE,"H");
  cMatrix D(KF_STATE_SIZE,KF_STATE_SIZE,"D");

  Q = Q.eye(); //0.01*Q.eye();
  R = R.eye();
  H = H.eye();

  //--- Static A Parts ---------
  A.set( -2.0*skew(w_iee),0,0);
  A.set(-1.0*skew(w_iee)*skew(w_iee),0,3);
  A.set(eye(3),3,0);

  kf.init(A,B,H);
  kf.setQ(1.0);
  kf.setR(R);
  kf.K.eye();

  //cout<<A;
}


void initKF(cCKF &kf){
	float w_iee[3]={0.0,0.0,NAV_W_IE};
	cMatrix Q(KF_STATE_SIZE,KF_STATE_SIZE,"Q");
	cMatrix R(KF_STATE_SIZE,KF_STATE_SIZE,"R");
	cMatrix A(KF_STATE_SIZE,KF_STATE_SIZE,"A KF");
	cMatrix B(KF_STATE_SIZE,KF_CONTROL_SIZE,"B KF");
	cMatrix H(KF_STATE_SIZE,KF_STATE_SIZE,"H");
	cMatrix D(KF_STATE_SIZE,KF_STATE_SIZE,"D");

	Q = Q.eye(); //0.01*Q.eye();
	R = R.eye();
	H = H.eye();

	//--- Static A Parts ---------
	A.set( -2.0*skew(w_iee),0,0);
	A.set(-1.0*skew(w_iee)*skew(w_iee),0,3);
	A.set(eye(3),3,0);

	kf.init(A,B,H,KF_DT);
	kf.setQ(300); // 300
	kf.setR(R);
	//kf.K.eye();

	//cout<<A;
}



/*!
  This function reads in a configuration file
  and sets the Q and R matricies. Currently,
  those matricies can only be diagonal ones.
  This function checks the dimensions of Q and
  R, and what is in the config file. If those
  dimensions do not match, this functions prints
  an error message and exits.
  \param kf a Kalman_Filter structure which MUST
  alread have been created.
*/
void getFilterProperties(cCKF &kf){
#if 0
  FILE *fd;
  char *filename = "config/nav.cfg";
  int r,c;
  int size;
  int i;
  char s[2];
  float data;

  if(kf == NULL){
    printf("ERROR: you must create the Kalman_Filter structure first\n");
    exit(1);
  }

  if((fd = fopen(filename,"r")) == NULL){
    printf("ERROR: could not open %s for the kalman filter\n",
	   filename);
    exit(1);
  }

  //--- Fill Q matrix ---------------------------------
  fscanf(fd,"Q size: %i %i \n",&r,&c);
  if(kf->Q->rows != r || kf->Q->cols != c){
    printf("ERROR: Q[%i,%i] dimensions are wrong!!!\n",
	   kf->Q->rows,
	   kf->Q->cols);
    exit(1);
  }

  size=( (r>c) ? r : c); // is row or column bigger?

  for(i=0;i<size;i++){
    fscanf(fd,"%lf",&data);
    kf->Q->p[i][i] = data;
  }
  while(s[0] != '\n') fscanf(fd,"%c",&s[0]); // get to end of line

  //--- Fill R matrix ---------------------------------
  fscanf(fd,"R size: %i %i \n",&r,&c);
  if(kf->R->rows != r || kf->R->cols != c){
    printf("ERROR: R[%i,%i] dimensions are wrong!!!\n",
	   kf->R->rows,
	   kf->R->cols);
    exit(1);
  }

  size=( (r>c) ? r : c); // is row or column bigger?

  for(i=0;i<size;i++){
    fscanf(fd,"%lf",&data);
    kf->R->p[i][i] = data;
  }
#endif
}

/*!
  This is the poor man's way of producing random noise.
  It is not true white noise statistically, but it
  looks like it to the naked eye.
*/
float randN(void){
  static float t=0;

  t+=.01;
  return sin(500*t)*cos(1000*t);
}


















