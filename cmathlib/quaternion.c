
#include "quaternion.h"

double* initQuat(double axis[3], double angle){
	double *out=(double*)malloc(4*sizeof(double));
	int sum=axis[0]+axis[1]+axis[2];
	out[0]=axis[0]/sum*sin(angle/2.0);
	out[1]=axis[1]/sum*sin(angle/2.0);
	out[2]=axis[2]/sum*sin(angle/2.0);
	out[3]=cos(angle/2.0);
	return out;
}




/*!
  This function converts a quaternion into roll, pitch,
  and yaw.
  \param q a pointer to a double array containing the
  four elements of a quaternion.
  \param r a pointer to roll.
  \param p a pointer to pitch.
  \param y a pointer to yaw.
*/
void q2e3(double *q, double *r, double *p, double *y){
  double xx,yy,cp;
  double r11,r12,r23,r13,r33;

  if(1){ // nechyba paper << this uses a 1-2-3 rot!!
    r11 = 1.0-2.0*(q[1]*q[1]+q[2]*q[2]);
    r12 = 2.0*(q[0]*q[1]+q[3]*q[2]);
    r23 = 2.0*(q[1]*q[2]+q[3]*q[0]);
    r13 = 2*(-q[3]*q[1]+q[0]*q[2]);
    r33 = 1.0-2.0*(q[0]*q[0]+q[1]*q[1]);
  }
  else if(0){ // use std quat rot mat def, screwd up!!
    r11 = 2.0*q[0]*q[0]-1.0+2.0*q[1]*q[1];
    r12 = 2.0*(q[1]*q[2]-q[0]*q[3]);
    r23 = 2.0*(q[2]*q[3]-q[0]*q[1]);
    r13 = 2*(q[1]*q[3]+q[0]*q[2]);
    r33 = 2.0*q[0]*q[0]-1.0+2.0*q[3]*q[3];
  }
  else{ // this works kuiper's 3-2-1 rotation matrix def
    r11 = (2.0*q[3]*q[3]-1.0+2.0*q[0]*q[0]);
    r12 = 2.0*(q[0]*q[1]+q[3]*q[2]);
    r23 = 2.0*(q[1]*q[2]+q[3]*q[0]);
    r13 = 2*(q[0]*q[2]-q[3]*q[1]);
    r33 = 2.0*q[3]*q[3]-1.0+2.0*q[2]*q[2];
  }

#if 0 // test
  q[0] = 0;
  q[1] = 0;
  q[2] = 0;
  q[3] = 1;
#endif

#if 0
  *y = atan((2.0*q[0]*q[1]+2.0*q[3]*q[2])/(2.0*q[3]*q[3]+2.0*q[0]*q[0]-1.0));
  *p = asin(-(2.0*q[0]*q[2]-2.0*q[3]*q[1]));
  *r = atan((2.0*q[1]*q[2]+2.0*q[3]*q[0])/(2.0*q[3]*q[3]+2.0*q[2]*q[2]-1.0));
#else // use a 3-2-1 definition of rotation matrix
  *p = asin(-r13);
  cp = cos(*p);

  if(p == 0){ // pitch = 90 degrees
    printf(" WARNING: pitch=0, gimble lock\n");
    *p = asin(-r13);

    xx = 0; //r22; << this is an error
    yy = -r23;
    *r = atan2(yy,xx); //<<check htis

    *y = 0.0;
  }
  else{ // pitch between +/- 90 degrees
    xx = r33/cp;
    yy = r23/cp;
    *r = atan2(yy,xx);

    xx = r11/cp;
    yy = r12/cp;
    *y = atan2(yy,xx);
  }

  if(*y < 0.0) *y += 2.0*M_PI; // want heading 0-360 degrees

  //printf("heading: %f\n",*y);
#endif
  //printf("roll %f %f %f\n",*r,*p,*y);
}

/*!
  This converts euler angles to quaterions.
  \param q array with quaternions [x y z real]
  \param r roll
  \param p pitch
  \param y yaw
  \note This function checks to ensure that the norm of the quaternion
  is 1.0 and if it is not, it then makes it one.  Also it protects
  against division by zero if a correction is made.
*/
void e2q3(double *q, double r, double p, double y){
  double sr,sp,sy,cr,cp,cy,mag;

  sr = sin(r/2.0);
  sp = sin(p/2.0);
  sy = sin(y/2.0);
  cr = cos(r/2.0);
  cp = cos(p/2.0);
  cy = cos(y/2.0);

  // calc quaternions
  q[0] = cy*cp*sr-sy*sp*cr; // x
  q[1] = cy*sp*cr+sy*cp*sr; // y
  q[2] = sy*cp*cr-cy*sp*sr; // z
  q[3] = cy*cp*cr+sy*sp*sr; // real

  mag = sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]
	     +q[3]*q[3]);

  if((mag>1.0001 || mag<0.9999) && mag>1e-4){ // check this!!!!!!
    q[0] /= mag; // x
    q[1] /= mag; // y
    q[2] /= mag; // z
    q[3] /= mag; // real
  }
}


/*!
  This converts euler angles to quaterions.
  \param e euler angles [roll pitch yaw]
  \param q array with quaternions [x y z real]
  \note This function checks to ensure that the norm of the quaternion
  is 1.0 and if it is not, it then makes it one.  Also it protects
  against division by zero if a correction is made.
*/
void e2q(double *e, double *q){
  double sr,sp,sy,cr,cp,cy,mag;

  sr = sin(e[0]/2.0);
  sp = sin(e[1]/2.0);
  sy = sin(e[2]/2.0);
  cr = cos(e[0]/2.0);
  cp = cos(e[1]/2.0);
  cy = cos(e[2]/2.0);

  // calc quaternions
  q[0] = cy*cp*sr-sy*sp*cr; // x
  q[1] = cy*sp*cr+sy*cp*sr; // y
  q[2] = sy*cp*cr-cy*sp*sr; // z
  q[3] = cy*cp*cr+sy*sp*sr; // real

	#if 0
  mag = sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]
	     +q[3]*q[3]);

  if((mag>1.0001 || mag<0.9999) && mag>1e-4){ // check this!!!!!!
    q[0] /= mag; // x
    q[1] /= mag; // y
    q[2] /= mag; // z
    q[3] /= mag; // real
  }
	#endif
}

/*!
  This function creates a rotation matrix from a quaternion.
  Each of the four elements are passed to this function.
  \param a Matrix to be filled.
  \param q1 x
  \param q2 y
  \param q3 z
  \param q4 real
*/
void q2R4(Matrix *a, double q1, double q2, double q3, double q4){
  if(a->rows != 3 || a->cols != 3 ){
    //printError("skew semetric double array"," ",' ',a->name,"data array");
    exit(1);
  }
#if 0
  //--- standard DCM definition ----
  a->p[0][0] = q1*q1-q2*q2-q3*q3+q4*q4;
  a->p[0][1] = 2.0*(q1*q2+q3*q4);
  a->p[0][2] = 2.0*(q1*q3-q2*q4);
  a->p[1][0] = 2.0*(q1*q2-q3*q4);
  a->p[1][1] = -q1*q1+q2*q2-q3*q3+q4*q4;
  a->p[1][2] = 2.0*(q2*q3+q1*q4);
  a->p[2][0] = 2.0*(q1*q3+q2*q4);
  a->p[2][1] = 2.0*(q2*q3-q1*q4);
  a->p[2][2] = -q1*q1-q2*q2+q3*q3+q4*q4;
#else
  //--- quaternion 3-2-1 -------------
  a->p[0][0] = 2.0*q4*q4-1.0+2.0*q1*q1;
  a->p[0][1] = 2.0*q1*q2+2.0*q4*q3;
  a->p[0][2] = 2.0*q1*q3-2.0*q4*q2;
  a->p[1][0] = 2.0*q1*q2-2.0*q4*q3;
  a->p[1][1] = 2.0*q4*q4-1.0+2.0*q2*q2;
  a->p[1][2] = 2.0*q2*q3+2.0*q4*q1;
  a->p[2][0] = 2.0*q1*q3+2.0*q4*q2;
  a->p[2][1] = 2.0*q2*q3-2.0*q4*q1;
  a->p[2][2] = 2.0*q4*q4-1.0+2.0*q3*q3;
#endif
}

/*!
  This function converts a quaternion to a rotation
  matrix.
  \param q a pointer to an array containing the quaternion.
*/
void q2R(Matrix *a, double *q){
  double q1,q2,q3,q4;

  q1 = q[0];
  q2 = q[1];
  q3 = q[2];
  q4 = q[3];

  if(a->rows != 3 || a->cols != 3 ){
    //printError("skew semetric double array"," ",' ',a->name,"data array");
    exit(1);
  }
#if 0
  a->p[0][0] = q1*q1-q2*q2-q3*q3+q4*q4;
  a->p[0][1] = 2.0*(q1*q2+q3*q4);
  a->p[0][2] = 2.0*(q1*q3-q2*q4);
  a->p[1][0] = 2.0*(q1*q2-q3*q4);
  a->p[1][1] = -q1*q1+q2*q2-q3*q3+q4*q4;
  a->p[1][2] = 2.0*(q2*q3+q1*q4);
  a->p[2][0] = 2.0*(q1*q3+q2*q4);
  a->p[2][1] = 2.0*(q2*q3-q1*q4);
  a->p[2][2] = -q1*q1-q2*q2+q3*q3+q4*q4;
#else
  //--- quaternion 3-2-1 -------------
  a->p[0][0] = 2.0*q4*q4-1.0+2.0*q1*q1;
  a->p[0][1] = 2.0*q1*q2+2.0*q4*q3;
  a->p[0][2] = 2.0*q1*q3-2.0*q4*q2;
  a->p[1][0] = 2.0*q1*q2-2.0*q4*q3;
  a->p[1][1] = 2.0*q4*q4-1.0+2.0*q2*q2;
  a->p[1][2] = 2.0*q2*q3+2.0*q4*q1;
  a->p[2][0] = 2.0*q1*q3+2.0*q4*q2;
  a->p[2][1] = 2.0*q2*q3-2.0*q4*q1;
  a->p[2][2] = 2.0*q4*q4-1.0+2.0*q3*q3;
#endif
}

/*!
  ????? not done yet ?????
*/
void setR321_Quat(Matrix *a, double roll, double pitch, double yaw){

  if(a->rows != 3 || a->cols != 3){
    // printerror
    exit(1);
  }
}

/*!
  This function multiplies two quaternions together, which
  is equivelent to multiplying two rotation matricies. Note
  that the order of the quaternions is switched for the
  multiplication.
  together.
  \n out = a * b = [b] * a
  \param a
  \param b
  \param out
*/
void quatMult(double *b, double *a, double *out){
  out[0] =  a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0];
  out[1] = -a[0]*b[2]+a[1]*b[3]+a[2]*b[0]+a[3]*b[1];
  out[2] =  a[0]*b[1]-a[1]*b[0]+a[2]*b[3]+a[3]*b[2];
  out[3] = -a[0]*b[0]-a[1]*b[1]-a[2]*b[2]+a[3]*b[3];
}


void normalize(double *a){
  double temp = sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]+a[3]*a[3]);
  a[0]/=temp;
  a[1]/=temp;
  a[2]/=temp;
  a[3]/=temp;
}

/*!
  This function returns the magnitude of a quaternion.
  \param quaternion
  \return magnitude
*/
double quatMag(double *a){
  return sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]+a[3]*a[3]);
}

/*!
This function copies the data of one quaternion in to
another quaternion.
\param in (source) quaternion to be copied.
\param out (destination)
*/
void quatCopy(double *in, double *out){
  out[0] = in[0];
  out[1] = in[1];
  out[2] = in[2];
  out[3] = in[3];

  normalize(out);
}


