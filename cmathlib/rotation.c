

#include "rotation.h"

/*!
This function updates the Earth-Centered Inertial
to Local Geodetic Vertical rotation matrix. The
geodetic longitude is converted to the inertial
longitude by the function.
\param a the rotation matrix to fill, which must be a 3x3.
\param lng is the geodetic longitude.
\param lat is the geodetic latitude.
\param time is the time since the navigation has began.
*/
void setR_iv(Matrix *a, float lng, float lat, float time){
  float lng_i;
  float clng, slng,clat,slat;

  if(a->rows != 3 || a->cols != 3){
    printf("ERROR in setR_iv: the rotation matrix MUST be 3x3\n");
    exit(1);
  }

  lng_i = lng+NAV_W_IE*time;
  clng = cos(lng_i); slng = sin(lng_i);
  clat = cos(lat); slat = sin(lat);
  a->p[0][0] = -clng*slat;
  a->p[0][1] = -slng*slat;
  a->p[0][2] = clat;
  a->p[1][0] = slng;
  a->p[1][1] = -clng;
  a->p[1][2] = 0.0;
  a->p[2][0] = clng*clat;
  a->p[2][1] = slng*clat;
  a->p[2][2] = slat;
}

/*!
This function updates the Earth-Centered Earth-Fixed
to Local Geodetic Vertical rotation matrix.Thisrotation
matrix is the same as R_iv except that the longitude
is not updated with time.
\sa updateR_iv
*/
void setR_ev(Matrix *a, float lng, float lat){
  if(a->rows != 3 || a->cols != 3){
    printf("ERROR in setR_ev: the rotation matrix MUST be 3x3\n");
    exit(1);
  }
  setR_iv(a,lng,lat,0.0);
}

/*!
This function performs the rotation from body
coordinates to inertial coordinates.
\param a rotation matrix to be set.
\param roll
\param pitch
\param yaw
*/
void setR123_Euler(Matrix *a, float roll, float pitch, float yaw){
  float cr,cp,cy;
  float sr,sp,sy;

  if(a->rows != 3 || a->cols != 3){
    printf("ERROR in setR123_Euler: the rotation matrix MUST be 3x3\n");
    exit(1);
  }

  sr = sin(roll);
  sp = sin(pitch);
  sy = sin(yaw);
  cr = cos(roll);
  cp = cos(pitch);
  cy = cos(yaw);

  a->p[0][0] = cp*cy;
  a->p[0][1] = sr*sp*cy-cr*sy;
  a->p[0][2] = cr*sp*cy+sr*sy;
  a->p[1][0] = cp*sy;
  a->p[1][1] = sr*sp*sy+cr*cy;
  a->p[1][2] = cr*sp*sy-sr*cy;
  a->p[2][0] = -sp;
  a->p[2][1] = sr*cp;
  a->p[2][2] = cr*cp;

  //printMatrix(a);
}

/*!
	This function creates a rotation matrix from ECI to LGV
	reference frames.
	\param a Matrix to be filled.
	\param pn
	\param pu
*/
void setR_cv(Matrix *a,float pn, float pu){
  float p = sqrt(pn*pn+pu*pu);

  if(a->rows != 3 || a->cols != 3){
    printf("ERROR in setR_cv: the rotation matrix MUST be 3x3\n");
    exit(1);
  }

  a->p[0][0] = pu;
  a->p[0][1] = 0;
  a->p[0][2] = pn;
  a->p[1][0] = 0;
  a->p[1][1] = p;
  a->p[1][2] = 0;
  a->p[2][0] = -pn;
  a->p[2][1] = 0;
  a->p[2][2] = pu;

  matrixDivS(a,p,a);
}

/*!
	This function creates a rotation matrix from nav to
	ECEF reference frame.
	\param a Matrix to be filled.
	\param lat the latitude in radians.
	\param lng the longitude in radians.
	\todo fix this transpose thing.
*/
void setR_ne(Matrix *a,double lat, double lng){
  double clat,clng,slat,slng;
  Matrix *b=initMatrix(3,3,"test");

  if(a->rows != 3 || a->cols != 3){
    printf("ERROR in setR_ne: the rotation matrix MUST be 3x3\n");
    exit(1);
  }

  clat = cos(lat);
  slat = sin(lat);
  clng = cos(lng);
  slng = sin(lng);

  a->p[0][0] = -slat*clng;
  a->p[0][1] = -slat*slng;
  a->p[0][2] = clat;
  a->p[1][0] = -slng;
  a->p[1][1] = clng;
  a->p[1][2] = 0.0;
  a->p[2][0] = -clat*clng;
  a->p[2][1] = -clat*slng;
  a->p[2][2] = -slat;

  matrixTrans(a,b);
  matrixCopy(b,a);
}


void R2rpy(Matrix *a, double *r, double *p, double *y){
  double mm;
}















