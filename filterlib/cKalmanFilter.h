
#ifndef CKALMANFILTER_H
#define CKALMANFILTER_H

#include <iostream>
#include <string>
#include "cMathlib.h"

#define KF_ERROR_STRING_SIZE 255
#define KF_CATCH_ERROR(type)                   \
catch( cMLError &e){                           \
	cMLError error(cMLError::type,__PRETTY_FUNCTION__);     \
	error+=e;                                   \
	throw error;                                \
}                                              \
catch ( ... ) {                                \
	cMLError error(cMLError::type,__PRETTY_FUNCTION__);     \
	error+="unknown error";                     \
	throw error;                                \
}



/////////////////////////////////////////////////////////////////
/// This is the base class for the various types of Kalman filters
/// and is never meant to be used directly. It contains all of the
/// common elements for filtering.
/////////////////////////////////////////////////////////////////
class cKalmanFilter {
 public:
  cKalmanFilter(void);
  void init(const cMatrix&,const cMatrix&);
  void init(const cMatrix&,const cMatrix&,const cMatrix&);
  void init(int,const cMatrix&,const cMatrix&,const cMatrix&);

  inline void setF(const cMatrix &m){F=m;}
  inline void setB(const cMatrix &m){B=m;}
  inline void setP(const cMatrix &m){P=m;}
  inline void setQ(const cMatrix &m){Q=m;}
  inline void setQ(ml_data d){Q=d*Q.eye();}
  inline void setR(const cMatrix &m){R=m;}
  inline void setR(ml_data d){R=d*R.eye();}
  //inline void setU(const cVector &v){u=v;}
  //inline void setZ(const cVector &v){z=v;}
  inline cVector& getEstimateState(void){return x;}
  inline const cVector& getEstimateOutput(void){return H*x;}

  //protected:
  cMatrix F,B,H,Q,R,P,K;
  cVector x,xmin;//,u,z;
  //cVector& (*f)(cVector&,cVector&,cVector&);

protected:
  //void linearize(ml_data);
  void c2d(void);
};

/////////////////////////////////////////////////////////////////
/// This is a discrete kalman filter (DKF).
///
/// \code
/// #include "cMathlib.h"
/// #include "cKalmanFilter.h"
///
/// void main(void){
///   cDKF kf;
/// }
/// \endcode
/////////////////////////////////////////////////////////////////
class cDKF : public cKalmanFilter {
 public:
  cDKF(void){;}
  cDKF(cMatrix&,cMatrix&);
  cDKF(cMatrix&,cMatrix&,cMatrix&);

  void update(cVector&,cVector&);
};

/////////////////////////////////////////////////////////////////
/// This is a continuos-discrete kalman filter (CDKF).
///
/// \code
/// #include "cMathlib.h"
/// #include "cKalmanFilter.h"
///
/// void main(void){
/// }
/// \endcode
/////////////////////////////////////////////////////////////////
class cCDKF : public cKalmanFilter {
 public:
  cCDKF(void);
  void init(cMatrix&,cMatrix&,cMatrix&,ml_data);
  void update(cVector&,cVector&); // time and measurement update
  void update(cVector&); // time update

 protected:
	  cVector& xmodel(cVector&,cVector&,cVector&);
  cVector& xmodel(cVector&,cVector&);
  cMatrix& pmodel(cMatrix&);
  cMatrix Fd,Bd;
  cMatrix k1p,k2p,k3p,k4p,pp;
  cVector k1x,k2x,k3x,k4x,xx;
  ml_data dt;
};


/////////////////////////////////////////////////////////////////
/// This is a continous kalman filter (CKF)
///
/// \code
/// #include "cMathlib.h"
/// #include "cKalmanFilter.h"
///
/// void main(void){
///   cMatrix F; // also need to setup matrix
///   cVector z(2),u(2),xest(2);
///   cCKF kf;
///
///   kf.init(F,eye(2),eye(2),.01);
///
///   for(;;){
///      z = getSensorData();
///      u = getControlEffort();
///      kf.update(z,u);
///      xest = kf.getEstimateState();
///   }
/// }
/// \endcode
/////////////////////////////////////////////////////////////////
class cCKF : public cKalmanFilter {
 public:
  cCKF(void);
  void init(cMatrix&,cMatrix&,cMatrix&,ml_data);

  void update(cVector&,cVector&);
  void update(cVector&);
 protected:
  cVector& xmodel(cVector&,cVector&,cVector&);
  cVector& xmodel(cVector&,cVector&);
  cMatrix& pmodel(cMatrix&);
  cMatrix k1p,k2p,k3p,k4p,pp;
  cVector k1x,k2x,k3x,k4x,xx;
  ml_data dt;
};


/////////////////////////////////////////////////////////////////
///
///
/// \code
/// #include "cMathlib.h"
/// #include "cKalmanFilter.h"
///
/// void main(void){
/// }
/// \endcode
/////////////////////////////////////////////////////////////////
class cCEKF : public cCKF {
 public:
  cCEKF(cVector& (*func)(cVector&,cVector&,cVector&),cMatrix&,int,ml_data);

  void update(cVector&,cVector&);

  cVector& (*f)(cVector&,cVector&,cVector&);

protected:
	  void linearize(ml_data,cVector&);
  cVector& xmodel(cVector&,cVector&,cVector&);
};

#endif
