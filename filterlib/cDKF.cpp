#include "cKalmanFilter.h"


static char errMsg[KF_ERROR_STRING_SIZE];

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
cDKF::cDKF(cMatrix &f, cMatrix &b){
  init(f,b);
}

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
cDKF::cDKF(cMatrix &f, cMatrix &b, cMatrix &h){
  init(f,b,h);
}


/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
void cDKF::update(cVector &z, cVector &u){
  try {
    //cout<<F<<P<<Pmin<<Q<<x<<xmin<<K<<B<<z<<u;
    P = F*P*F.trans()+Q;
    xmin = F*x+B*u;
    K = P*H.trans()*(H*P*H.trans()+R).inv();
    x=xmin+K*(z-H*xmin);
    P = P-K*H*P;
    //printf("void cDKF::update(void)\n");
  }
	KF_CATCH_ERROR(FATAL)
}

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
cCKF::cCKF(void){
  ;
}

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
void cCKF::init(cMatrix &f, cMatrix &b, cMatrix &h, ml_data idt){
  cKalmanFilter::init(f,b,h);
  try {
    xx.resize(F.r);
    k1x.resize(F.r);
    k2x.resize(F.r);
    k3x.resize(F.r);
    k4x.resize(F.r);

    pp.resize(P.r,P.c);
    k1p.resize(P.r,P.c);
    k2p.resize(P.r,P.c);
    k3p.resize(P.r,P.c);
    k4p.resize(P.r,P.c);

    dt = idt;
  }
  KF_CATCH_ERROR(FATAL)

}


//#define xmodel(x,u) (F*x+B*u)
//#define pmodel(p) (F*p+p*F.trans()+Q-P*H.trans()*R.inv()*H*P)

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
cVector& cCKF::xmodel(cVector& x, cVector& u, cVector& z){
	//cout<<z<<x;
  return F*x+B*u+K*(z-H*x);
}

cVector& cCKF::xmodel(cVector& x, cVector& u){
	//cout<<z<<x;
	return F*x+B*u; //+K*(z-H*x);
}

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
inline cMatrix& cCKF::pmodel(cMatrix& p){
  return F*p+p*F.trans()-P*H.trans()*R.inv()*H*P+Q;
}

/////////////////////////////////////////////////////////////////
/// Updates the current estimation, covarience matrix (P), and
/// the Kalman gain (K).
/// \param z measurement of current state
/// \param iu deterministic input (i.e. control effort for system)
/////////////////////////////////////////////////////////////////
void cCKF::update(cVector &z, cVector &u){
  try {

    // time update error covarience
    k1p=dt*pmodel(P);
    pp = P+.5*k1p;
    k2p=dt*pmodel(pp);
    pp = P+.5*k2p;
    k3p=dt*pmodel(pp);
    pp = P+k3p;
    k4p=dt*pmodel(pp);
    P = P +(1.0/6.0)*(k1p + 2.0*k2p + 2.0*k3p + k4p );

    K = P*H.trans()*R.inv();
	 //cout<<K;

    // time update state
    k1x=dt*xmodel(x,u,z);
    xx = x+.5*k1x;
    k2x=dt*xmodel(xx,u,z);
    xx = x+.5*k2x;
    k3x=dt*xmodel(xx,u,z);
    xx = x+k3x;
    k4x=dt*xmodel(xx,u,z);
    x = x +(1.0/6.0)*(k1x + 2.0*k2x + 2.0*k3x + k4x );
    //cout<<H*x;
	 //cout<<u;
  }
	KF_CATCH_ERROR(FATAL) ;
}

void cCKF::update(cVector &u){
	try {

		// time update error covarience
		pp = P;
		k1p=dt*pmodel(pp);
		pp = P+.5*k1p;
		k2p=dt*pmodel(pp);
		pp = P+.5*k2p;
		k3p=dt*pmodel(pp);
		pp = P+k3p;
		k4p=dt*pmodel(pp);
		P = P +(1.0/6.0)*(k1p + 2.0*k2p + 2.0*k3p + k4p );

		//K = P*H.trans()*R.inv();
		//cout<<K;

		// time update state
		xx = x;
		k1x=dt*xmodel(xx,u);
		xx = x+.5*k1x;
		k2x=dt*xmodel(xx,u);
		xx = x+.5*k2x;
		k3x=dt*xmodel(xx,u);
		xx = x+k3x;
		k4x=dt*xmodel(xx,u);
		x = x +(1.0/6.0)*(k1x + 2.0*k2x + 2.0*k3x + k4x );
		//cout<<H*x;
	 //cout<<u;
	}
	KF_CATCH_ERROR(FATAL) ;
}


/////////////////////////////////////////////////////////////////
/// Constructor
/////////////////////////////////////////////////////////////////
cCEKF::cCEKF(cVector& (*func)(cVector&,cVector&,cVector&),
	     cMatrix &h, int u, ml_data idt){
  cCKF::init(zeros(h.r,h.r),zeros(h.r,u),h,idt);
  f = func;
}

/////////////////////////////////////////////////////////////////
/// Updates the current estimation, covarience matrix (P), and
/// the Kalman gain (K). Since the system is assumed to be
/// nonlinear, the system (F) is linearized first, then passed
/// to cCKF::update().
/// \param z measurement of current state
/// \param iu deterministic input (i.e. control effort for system)
/// \sa cCKF::update()
/////////////////////////////////////////////////////////////////
void cCEKF::update(cVector &z,cVector &u){
  linearize(dt,u);
  //cCKF::update(z,u);
	try {

		// time update error covarience
		k1p=dt*pmodel(P);
		pp = P+.5*k1p;
		k2p=dt*pmodel(pp);
		pp = P+.5*k2p;
		k3p=dt*pmodel(pp);
		pp = P+k3p;
		k4p=dt*pmodel(pp);
		P = P +(1.0/6.0)*(k1p + 2.0*k2p + 2.0*k3p + k4p );

		K = P*H.trans()*R.inv();
		//cout<<K;

		// time update state
		k1x=dt*xmodel(x,u,z);
		xx = x+.5*k1x;
		k2x=dt*xmodel(xx,u,z);
		xx = x+.5*k2x;
		k3x=dt*xmodel(xx,u,z);
		xx = x+k3x;
		k4x=dt*xmodel(xx,u,z);
		x = x +(1.0/6.0)*(k1x + 2.0*k2x + 2.0*k3x + k4x );
		//cout<<H*x;
	 //cout<<u;
	}
	KF_CATCH_ERROR(FATAL) ;
}


/////////////////////////////////////////////////////////////////
/// This function linearizes nonlinear system dynamics.
/// \param dt step size used in linearization.
/////////////////////////////////////////////////////////////////
void cCEKF::linearize(ml_data dt,cVector &u){
	int size = F.c;
	int j;
	static cVector linstate(F.c,"kf linstate");
	static cVector ta(F.c,"kf ta");
	static cVector tb(F.c,"kf tb");
	static cVector dist(F.c,"kf dist"); // i don't need this!!!

	//printf("lineraize\n");
	try {
		for(j=0; j<size;j++){  // linearize the system
			linstate = x;
			linstate.p[j] += dt;
			ta = f(linstate,u,dist);

			linstate = x;
			linstate.p[j] -= dt;
			tb = f(linstate,u,dist);

			linstate = (ta - tb)/(2.0*dt);
			F.setC(linstate.p,j);
		}
		//cout<<F<<ta<<tb<<linstate<<dt;
	}
	KF_CATCH_ERROR(FATAL);

	//cout<<F;
}

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
inline cVector& cCEKF::xmodel(cVector& x, cVector& u, cVector& z){
	static cVector dist(F.c,"kf dist"); // i don't need this!!!
	//printf("hi\n");
	return f(x,u,dist)+K*(z-H*x);
}
