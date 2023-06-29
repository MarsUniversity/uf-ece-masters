
#include "cKalmanFilter.h"


/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
cCDKF::cCDKF(void) {
	pp.setName("KF tmp P");
	xx.setName("KF tmp X");
}

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
void cCDKF::init(cMatrix &f, cMatrix &b, cMatrix &h, ml_data idt){
	try {
cKalmanFilter::init(f,b,h);

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


/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
void cCDKF::update(cVector &z, cVector &u){
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

		K = P*H.trans()*R.inv();

		// time update state
		xx = x;
		k1x=dt*xmodel(xx,u,z);
		xx = x+.5*k1x;
		k2x=dt*xmodel(xx,u,z);
		xx = x+.5*k2x;
		k3x=dt*xmodel(xx,u,z);
		xx = x+k3x;
		k4x=dt*xmodel(xx,u,z);
		x = x +(1.0/6.0)*(k1x + 2.0*k2x + 2.0*k3x + k4x );
		//std::cout<<K<<P;
	}
	KF_CATCH_ERROR(FATAL)

}

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
inline cVector& cCDKF::xmodel(cVector& x, cVector& u, cVector& z){
	return F*x+B*u+K*(z-H*x);
}

/////////////////////////////////////////////////////////////////
/// \todo double check this!!!
/////////////////////////////////////////////////////////////////
inline cVector& cCDKF::xmodel(cVector& x, cVector& u){
	return F*x+B*u+K*(-H*x); // this doesn't seem right
}


/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
inline cMatrix& cCDKF::pmodel(cMatrix& p){
	return F*p+p*F.trans()-P*H.trans()*R.inv()*H*P+Q;
}

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
void cCDKF::update(cVector &u){
	try {
		
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
	}
	KF_CATCH_ERROR(FATAL)

}

