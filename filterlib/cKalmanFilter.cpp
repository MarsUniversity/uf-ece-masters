#include "cKalmanFilter.h"

//static char errMsg[KF_ERROR_STRING_SIZE];

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
cKalmanFilter::cKalmanFilter(void){
	try {
		F.setName("KF F");
		B.setName("KF B");
		Q.setName("KF Q");
		R.setName("KF R");
		H.setName("KF H");
		P.setName("KF P");
		K.setName("KF K");

		x.setName("KF x");
		xmin.setName("KF xmin");
		//z.setName("KF z");
		//u.setName("KF u");
	}
	KF_CATCH_ERROR(FATAL);
}

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
void cKalmanFilter::init(const cMatrix &f, const cMatrix &b){
	try {
		F.resize(f.r,f.c); F=f;
		B.resize(b.r,b.c); B=b;
		H.resize(f.c,f.c); H.eye();
		Q.resize(f.c,f.c); Q.eye();
		R.resize(f.c,f.c); R.eye();
		P.resize(f.c,f.c);
		//Pmin.resize(q.r,q.c);
		K.resize(f.r,f.c);

		x.resize(f.r);
		xmin.resize(f.r);
		//z.resize(f.c);
		//u.resize(b.c);
		//xmin.resize(f.r);
  //Pmin.resize(q.r,q.c);
	}

	KF_CATCH_ERROR(FATAL);

}


/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
void cKalmanFilter::init(const cMatrix &f, const cMatrix &b, const cMatrix &h){
	try{
		F.resize(f.r,f.c); F=f;
		B.resize(b.r,b.c); B=b;
		H.resize(h.r,h.c); H=h;
		Q.resize(f.c,f.c); Q.eye();
		R.resize(h.r,h.r); R.eye();
		P.resize(f.c,f.c);
		//Pmin.resize(q.r,q.c);
		K.resize(f.r,h.r);

		x.resize(f.r);
		xmin.resize(f.r);
		//z.resize(h.r);
		//u.resize(b.c);
		//xmin.resize(f.r);
		//Pmin.resize(q.r,q.c);
	}
	KF_CATCH_ERROR(FATAL);

}


/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
void cKalmanFilter::init(int usize,const cMatrix &h, const cMatrix &q, const cMatrix &r){
	try {
		F.resize(h.c,h.c);
		B.resize(h.c,usize);
		H.resize(h.r,h.c); H=h;
		Q.resize(q.r,q.c); Q=q;
		R.resize(r.r,r.c); R=r;
		P.resize(q.r,q.c);
		//Pmin.resize(q.r,q.c);
		K.resize(q.r,q.c);

		x.resize(h.c);
		xmin.resize(h.c);
		//z.resize(h.r);
		//u.resize(usize);
	}
	KF_CATCH_ERROR(FATAL);

}
