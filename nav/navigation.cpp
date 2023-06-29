
#include "navigation.h"
#include <unistd.h>
#include <string.h>
#include "fileio.h"
#include <iostream>
#include <math.h>

using namespace std;

model_t *nav = NULL;      // holds nav equations
cRK4 *rk = NULL;          // integrates nav equations
//cDKF kf;                  // kalman filter
cCKF kf;
nav_rotation_t *r = NULL;

#define D2R (M_PI/180.0)
#define R2D (180.0/M_PI)

float start_pos[3]={0,0,0};
// std::vector<float*> data;

/*!
This function handles the termination signal.
 */
void nav_handleTermSignal(void *none){
	printf(" - Nav is exiting \n");
	exit(1);
}

////////////////////////////////////////////
/// dddd.dddd = min2deg(ddddmmmm.mmmm)
/// all units are degrees
////////////////////////////////////////////
float min2deg(float gps){
	float deg=floor(gps/100);
	float min=fmod(gps,100);
	return deg+(min/60.0);
}

void Rnb(cMatrix &m, float r, float p, float y){
	//r=r+M_PI;  // don't i already take care of this??

	float sr = sin(r);
	float cr = cos(r);
	float sp = sin(p);
	float cp = cos(p);
	float sy = sin(y);
	float cy = cos(y);

	m.p[0] = cp*cy;
	m.p[1] = sr*sp*cy-cr*sy;
	m.p[2] = sr*sy+cr*sp*cy;
	m.p[3] = cp*sy;
	m.p[4] = cr*cy+sr*sp*sy;
	m.p[5] = cr*sp*sy-sp*cy;
	m.p[6] = -sp;
	m.p[7] = sr*cp;
	m.p[8] = cr*cp;
}

void Rne(cMatrix &m, float lat, float lon){
	float slat = sin(lat);
	float clat = cos(lat);
	float slon = sin(lon);
	float clon = cos(lon);

	m.p[0] = -slat*clon;
	m.p[1] = -slat*slon;
	m.p[2] = clat;
	m.p[3] = -slon;
	m.p[4] = clat;
	m.p[5] = 0;
	m.p[6] = -clat*clon;
	m.p[7] = -clat*slon;
	m.p[8] = -slat;

}

void Reb(cMatrix &m, float lat, float lon, float r, float p, float y){
	static cMatrix en(3,3),nb(3,3);
	Rne(en,lat,lon);
	en=en.trans();
	Rnb(nb,r,p,y);

	m=en*nb;
}

void Rec(cMatrix &m, float x, float y, float z, float longitude){
	float Pn=sqrt(x*x+y*y);
	float P=sqrt(Pn*Pn+z*z);

	float latc=atan2(z/P,Pn/P);
	float lon=M_PI/180.0*longitude;

	float slat = sin(latc);
	float clat = cos(latc);
	float slon = sin(lon);
	float clon = cos(lon);

	m.p[0] = -clon*slat;
	m.p[1] = slon;
	m.p[2] = clon*clat;
	m.p[3] = -slon*slat;
	m.p[4] = -clon;
	m.p[5] = slon*clat;
	m.p[6] = clat;
	m.p[7] = 0.0;
	m.p[8] = slat;


}

void setR123_Euler(cMatrix &a, float roll, float pitch, float yaw){
	float cr,cp,cy;
	float sr,sp,sy;

	if(a.r != 3 || a.c != 3){
		printf("ERROR in setR123_Euler: the rotation matrix MUST be 3x3\n");
		exit(1);
	}

	sr = sin(roll);
	sp = sin(pitch);
	sy = sin(yaw);
	cr = cos(roll);
	cp = cos(pitch);
	cy = cos(yaw);

	a.p[0] = cp*cy;
	a.p[1] = sr*sp*cy-cr*sy;
	a.p[2] = cr*sp*cy+sr*sy;
	a.p[3] = cp*sy;
	a.p[4] = sr*sp*sy+cr*cy;
	a.p[5] = cr*sp*sy-sr*cy;
	a.p[6] = -sp;
	a.p[7] = sr*cp;
	a.p[8] = cr*cp;

	//printMatrix(a);
}


void e2q(const float roll,const float pitch,const float yaw, float *q){
	float cr = cos(roll/2);
	float sr = sin(roll/2);
	float cp = cos(pitch/2);
	float sp = sin(pitch/2);
	float cy = cos(yaw/2);
	float sy = sin(yaw/2);

	q[0] = cy*cp*sr-sy*sp*cr; // x
	q[1] = cy*sp*cr+sy*cp*sr; // y
	q[2] = sy*cp*cr-cy*sp*sr; // z - check this neg sign!!!
	q[3] = cy*cp*cr+sy*sp*sr; // real
}

/////////////////////////////////////////////////
/// converts quaternions to euler angles.
/////////////////////////////////////////////////
void q2e(float *q, float *out){
	float a11=2*q[3]*q[3]-1+2*q[0]*q[0];
	float a12=2*q[0]*q[1]+2*q[3]*q[2];
	float a13=2*q[0]*q[2]-2*q[3]*q[1];
	float a23=2*q[1]*q[2]+2*q[3]*q[0];
	float a33=2*q[3]*q[3]-1+2*q[2]*q[2];

	out[1] = asin(-a13);
	out[0] = atan2(a23/cos(out[1]),a33/cos(out[1]));
	out[2] = atan2(a12/cos(out[1]),a11/cos(out[1]));

	if (out[2] < 0) out[2] = 2*M_PI+out[2];
}

/////////////////////////////////////////////////
/// This function converts gps positions into
/// the ECEF reference frame.
/// \warning Make sure the gps coordinates are
/// first converted from degrees/minutes to
/// degrees in radians
////////////////////////////////////////////////
void gps2ecef(float gps[3], float out[3]){
	float a=6379137;       // meters
	float f=1.0/298.257224;

	float lat=gps[0];
	float lon=2*M_PI-gps[1];
	float h=gps[2];

	float C=1.0/sqrt(cos(lat)*cos(lat) + (1.0-f)*(1.0-f)*sin(lat)*sin(lat));
	float S=(1.0-f)*(1.0-f)*C;

	out[0]=(a*C+h)*cos(lat)*cos(lon); //x
	out[1]=(a*C+h)*cos(lat)*sin(lon); //y
	out[2]=(a*S+h)*sin(lat); //z

	//printf("gps: %f %f %f\n",lat*R2D,lon*R2D,h);
 //printf("out: %f %f %f\n",out[0],out[1],out[2]);
}

void saveData(const string& filename, const vector<float*> data){
	int i,j;
	FILE *fp = openFile(filename.c_str(),"w+");

	for(i=0;i<data.size()-1;i++){

		fprintf(fp,"%f ",NAV_DT*i);
		for(j=0;j<DATA_LEN;j++) {
			fprintf(fp,"%f ",data[i][j]);
		}
		fprintf(fp,"\n");
	}
	fclose(fp);
}

// average data so reduce the effect of bad data measurements
#define SUM_LEN 30
float findAve(float *array,int index, float current){
	float mean = 0;
	int j=0;

	array[index%SUM_LEN] = current;
	for(j=0;j<SUM_LEN;j++){
		mean += array[j]/SUM_LEN;
	}

	return mean;
}

cVector& updateKF(nav_sensor_t *s){
	int i,j;
	static float p[3] = {0,0,0};
	static float e[3] = {0,0,0};
	static cMatrix tmpM(3,3,"tmpM");
	static cVector z(KF_STATE_Z,"updateKF::z");
	static int index=0;
	static float gps_old[3];
	static float mean[3];
	static cMatrix I(KF_STATE_SIZE,KF_STATE_SIZE,"updateKF::I");
	float dt = KF_DT;
	static float w_iee[3]={0.0,0.0,NAV_W_IE};
	static float Vold[3][SUM_LEN];
	static float Pold[3][SUM_LEN];

	//--- fill A -----------------------------
	kf.F.set(r->R_eb*tmpM.skew(s->a),0,6);
	kf.F.set(r->R_eb*tmpM.skew(s->w),6,6);

	//--- if discrete ---------------------
	if (0) {
		kf.F.set( -2.0*skew(w_iee),0,0);
		kf.F.set(-1.0*skew(w_iee)*skew(w_iee),0,3);
		kf.F.set(eye(3),3,0);
		kf.F = I+kf.F*dt+kf.F*kf.F*dt*dt*.5+kf.F*kf.F*kf.F*dt*dt*dt/6.0;
	}

	//--- Z --------------------------
	//--- position -------------------
	gps2ecef(s->gps,p);
	if(index == 0){
		cout<<"reset updateKF\n";
		gps_old[0] = p[0];
		gps_old[1] = p[1];
		gps_old[2] = p[2];

		for(i=0;i<3;i++){
			memset(Vold[i],0,SUM_LEN*sizeof(float));
			memset(Pold[i],0,SUM_LEN*sizeof(float));
			for(j=0;j<SUM_LEN;j++){
				Pold[i][j] = p[i];
			}
		}

	}

	for(i=0;i<3;i++){
		mean[i] = findAve(Pold[i],index,p[i]);
	}

	z.p[3] = rk->x.p[3] - mean[0];
	z.p[4] = rk->x.p[4] - mean[1];
	z.p[5] = rk->x.p[5] - mean[2];

	//--- velocity -------------------
	for(i=0;i<3;i++){
		mean[i] = findAve(Vold[i],index,(p[i] - gps_old[i])/KF_DT);
		gps_old[i] = p[i];
	}
	z.p[0] = rk->x.p[0] - mean[0];
	z.p[1] = rk->x.p[1] - mean[1];
	z.p[2] = rk->x.p[2] - mean[2];

	// attitude
	q2e( &(rk->x.p[6]), e );
	z.p[6] = e[0] - s->r;
	z.p[7] = e[1] - s->p;
	z.p[8] = 0; //e[2] - s->y;

	index++;

	return z;
}


/*!
This function sets the current velocity to zero,
 sets the current nav position to the gps position,
 and sets the current attitude.
 */
void reset(nav_sensor_t *s){
	static float p[3] = {0,0,0};
	static float q[4];
	int i;

	// velocity
	rk->x.p[0] = 0;
	rk->x.p[1] = 0;
	rk->x.p[2] = 0;

	// position
	gps2ecef(s->gps,p);
	rk->x.p[3] = p[0];
	rk->x.p[4] = p[1];
	rk->x.p[5] = p[2];
	//printf("reset(gps): %f %f %f\n",s->gps[0],s->gps[1],s->gps[2]);
	//printf("reset(ecef): %f %f %f\n",p[0],p[1],p[2]);

	// attitude
	e2q(s->r,s->p,s->y,q);
	rk->x.p[6] = q[0];
	rk->x.p[7] = q[1];
	rk->x.p[8] = q[2];
	rk->x.p[9] = q[3];
}


nav_rotation_t* initRotation(){
	nav_rotation_t *o = new nav_rotation_t;

	if(o == NULL){
		printf("ERROR: couldn't create nav_rotation_t\n");
		exit(1);
	}

	//o->R_bn.resize(3,3);o->R_bn.setName("R_bn nav");
	//o->R_ne.resize(3,3);o->R_ne.setName("R_ne nav");
	//o->R_be.resize(3,3);o->R_be.setName("R_be nav");
	o->R_eb.resize(3,3);o->R_eb.setName("R_eb nav");

	return o;
}

/*!
This function initializes the navigational structure ( nav_t )
 and all of its Matricies and Vectors. It also sets up the static
 parts of the A and B matrix (i.e. the elements that don't change
										during run time) and the gravity model.
 */
model_t* initNavigationECEF(void){
	model_t *nav = new model_t;
	cMatrix o(3,3);
	cMatrix oo(3,3);
	float w_iee[3]={0.0,0.0,NAV_W_IE};

	if(nav == NULL){
		printf("ERROR: couldn't create model_t\n");
		exit(1);
	}

	nav->A.resize(NAV_STATE_SIZE,NAV_STATE_SIZE);
	nav->A.setName("A nav");
	nav->B.resize(NAV_STATE_SIZE,NAV_CONTROL_SIZE);
	nav->B.setName("B nav");

	//--- A ----------------------
	o.skew(w_iee);
	oo = -1.0*o*o;
	o=-2.0*o;
	nav->A.clear();
	nav->A.set(o,0,0);
	nav->A.set(oo,0,3);
	o.eye();
	nav->A.set(o,3,0);

	return nav;
}


/*!
This function updates the state space matricies during run-time.
 Currently the system uses ECEF reference frame, and thus the
 only state space matrix that needs updating is the B matrix.
 \param A pointer to the state transition Matrix.
 \param B pointer to the gain Matrix.
 \param u pointer the control effort Vector.
 */
void updateNav(const nav_sensor_t *s){
	static float a[4];
	static float bias[] = {-0.357,1.08,0.0};

	//--- A -------------------------
	nav->A.set(.5*skew(s->w,4),6,6); // float check skew(w,4) works!!

	//--- B -------------------------
	nav->B.set(r->R_eb,0,0);
	nav->B.set(r->R_eb,0,3);

	//--- U -------------------------
	rk->u.p[0] = 0;
	rk->u.p[1] = 0;
	rk->u.p[2] = -9.7;
	rk->u.p[3] = -s->a[0] - bias[0];
	rk->u.p[4] = -s->a[1] - bias[1];
	rk->u.p[5] = -s->a[2] - bias[2];

	//--- Correct Nav State ---
	// correct nav equations
 // velocity
	rk->x.p[0] -= kf.x.p[0];
	rk->x.p[1] -= kf.x.p[1];
	rk->x.p[2] -= kf.x.p[2];

	// position
	rk->x.p[3] -= kf.x.p[3];
	rk->x.p[4] -= kf.x.p[4];
	rk->x.p[5] -= kf.x.p[5];

	// attitude
	//e2q(kf.x.p[6],kf.x.p[7],kf.x.p[8],a);
	//rk->x.p[6] -= a[0];
	//rk->x.p[7] -= a[1];
	//rk->x.p[8] -= a[2];
	//rk->x.p[9] -= a[3];


	static float e[3];
	q2e(&(rk->x.p[6]),e);
	e2q(s->r,s->p,e[2],a);
	rk->x.p[6] = a[0];
	rk->x.p[7] = a[1];
	rk->x.p[8] = a[2];
	rk->x.p[9] = a[3];

}

/*
 This is a model of the entire system which contains
 both the position/velocity and oritentation stuff
 represented in quaternions [x y z real].
 \param x states [xdot ydot zdot x y z q1 q2 q3 q4].
 \param u accelerations measured from imu
 [accel_x accel_y accel_z roll_rate pitch_rate yaw_rate].
 \param sys the output of the model.
 */
cVector& nav_model(cVector &x, cVector &u,  cVector &dist){
	cVector *sys = cBaseMath::getTmp(x.size);

	*sys = nav->A*x+nav->B*u;
	return *sys;
}

/*!
This function gets data for the navigation equations.
 Some measurements come from the IMU, while some come from
 the raw_data_nav in shared memory. This is because it
 needs an initial condition and to track the current states.
 \param x state vector.
 \param u inputs for the navigation equations.
 \todo get rid of old filtering stuff
 */
void navGetData(char *line, nav_sensor_t *s){
	float none;

	//printf("%s\n",line);
#if 1
	sscanf(line,"%f %f %f %f %f %f %f %f %f %f %f %f %f %f %d",
		  &s->a[0],&s->a[1],&s->a[2],     // accelerations
		  &s->w[0],&s->w[1],&s->w[2],     // angular rates
		  &s->r,&s->p,&s->y,              // attitude
		  &none,&none,       // not used
		  &s->gps[0],&s->gps[1],&s->gps[2], //gps
		  &s->satNum);               // number of gps sats

	// convert deg to rad
	s->r*=D2R;
	s->p*=D2R;
	s->y*=D2R;

	// convert gps
	s->gps[0] = D2R*min2deg(s->gps[0]);
	s->gps[1] = D2R*min2deg(s->gps[1]);
	//s->gps[2] = D2R*min2deg(s->gps[2]);

#else
	//--- imu ---
	s->a[0] = 0;
	s->a[1] = 0;
	s->a[2] = 9.7;

	//--- rates ---
	s->w[0] = 0;
	s->w[1] = 0;
	s->w[2] = 0;

	//--- attitude ---
	s->r = 0;
	s->p = 0;
	s->y = 0;

	//--- gps ---
	s->gps[0] = D2R*min2deg(8200.0);
	s->gps[1] = D2R*min2deg(2900.0);
	s->gps[2] = 40.0;

#endif

}


/*!
This function is called when all operations on the data is
 complete and it needs to be returned to shared memory. The
 position and velocity are transformed from ECEF to nav.
 \param n navigation_t
 \param v value_t
 \todo don't make new vectors here!
 */
void navPutData(int index, nav_sensor_t *s, vector<float*> data){
	static float att[3] = {0,0,0};
	static float g[3] = {0,0,0};
	static float time = 0;
	static float p[3];
	int i = 0;
	float *d = NULL;

	// create new row
	d = new float[DATA_LEN];

	// velocity
	d[i++] = rk->x.p[0];
	d[i++] = rk->x.p[1];
	d[i++] = rk->x.p[2];

	// position
	d[i++] = rk->x.p[3] - start_pos[0];
	d[i++] = rk->x.p[4] - start_pos[1];
	d[i++] = rk->x.p[5] - start_pos[2];
	//printf(" position: %f %f %f\n",d[3],d[4],d[5]);

	// attitude
	q2e( &(rk->x.p[6]), att );
	d[i++] = att[0];
	d[i++] = att[1];
	d[i++] = att[2];

	// gps positions
	gps2ecef(s->gps,g);
	d[i++] = g[0] - start_pos[0];
	d[i++] = g[1] - start_pos[1];
	d[i++] = g[2] - start_pos[2];

	data.push_back(d);

}

inline void updateR(nav_sensor_t *s){
	//float roll = s->r+M_PI; // body and nav are flipped
	//float pitch = s->p;
	//float yaw = s->y;
	//float x = rk->x.p[3];
	//float y = rk->x.p[4];
	//float z = rk->x.p[5];

	Reb(r->R_eb,s->gps[0],s->gps[1],s->r,s->p,s->y);
}

/*!
This is the main function which loops and takes raw accelerations and
 gyro rates and turns them into velocity, position, and attudes.
 */
int main(int argc, char *argv[]){
	int i,j;
	int index;
	int num;
	char line[255];
	FILE *fp = NULL;
	cVector z(KF_STATE_Z,"z");
	cVector u(KF_CONTROL_SIZE,"u");
	nav_sensor_t *s;
  std::vector<nav_sensor_t*> sensor;
  std::vector<float*> data;

	printf(" + Start Nav\n");

	initKF(kf);    // create kalman filter
	nav = initNavigationECEF();
	rk = new cRK4(nav_model,NAV_DT,NAV_STATE_SIZE,NAV_CONTROL_SIZE);
	r = initRotation();

	if(nav == NULL || rk == NULL || r == NULL){
		printf("\n\tcould not allocate nav[%d] or rk[%d] or rotation[%d]\n",
			nav,rk,r);
		exit(1);
	}

	fp = openFile("data/data.dat","r");
	if(fp == NULL) exit(1);

	// need to zero out kf states
	//memset(s,0,sizeof(nav_sensor_t));
	//reset(s);
	z.clear();
	u.clear();
	index = 0;
	kf.x.clear();
	kf.P.clear();
	kf.K.clear();
	float gps_old[3] = {0,0,0};

	printf(" Reading in Data\n");
	while((num = readALine(fp,line,255,'#')) != -1){
		s = new nav_sensor_t;
		navGetData(line,s);
		sensor.push_back(s);
	}

	printf(" Doing Navigation\n");
	// do the navigation stuff
	int start = 4400;
	for(index=start;index<sensor.size();index++){
		try{
			if(index%500 == 0) {
				printf("loop[%d]\n",index);
				//printf("gps: %f %f %f\n",s->gps[0],s->gps[1],s->gps[2]);
			}

			s = sensor[index];

			// initialize stuff
			if (index == start ){
				float p[3];
				gps2ecef(s->gps,p);
				start_pos[0] = p[0];
				start_pos[1] = p[1];
				start_pos[2] = p[2];
				printf("start pos: %f %f %f\n",p[0],p[1],p[2]);
				reset(s);
				gps_old[0] = s->gps[0];
				gps_old[1] = s->gps[1];
				gps_old[2] = s->gps[2];
			}

			updateR(s);

			//printf("gps: %f %f %f\n",s->gps[0],s->gps[1],s->gps[2]);
      //printf("attitude: %f %f %f\n",s->r,s->p,s->y);

			//--- EKF -------------
			z = updateKF(s);
			//cout<<z;

			if(gps_old[0] == s->gps[0]){
				//z.clear();
				kf.update(z,u);
			}
			else {
				gps_old[0] = s->gps[0];
				kf.update(z,u);
				//cout<<kf.K;
			}

			//--- Nav Solution ----
			updateNav(s);
			rk->integrate();
			//cout<<rk->x;

			//--- done ------------
			navPutData(index,s,data);

			if(index > 5700 ) break;
		}
		catch(cMLError &e){
			//cout<<e;
			exit(1);
		}
	}
	fclose(fp);
	fp = NULL;

	// save data
	printf(" -<< Saving Data >>-\n");
	saveData("data/results.dat", data);
	printf(" - Done \n");

	return 0;
}






