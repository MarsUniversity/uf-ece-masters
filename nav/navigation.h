
#ifndef KEVINS_NAVIGATION_H
#define KEVINS_NAVIGATION_H

//--- C -----------------------
#include <stdlib.h>

//--- Sub files -------------
#include "cMathlib.h"
#include "cRk4.h"
#include "cKalmanFilter.h"
#include "nav_filter.h"

//--- Nav Defines ---------------
#define DATA_LEN 12

#define NAV_DT .041666
#define NAV_STATE_SIZE 10
#define NAV_CONTROL_SIZE 6 // was 9

//#define NAV_W_S 1.23945e-3                      // schuler freq (rad/s)
#define NAV_W_IE (7.29115e-5)                   // rotation rate earth (rad/s)
//#define NAV_R    6378137.0                      // radius earth (m)
//#define NAV_E2   (.0818191908426*.0818191908426)// eccentricity (none)
//#define NAV_H    51.816                         // height (m)
//#define NAV_LNG  (82.349352*M_PI/180.0)         // approx. longitude (rads)
//#define NAV_LAT  (29.646416*M_PI/180.0)         // approx. latitude (rads)
//#define NAV_GM  3.986005e14                     // (m3/s2)

// Eqn. 3.82 from Titterton
//#define NAV_RM NAV_R*(1.0-NAV_E2)/( pow(1.0-NAV_E2*sin(NAV_LAT)*sin(NAV_LAT),1.5) )
//#define NAV_RN NAV_R/( sqrt(1.0-NAV_E2*sin(NAV_LAT)*sin(NAV_LAT)) )
// initial position on earth (ECEF)
//#define NAV_X_INIT (NAV_RN+NAV_H)*cos(NAV_LAT)*cos(NAV_LNG)
//#define NAV_Y_INIT (NAV_RN+NAV_H)*cos(NAV_LAT)*sin(NAV_LNG)
//#define NAV_Z_INIT (NAV_RN*(1.0-NAV_E2)+NAV_H)*sin(NAV_LAT)
//---------------------------

//--- KF Defines ----------------
#define KF_STATE_SIZE 9
#define KF_STATE_Z 9
#define KF_CONTROL_SIZE 1
#define KF_DT .041666
//----------------------------

enum Coordinate_Frames {ECI,ECEF,LGV};

typedef struct{
  float a[3];   // acceleration
  float w[3];   // angular velocity
  float gps[3]; // lat, lon, alt
  float r,p,y;    //roll and pitch
  int satNum;
} nav_sensor_t;

typedef struct{
  //cMatrix R_bn;//(3,3,"R_bn nav");
  //cMatrix R_ne;//(3,3,"R_ne nav");
  //cMatrix R_be;//(3,3,"R_be nav");
  cMatrix R_eb;//(3,3,"R_eb nav");
} nav_rotation_t;

typedef struct{
  float bias[3];  // current bias of accelerometers
  float drift[3]; // current drift of gyros

  cMatrix A;
  cMatrix B;
} model_t;




int main(int argc, char *argv[]);



#endif









