
#ifndef CRAND_H
#define CRAND_H

#include "cMathlib.h"

#include <cstdlib>
#include <cstdlib>
#include <cstdio>

#include<iostream>

/*!
 A random number generator that returns numbers between 0 and 1.
 */
class cRand {
public:
	cRand(int=256);
	~cRand(void);
	ml_data get(void);
	void initSeed(void);
	void openRandDev(void);
	void openRandDev(char*);
protected:
   ml_data max;
	unsigned long *seed;
	int index;
	FILE *file;
	int bufferSize;
};

#endif
