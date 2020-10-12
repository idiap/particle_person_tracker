/*
 * Copyright (c) 2010-2020 Idiap Research Institute
 *
 * See file COPYING for the licence associated with this software.
 */

#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <cv.h>

//using namespace std;

/*
 *  pantheios logging levels
 */
/*
SEV_EMERGENCY 	system is unusable
SEV_ALERT 	action must be taken immediately
SEV_CRITICAL 	critical conditions
SEV_ERROR 	error conditions
SEV_WARNING 	warning conditions
SEV_NOTICE 	normal but significant condition
SEV_INFORMATIONAL 	informational
SEV_DEBUG 	debug-level messages
*/

typedef unsigned char uchar;

// Basic definitions

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif
#ifndef MIN
#define MIN(x,y) ( ( (x) < (y) )? (x) : (y) )
#endif
#ifndef MAX
#define MAX(x,y) ( ( (x) > (y) )? (x) : (y) )
#endif
#ifndef ABS
#define ABS(x) ( ( (x) > 0 )? (x) : (-x) )
#endif

#define ROUND_TO_INT(x) ( ((x)>0)? (int((x)+.5)) : (int((x)-.5)) )
#define ROUND_TO_INTP(x) (int((x)+.5))
#define ROUND_TO_INTN(x) (int((x)-.5))


#define FLT_EPS 0.000000000000000000000000000000001


//fprintf(stderr, "Assertion failed: %s. %s\n", #condition, message);
	//log_ALERT("Assertion failed: ", #condition, message, "\n");
	//pantheios::pantheios_logprintf("Assertion failed: %s\n", #condition, message);

#ifdef DEBUG
  #define ASSERT(condition,message) { \
      if (! (condition)) { \
	pantheios::log_ALERT("Assertion failed: ", #condition, message, "\n"); \
	exit(EXIT_FAILURE); \
      } \
  }
#else
  //not exiting in OPT (release version)
  #define ASSERT(condition,message) { \
      if (! (condition)) { \
	pantheios::log_ALERT("Assertion failed: ", #condition, message, "\n"); \
      } \
  }
#endif




///////////// exponential approximations /////////////////////

inline float fastexp3(float x) {
    return (6+x*(6+x*(3+x)))*0.16666666f;
}

inline float fastexp4(float x) {
    return (24+x*(24+x*(12+x*(4+x))))*0.041666666f;
}

inline float fastexp5(float x) {
    return (120+x*(120+x*(60+x*(20+x*(5+x)))))*0.0083333333f;
}

inline float fastexp6(float x) {
    return (720+x*(720+x*(360+x*(120+x*(30+x*(6+x))))))*0.0013888888f;
}

inline float fastexp7(float x) {
    return (5040+x*(5040+x*(2520+x*(840+x*(210+x*(42+x*(7+x)))))))*0.00019841269f;
}

inline float fastexp8(float x) {
    return (40320+x*(40320+x*(20160+x*(6720+x*(1680+x*(336+x*(56+x*(8+x))))))))*2.4801587301e-5;
}

inline float fastexp9(float x) {
  return (362880+x*(362880+x*(181440+x*(60480+x*(15120+x*(3024+x*(504+x*(72+x*(9+x)))))))))*2.75573192e-6;
}

/***************************
* Schraudolph's algorithm: *
***************************/
/*
static union
{
	double d;
	struct
	{
//#ifdef LITTLE_ENDIAN
//		int j, i;
//#else
		int i, j;
//#endif
	} n;
} _eco;
*/


#define EXP_A (1048576/M_LN2) /* 2^20/LN(2) use 1512775 for integer version */
#define EXP_C 60801 /* see text for choice of c values */
#define EXP(y) (_eco.n.i = EXP_A*(y) + (1072693248 - EXP_C), _eco.d)

std::string itos(int i);
double arctan( double x);
// float InvSqrt(float f);


void quicksort(int* array, int top, int bottom);
int partition(int* array, int top, int bottom);
void quicksort_map(int* index, float* values, int top, int bottom);
int partition_map(int* index, float* values, int top, int bottom);

float calc_mean(float* array, int nb_entries);
float calc_stddev(float* array, int nb_entries);

double gettimeofday_in_s();

#endif
