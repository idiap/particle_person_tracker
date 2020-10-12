/*
 * Copyright (c) 2010-2020 Idiap Research Institute
 *
 * See file COPYING for the licence associated with this software.
 */

#include "image_processing/Parameters.h"



// Standard C++ library header
#include <sys/time.h>
#include <math.h>
#include <stdio.h>
#include <errno.h>
#include <time.h>
#include <stdarg.h>
#include <stdlib.h>
#include <unistd.h>
#include <vector>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
using namespace std;

// User defined
#include "image_processing/utils.h"





/////////////////////////// CONVERT INTEGER TO STRING ////////////////////////

string itos(int i)	// convert int to string
	{
		stringstream s;
		s << i;
		return s.str();
	}


////////////////////////////// ARCTAN FUNCTION ///////////////////////////////
double arctan( double x)
{

int sign = 100;

if (x < 0.0)
{
x = -1 * x;
sign = -100;
}

if ( x <=1  )
{
x = x / (1 + x * x* 0.28);
} else {
x = 1.570796325 - x / ( x*x + 0.28);
}

return (sign * x );
}


////////////////////// INV SQRT FUNCTION /////////////////////////////////////////

/*float InvSqrt(float x)  32 bit implementation
{
    float xhalf = 0.5f*x;
    int i = *(int*)&x;
    i = 0x5f37a86 - (i >> 1);
    x = *(float*)&i;
    x = x*(1.5f - xhalf*x*x);
    return x;
}
*/



// float InvSqrt( float f )
// {
//   float fhalf = 0.5f*f;
// #ifdef ARCH64
//   asm
//   (
//     "mov %1, %%eax;"
//     "sar %%eax;"
//     "mov $0x5F3759DF, %%ebx;"
//     "sub %%eax, %%ebx;"
//     "mov %%ebx, %0"
//     :"=g"(f)
//     :"g"(f)
//     : "%eax", "%ebx"
//   );
// #else
//   asm
//   (
//     "mov %1, %%eax;"
//     "sar %%eax;"
//     "pushl %%ebx;"
//     "mov $0x5F3759DF, %%ebx;"
//     "sub %%eax, %%ebx;"
//     "mov %%ebx, %0;"
//     "popl %%ebx"
//     :"=g"(f)
//     :"g"(f)
//     : "%eax"
//   );
// #endif
//   f *= 1.5f - fhalf*f*f;
//   return f;
// }


void quicksort(int* array, int top, int bottom)
{
      // top = subscript of beginning of vector being considered
      // bottom = subscript of end of vector being considered
      // this process uses recursion - the process of calling itself
     int middle;
     if (top < bottom)
    {
          middle = partition(array, top, bottom);
          quicksort(array, top, middle);   // sort top partition
          quicksort(array, middle+1, bottom);    // sort bottom partition
     }
     return;
}


//Function to determine the partitions
// partitions the array and returns the middle index (subscript)
int partition(int* array, int top, int bottom)
{
     int x = array[top];
     int i = top - 1;
     int j = bottom + 1;
     int temp;
     do
     {
           do
           {
                  j--;
           }while (x >array[j]);

          do
         {
                 i++;
          } while (x <array[i]);

          if (i < j)
         {
                 temp = array[i];    // switch elements at positions i and j
                 array[i] = array[j];
                 array[j] = temp;
         }
     }while (i < j);
     return j;           // returns middle index
}


// sorts a map (represented by two arrays: index-value) using the values
void quicksort_map(int* index, float* values, int top, int bottom)
{
      // top = subscript of beginning of vector being considered
      // bottom = subscript of end of vector being considered
      // this process uses recursion - the process of calling itself
     int middle;
     if (top < bottom)
    {
          middle = partition_map(index, values, top, bottom);
          quicksort_map(index, values, top, middle);   // sort top partition
          quicksort_map(index, values, middle+1, bottom);    // sort bottom partition
     }
     return;
}


//Function to determine the partitions
// partitions the array and returns the middle index (subscript)
int partition_map(int* index, float* values, int top, int bottom)
{
     float x = values[top];
     int i = top - 1;
     int j = bottom + 1;
     float temp;
     do
     {
           do
           {
                  j--;
           }while (x >values[j]);

          do
         {
                 i++;
          } while (x <values[i]);

          if (i < j)
         {
                 temp = values[i];    // switch elements at positions i and j
                 values[i] = values[j];
                 values[j] = temp;
                 temp = index[i];    // switch indices at positions i and j
                 index[i] = index[j];
                 index[j] = temp;
         }
     }while (i < j);
     return j;           // returns middle index
}


float calc_mean(float* array, int nb_entries)
{
  if (nb_entries==0)
    return 0;

  float sum=0;
  for(int i=0; i<nb_entries; i++)
    sum+=array[i];
  return sum/nb_entries;
}

float calc_stddev(float* array, int nb_entries)
{
  float mean = calc_mean(array, nb_entries);

  float sum=0;
  for(int i=0; i<nb_entries; i++)
    sum+=(array[i]-mean)*(array[i]-mean);
  return sqrt(sum/nb_entries);
}


double gettimeofday_in_s()
{
  timeval tv;
  if (gettimeofday(&tv, NULL))
    return 0;
  //printf("%f %f           %f\n", (double)tv.tv_sec, (double)tv.tv_usec, (double)tv.tv_sec + (double)tv.tv_usec*1e-6);
  return (double)tv.tv_sec + (double)tv.tv_usec*1e-6;
}
