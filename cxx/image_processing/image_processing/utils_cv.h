/*
 * Copyright (c) 2010-2020 Idiap Research Institute
 *
 * See file COPYING for the licence associated with this software.
 */

#ifndef UTILS_CV_H
#define UTILS_CV_H

#include <string>
#include "ip_Image.h"
#include "utils.h"
#include "ip_BoundingBox.h"


#define getCV_RGB(ip_color_element) CV_RGB(ip_color_element.r,ip_color_element.g,ip_color_element.b)

/*
inline CvScalar getCV_RGB(ip_ColorElement8u ce)
{
  return CV_RGB(ce.r,ce.g,ce.b);
}
*/


/////////////////////////// INLINE FUNCTIONS ///////////////////////////

/* returns a pixel value from a 8-bit image */
inline unsigned char pixval8u(IplImage* img, int r, int c)
{
  return ((unsigned char*)(img->imageData + r*img->widthStep))[c];
}

/* sets a pixel value in a 8-bit image */
inline void setpix8u(IplImage* img, int r, int c, unsigned char val)
{
  ( (unsigned char *)(img->imageData + img->widthStep*r) )[c] = val;
}

/* returns a pixel value from a 32-bit floating point image */
inline float pixval32f(IplImage* img, int r, int c)
{
  return ( (float*)(img->imageData + img->widthStep*r) )[c];
}

/* sets a pixel value in a floating point image */
inline void setpix32f(IplImage* img, int r, int c, float val)
{
  ( (float*)(img->imageData + img->widthStep*r) )[c] = val;
}

/* returns a pixel value from a 32-bit signed integer image */
inline int pixval32s(IplImage* img, int r, int c)
{
  return ( (int*)(img->imageData + img->widthStep*r) )[c];
}

/* sets a pixel value in a 32-bit signed integer image */
inline void setpix32s(IplImage* img, int r, int c, int val)
{
  ( (int*)(img->imageData + img->widthStep*r) )[c] = val;
}

/* sets a pixel value in a 8-bit colour BGR image */
// assume BGR colour order
inline void setpix8u3(IplImage* img, int r, int c, unsigned char red, unsigned char green, unsigned char blue)
{
  unsigned char* pimg = (unsigned char*)(img->imageData + img->widthStep*r + c*3);
  *pimg = blue;
  pimg++;
  *pimg = green;
  pimg++;
  *pimg = red;

  /*
  ( (unsigned char *)(img->imageData + img->widthStep*r) )[c*3] = blue;
  ( (unsigned char *)(img->imageData + img->widthStep*r) )[c*3+1] = green;
  ( (unsigned char *)(img->imageData + img->widthStep*r) )[c*3+2] = red;
  */
}


// assume BGR colour order
inline void getpix8u3(IplImage* img, int r, int c, unsigned char& red, unsigned char& green, unsigned char& blue)
{
  unsigned char* pimg = (unsigned char*)(img->imageData + img->widthStep*r + c*3);
  blue = *pimg;;
  pimg++;
  green = *pimg;
  pimg++;
  red = *pimg;
}






// structure for mouse handler
typedef struct params {
  CvPoint loc1;
  CvPoint loc2;
  IplImage* objects;
  const char* win_name;
  IplImage* orig_img;
  IplImage* cur_img;
} params;

void mouseHandler(int event, int x, int y, int flags, void* param);
CvRect* select_ROI( IplImage* frame , const char* window_name);
IplImage* load_frame(int i, std::string path);
IplImage* load_frame_CU(int i, std::string path);
void displayHistogram(CvHistogram* hist);

CVAPI(CvSeq*)
cvHaarDetectObjectsWithHistory( const CvArr* _img,
                     CvHaarClassifierCascade* cascade,
                     CvMemStorage* storage, double scale_factor,
                     int min_neighbors, int flags, CvSize min_size,
		     float* change_int_img, ImageProcessing::ip_Image<float>* detection_history,
		     IplImage* pruning_image, ImageProcessing::ip_BoundingBox roi);



#endif
