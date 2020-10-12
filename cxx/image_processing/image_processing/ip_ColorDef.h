/*
 * Copyright (c) 2010-2020 Idiap Research Institute
 *
 * See file COPYING for the licence associated with this software.
 */

#ifndef IP_COLOR_DEF_INC
#define IP_COLOR_DEF_INC

//#include "macros.h"
#include "utils.h"


namespace ImageProcessing {

  /** 
    
      @author Jean-marc Odobez (Jean-Marc.Odobez@idiap.ch)
      @author Daniel Gatica-Perez (gatica@idiap.ch)
  */
  //===================================================
  //
  //    class         ip_ColorElement
  //
  //     Usefull for 3 bands interleaved representation
  //    
  //
  //    
  //
  //===================================================

  template<class T>
    class ip_ColorElement
    {
    public:
      union { T one; T r; };
      union { T two; T g; };
      union { T three; T b; };
  
      ip_ColorElement(){r=(T)0.; g=(T)0.; b=(T)0.; }
      ip_ColorElement(T v){r=v; g=v; b=v; }
      ip_ColorElement(T _one, T _two, T _three){r=_one; g=_two; b=_three; }
      ~ip_ColorElement() {}
      ip_ColorElement<T> operator*=(float scalar) { r*=scalar; g*=scalar; b*=scalar; return *this; };
      ip_ColorElement<T> operator+=(float scalar) { r+=scalar; g+=scalar; b+=scalar; return *this; };
    };



  typedef ip_ColorElement<unsigned char> ip_ColorElement8u;
  typedef ip_ColorElement<int>   ip_ColorElement32s;
  typedef ip_ColorElement<float> ip_ColorElement32f;

#define ip_GREEN   ip_ColorElement8u(0,255,0)
#define ip_RED     ip_ColorElement8u(255,0,0)
#define ip_BLUE    ip_ColorElement8u(0,0,255)
#define ip_MAGENTA ip_ColorElement8u(255,0,255)
#define ip_YELLOW  ip_ColorElement8u(255,255,0)
#define ip_PURPLE  ip_ColorElement8u(192,0,192)
#define ip_WHITE   ip_ColorElement8u(255,255,255)
#define ip_BLACK   ip_ColorElement8u(0,0,0)
#define ip_GREY    ip_ColorElement8u(128,128,128)
#define ip_USERCOL ip_ColorElement8u(250,150,0)

// for inverted channels (BGR)
#define ip_iGREEN   ip_ColorElement8u(0,255,0)  
#define ip_iRED     ip_ColorElement8u(0,0,255)
#define ip_iBLUE    ip_ColorElement8u(255,0,0)
#define ip_iMAGENTA ip_ColorElement8u(255,0,255)
#define ip_iYELLOW  ip_ColorElement8u(0,255,255)
#define ip_iPURPLE  ip_ColorElement8u(192,0,192)
#define ip_iWHITE   ip_ColorElement8u(255,255,255)
#define ip_iBLACK   ip_ColorElement8u(0,0,0)
#define ip_iGREY    ip_ColorElement8u(128,128,128)
#define ip_iUSERCOL ip_ColorElement8u(0,150,250)

  //===================================================
  //
  //     Color Transform for a given pixel 
  //
  //     - RGBToGray(T2 r,T2 g,T2 b, float & gray)
  //     - RGBToHSV(T r,T g, T b, float & H, float & S, float & V)
  //
  //    
  //
  //===================================================

  //-------------------
  template <class T1>
    inline void RGBToGray(ip_ColorElement8u x,T1 & gray){
    gray = (T1)(0.299*x.r+0.587*x.g+0.114*x.b);
  } 

  //-------------------
  template <class T2,class T1>
    inline void RGBToGray(T2 r,T2 g,T2 b,T1 & gray){
    gray = (T1)(0.299*r+0.587*g+0.114*b);
  } 
  //=========================== end of RGBToGRAY
  
#define MyHSVVal(m,M,offset,a,b)  V=M; diff=V-m; diff=MAX((float)0.1,diff); \
                                 S=255.*diff/V; \
                                diff=1/diff; H=offset+(a-b)*diff; \
                               if(H<0.) H+=6.; 

  //-------------------
  template< class T>
    inline  void RGBToHSV(T r,T g, T b, float & H, float & S, float & V){

    float diff;
    // mini
    if(r<g){ // r<g
      if(r<b) { 
	if(g>b){ // r < b < g
	  MyHSVVal(r,g,2.,b,r);
	}
	else { // r < g < b
	  MyHSVVal(r,b,4.,r,g);
      }
      }
      else { //b < r < g
	MyHSVVal(b,g,2.,b,r);
      }
    }
    else { // g<r
      if (g<b) {
      if(r<b){ // g < r < b
	MyHSVVal(g,b,4.,r,g);
      } 
      else {// g < b < r
	MyHSVVal(g,r,0.,g,b);
      }
      }
      else { // b < g < r 
	MyHSVVal(b,r,0.,g,b);
      }
    }

  }
  //=========================== end of RGBToHSV



}

#endif
