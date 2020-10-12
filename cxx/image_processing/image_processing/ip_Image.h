/*
 * Copyright (c) 2010-2020 Idiap Research Institute
 *
 * See file COPYING for the licence associated with this software.
 */

#ifndef IP_IMAGE_INC
#define IP_IMAGE_INC

#include <stdio.h>

#ifdef USE_PANTHEIOS
  #include <pantheios/pantheios.hpp>
  using namespace pantheios;
#else
  #define log_ERROR printf
#endif
//#include "general.h"


namespace ImageProcessing {

  //-----
  // Definition of some macros useful for smaller program writing

#define ForAllPtsInImage(Image,li,co) for(li=0;li<Image.nbLines();li++) \
                                      for(co=0;co<Image.nbColumns();co++)

#define ForAllPtsInImageP(Image,li,co) for(li=0;li<Image->nbLines();li++) \
                                      for(co=0;co<Image->nbColumns();co++)


  //-----
  
  /** 
   
      @author Jean-marc Odobez (Jean-Marc.Odobez@idiap.ch)
      @author Daniel Gatica-Perez (gatica@idiap.ch)
  */
  //===================================================
  //
  //    class         ip_Image
  //
  //    Base virtual class for image representation
  //
  //    Correspond to one band of whatever type
  //    
  //    Note : to simplify representation of color image
  //           with interleaved format (i.e. pixel values
  //           one after the other), we define
  //           a 3 band structure in ip_ColorDef.h
  //
  //    2-May-2002 : we add the definition of an image
  //                 allocator
  //
  //===================================================

  //===================================================
  template <class Type>
    class  ip_Image {
    public:
    //----------------
    // informations members
    //
    // line index : from 0 to nbLines-1
    // col  index : from 0 to nbColumns-1
    virtual int nbLines()=0;
    virtual int nbColumns()=0;
   
    //----------------
    // managing memory and initialisation

    virtual void allocateMem(int nbli,int nbco)=0; // (re) allocate memory, if necessary
    virtual void freeMem()=0;     // destroy memory (should also be done
    // by destructor)

    virtual void init(Type value)=0;  // initialisation to value
    virtual void init(int nbli,int nbco,Type value)=0;  // (re) allocate + initialisation

    // virtual void permute(ip_Image<Type> & tab); // permutation of images (without
    // recopy of content

    //------------------
    //    virtual ip_Image<Type> & operator=(ip_Image<Type> & Ima)=0;

    //------------------
    // accessing elements
    // Note that normally, the () operator and value fonction are faster
    // than the setVal or getVal functions

    virtual Type & operator()(int li,int co) = 0; // M(i,j)=a; or a=M(i,j);
    virtual Type & value(int li,int co) = 0;    // M.value(i,j)=a; or a=M.value(i,j);
    virtual Type & operator()(int li,int co) const = 0;
    virtual Type & value(int li,int co) const = 0; // M.value(i,j)=a; or a=M.value(i,j);
    virtual void setVal(int li,int co, Type val) =0;
    virtual Type getVal(int li,int co) =0;
    
    //------------------
    // sweeping through image
    // Setting image line, and then accessing through
    // column number

    virtual void setLine(int li) = 0;
    virtual Type & valueCol(int co) = 0; 
    virtual Type & valueCol(int co) const = 0; 

    //----------------------------------------------------------
    // ADDING NEW FUNCTIONNALITIES INDEPENDANT OF REPRESENTATION
    // 
    // 
    
    //----------------------------------
    // Copy an image into current image
    virtual void putImage(ip_Image<Type> & InImage,int startLine,int startColumn);
        
    //----------------------------------
    // Test whether a position is inside image
    virtual bool in(int li,int co);
    
    //--------------------------
    virtual ~ip_Image(){}
    
    /// Multiply each pixel with a scalar value
    virtual void multiply(float scalar);
    virtual void add(float scalar);
    virtual void setROI(int topleft_x, int topleft_y, int width, int height)
    {
      log_ERROR("ip_Image::setROI() not implemented.\n");
    };
    virtual void resetROI()
    {
      log_ERROR("ip_Image::resetROI() not implemented.\n");
    };
    virtual void fill(Type value)=0;
    virtual void AND(ip_Image<Type>* image)
    {
      log_ERROR("ip_Image::AND() not implemented.\n");
    };
    virtual void max(ip_Image<Type>* tmp_image)
    {
      log_ERROR("ip_Image<T>::max() not implemented");
    }
    virtual void min(ip_Image<Type>* tmp_image)
    {
      log_ERROR("ip_Image<T>::max() not implemented");
    }

    virtual int save(const char* filename)=0;
  };

  //===================================================
  //
  //    class         ip_ImageAllocator
  //
  //    Base virtual class for allocating new images
  //
  //
  //===================================================
  template <class Type>
    class ip_ImageAllocator {
    public:
    virtual ip_Image<Type> * newIpImage()=0;
  };



  //===================================================
  // DEFINITION OF GENERAL PURPOSE FUNCTION
  //===================================================


  //----------------------------------
  // Copy an image into current image
  template<class Type>
    void ip_Image<Type>::putImage(ip_Image<Type> & InImage,int startLine,int startColumn)
    {
      int li,co;
      li=0;
      while(li<InImage.nbLines() && li+startLine<nbLines()){
	setLine(li+startLine);
	InImage.setLine(li);
	co=0;
	while(co<InImage.nbColumns() && co+startColumn<nbColumns()){
	  valueCol(co+startColumn)=InImage.valueCol(co);
	  co++;
	}
	li++;
      }
    }
  
  //----------------------------------
  // 
  template<class Type>
    bool ip_Image<Type>::in(int li,int co)
    {
      return (li>=0 && li<nbLines() && co>= 0 && co<nbColumns());
    }
  

  template<class Type>
    void ip_Image<Type>::multiply(float scalar)
    {
      int nli = nbLines();
      int nco = nbColumns();
      for(int li=0; li<nli; li++)
      {
	setLine(li);
	for(int co=0; co<nco; co++)
	  valueCol(co)*=scalar;
      }
    }
   
  template<class Type>
    void ip_Image<Type>::add(float scalar)
    {
      int nli = nbLines();
      int nco = nbColumns();
      for(int li=0; li<nli; li++)
      {
	setLine(li);
	for(int co=0; co<nco; co++)
	  valueCol(co)+=scalar;
      }
    }
   
}

#endif
