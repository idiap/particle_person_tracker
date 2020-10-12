/*
 * Copyright (c) 2010-2020 Idiap Research Institute
 *
 * See file COPYING for the licence associated with this software.
 */

#ifndef IPCV_IMAGE_INC
#define IPCV_IMAGE_INC

/*
#include <pantheios/pantheios.hpp>
#include <pantheios/inserters.hpp>
*/
#include "ip_ColorDef.h"
#include "cv.h"
#include "highgui.h"
#include "ip_Image.h"
#include "utils_cv.h"

//using namespace pantheios;

namespace ImageProcessing {

  //#define DEBUG_CLASSES 0

  //-----

  /**

      @author Jean-marc Odobez (Jean-Marc.Odobez@idiap.ch)
      @author Daniel Gatica-Perez (gatica@idiap.ch)
  */
  //===================================================
  //
  //    class         ipcv_Image
  //
  //    class for one band image interface using opencv
  //       image representation
  //
  //    Supported types :
  //       - 5 types of ipl librairy (char, unsigned char,
  //          short int, int , float)
  //       - one band of these types, or three bands,
  //         cf ip_ColorElement
  //
  //    Remarks : The iplImage representation is
  //              left available so that it can be used
  //              with opencv.
  //              When using opencv functions that may
  //              change the image size and storage place,
  //              use the refresh() function to be sure
  //              that the interface is in concordance
  //              with the current storage room of the
  //              iplImage structure.
  //
  //===================================================

  //==================================================
  //
  //    function needed to automatically set the
  //    the type of image element (needed for the
  //    image representation with the opencv image
  //    format).
  //
  //==================================================
  template<class T>
    void  TypeIpl(T a,int & depth,int & nbbands){
    printf("Not an IPL type \n"); fflush(stdout); exit(1); return;  }

  void  TypeIpl(uchar a,int & depth,int & nbbands);
  void  TypeIpl(char a,int & depth,int & nbbands);
  void  TypeIpl(short int a,int & depth,int & nbbands);
  void  TypeIpl(int a,int & depth,int & nbbands);
  void  TypeIpl(float a,int & depth,int & nbbands);

  template<class T>
    void  TypeIpl(ip_ColorElement<T>  a,int & depth,int & nbbands){
    TypeIpl(a.r,depth,nbbands); nbbands=3;   }

  //===================================================
  //===================================================
  //===================================================

  template <class Type>
    class  ipcv_Image
    :     public ip_Image<Type> {
    public:
    IplImage    *m_pImage;

    // redundant with parameter in image,
    // but speeds up things
    // POTENTIAL DANGER :
    // SOME OPENCV DO REALLOCATION
    // --> DATA AND STEPLINE MAY THUS
    // --> NOT BE "UP-TO-DATE"

    private:
    int         m_iDepth; // to perform floatlocation
    int         m_iNbBands;
    char       *m_pData;
    int         m_iStepLine;
    // for sweeping
    Type       *m_pCurrLine;

    public:

    //----------------
    // constructors
    //
    //
    ipcv_Image();
    ipcv_Image(int nbli,int nbco);
    ipcv_Image(int nbli,int nbco, Type value);

    //----------------
    // informations members
    //
    // line index : from 0 to nbLines-1
    // col  index : from 0 to nbColumns-1

    virtual inline int nbLines(){return m_pImage->height;}
    virtual inline int nbColumns(){ return m_pImage->width; }

    //----------------
    // managing memory and initialisation


    void allocateMem(int nbli,int nbco); // (re) allocate memory, if necessary
    void freeMem();     // destroy memory (should also be done
    // by destructor)

    virtual void init(Type value);  // initialisation to value
    virtual void init(int nbli,int nbco,Type value);  // (re) allocate + initialisation

    // virtual void permute(ip_Image<Type> & tab); // permutation of images (without
    // recopy of content


    //------------------
    virtual ipcv_Image<Type> & assign(ip_Image<Type> * Ima);
    virtual ipcv_Image<Type> & operator=(ip_Image<Type> & Ima);
    virtual ipcv_Image<Type> & operator=(ipcv_Image<Type> & Ima);

    //------------------
    // accessing elements
    // Note that normally, the () operator and value fonction are faster
    // than the setVal or getVal functions

    // M(i,j)=a; or a=M(i,j);
    virtual inline Type & operator()(int li,int co){
      return *((Type *)(m_pData + m_iStepLine * li) +co);}

    // M.value(i,j)=a; or a=M.value(i,j);
    virtual inline Type & value(int li,int co){
      return *((Type *)(m_pData + m_iStepLine * li) +co);}

    virtual inline Type & operator()(int li,int co) const {
      return *((Type *)(m_pData + m_iStepLine * li) +co);}

    virtual inline Type & value(int li,int co) const {
      return *((Type *)(m_pData + m_iStepLine * li) +co);}

    virtual inline void setVal(int li,int co, Type val){
      *((Type *)(m_pData + m_iStepLine * li) +co)=val;}

    virtual inline Type getVal(int li,int co){
      return *((Type *)(m_pData + m_iStepLine * li) +co);
    }

    //------------------
    // sweeping through image
    // Setting image line, and then accessing through
    // column number

    inline void setLine(int li){
      m_pCurrLine=(Type *)(m_pImage->imageData+m_pImage->widthStep*li);
    }

    inline Type & valueCol(int co){
      return *(m_pCurrLine+co);
    }

    inline Type & valueCol(int co) const {
      return *(m_pCurrLine+co);
    }

    //------------------
    // the refresh function
    inline void refresh(){
      if(m_pImage->nChannels!=m_iNbBands){
	printf("Attention danger \n");
      }

      m_iNbBands=m_pImage->nChannels;
      m_pData=m_pImage->imageData;
      m_iStepLine=m_pImage->widthStep;
    }

    //------------------
    virtual ~ipcv_Image(){
      freeMem();
      if (m_pImage)
	cvReleaseImageHeader(&m_pImage);
    }

    void multiply(float scalar);
    void add(float scalar);
    void setROI(int topleft_x, int topleft_y, int width, int height);
    void resetROI();
    void fill(Type value);  /// like init() but only for (cv)scalar types and quicker
    void AND(ip_Image<Type>* image);
    void max(ip_Image<Type>* image);
    void min(ip_Image<Type>* image);
    int save(const char* filename);
  };

  //=======================================================
  //
  //
  //
  //=======================================================

  //----------------
  template <class Type>
  ipcv_Image<Type>::ipcv_Image(){
    CvSize s;

#ifdef  DEBUG_CLASSES
    printf("ipcv_Image : default constructor : 1 \n"); fflush(stdout);
#endif

    TypeIpl((Type)0,m_iDepth,m_iNbBands);
    s.width=0; s.height=0;
    m_pImage=cvCreateImageHeader(s,m_iDepth,m_iNbBands);
    m_pImage->imageData=NULL;
    allocateMem(0,0);
  }

  //----------------
  template <class Type>
    ipcv_Image<Type>::ipcv_Image(int nbli,int nbco){

#ifdef  DEBUG_CLASSES
    printf("ipcv_Image : constructor (nbli=%d,nbco=%d)  : 1 \n",nbli,nbco); fflush(stdout);
#endif

    CvSize s;
    TypeIpl((Type)0,m_iDepth,m_iNbBands);
    s.width=nbco; s.height=nbli;
    m_pImage=cvCreateImageHeader(s,m_iDepth,m_iNbBands);
    m_pImage->imageData=NULL;
    allocateMem(nbli,nbco);
  }

  template <class Type>
  ipcv_Image<Type>::ipcv_Image(int nbli,int nbco, Type value)
  {
    CvSize s;
    TypeIpl((Type)0,m_iDepth,m_iNbBands);
    s.width=nbco; s.height=nbli;
    m_pImage=cvCreateImageHeader(s,m_iDepth,m_iNbBands);
    m_pImage->imageData=NULL;
    allocateMem(nbli,nbco);
    init(value);
  }


  //----------------
  // managing memory and initialisation

  template <class Type>
  void ipcv_Image<Type>::allocateMem(int nbli,int nbco){

    bool alloc=true;

#ifdef  DEBUG_CLASSES
    printf("ipcv_Image : allocateMem  : 1 (nbli=%d nbco=%d)\n",nbli,nbco); fflush(stdout);
#endif

    if(m_pImage->imageData!=NULL  && nbli == m_pImage->height && nbco==m_pImage->width){
#if   DEBUG_CLASSES > 2
      printf("ipcv_Image : allocateMem : 2 \n"); fflush(stdout);
#endif
      return;
    }

    if(m_pImage->imageData==NULL){
#if   DEBUG_CLASSES > 2
      printf("ipcv_Image : allocateMem : 3 \n"); fflush(stdout);
#endif
      //      cvInitImageHeader(m_pImage,cvSize(nbco,nbli),m_iDepth,m_iNbBands,
      //		IPL_ORIGIN_TL,IPL_ALIGN_DWORD,1);
      cvInitImageHeader(m_pImage,cvSize(nbco,nbli),m_iDepth,m_iNbBands,
 			IPL_ORIGIN_TL,IPL_ALIGN_DWORD);
    }
    else {
      if(nbli != m_pImage->height || nbco != m_pImage->width){ // should be true
#if   DEBUG_CLASSES > 2
	printf("ipcv_Image : allocateMem : 4 \n"); fflush(stdout);
#endif
	// deallocate
	freeMem();
	//	cvInitImageHeader(m_pImage,cvSize(nbco,nbli),m_iDepth,m_iNbBands,
	//	  IPL_ORIGIN_TL,IPL_ALIGN_DWORD,1);
	cvInitImageHeader(m_pImage,cvSize(nbco,nbli),m_iDepth,m_iNbBands,
			  IPL_ORIGIN_TL,IPL_ALIGN_DWORD);
	m_pImage->imageData=NULL;
      }
      else{
#if   DEBUG_CLASSES > 2
	printf("ipcv_Image : allocateMem : 5 \n"); fflush(stdout);
#endif
	alloc=false;
      }

    }


    if(alloc && nbli>0 && nbco>0){
#if   DEBUG_CLASSES > 2
      printf("ipcv_Image : allocateMem  : 6 \n"); fflush(stdout);
#endif
      // cvCreateImageData(this->m_pImage);
      cvCreateData(this->m_pImage);
      m_pData=m_pImage->imageData;
      m_iStepLine=m_pImage->widthStep;
    }
  }

  //----------------
  template <class Type>
  void ipcv_Image<Type>::freeMem(){     // destroy image memory
#ifdef  DEBUG_CLASSES
    printf("ipcv_Image : freeMem  : 1 \n"); fflush(stdout);
#endif
    cvReleaseData(this->m_pImage);
    // cvReleaseImageData(this->m_pImage);
    m_pImage->imageData=NULL;
    m_pData=NULL;
    m_iStepLine=0;
  }

  //----------------
  template<class Type>
    ipcv_Image<Type> & ipcv_Image<Type>::assign(ip_Image<Type> * Ima){

#ifdef  DEBUG_CLASSES
    printf("ipcv_Image : assign ip_Image : 1 \n"); fflush(stdout);
#endif

    if(this != Ima){
      int li,co;
#if   DEBUG_CLASSES > 2
      printf("ipcv_Image : assign  : 2 \n"); fflush(stdout);
#endif
      (*this).allocateMem(Ima->nbLines(),Ima->nbColumns());
      for(li=0;li<nbLines();li++){
	Ima->setLine(li);
	setLine(li);
	for(co=0;co<nbColumns();co++)
	  valueCol(co)=Ima->valueCol(co);
      }
    }
    return *this;
  }

  //----------------
  template<class Type>
    ipcv_Image<Type> & ipcv_Image<Type>::operator=(ip_Image<Type> & Ima){

#ifdef  DEBUG_CLASSES
    printf("ipcv_Image : operator= ip_Image : 1 \n"); fflush(stdout);
#endif

    assign(&Ima);
    return *this;
  }

  //----------------
  template<class Type>
    ipcv_Image<Type> & ipcv_Image<Type>::operator=(ipcv_Image<Type> & Ima){

#ifdef  DEBUG_CLASSES
    printf("ipcv_Image : operator= ipcv_Image : 1 \n"); fflush(stdout);
#endif

    //assign(&Ima);
    m_iDepth = Ima.m_iDepth;
    cvCopy(Ima.m_pImage, m_pImage);
    refresh();
    return *this;
  }

  //----------------
  template<class Type>
  void ipcv_Image<Type>::init(Type val){  // initialisation to value
    int li,co;
    for(li=0;li<nbLines();li++){
      setLine(li);
      for(co=0;co<nbColumns();co++)
	valueCol(co)=val;
    }
  }

  //----------------
  template<class Type>
  void ipcv_Image<Type>::init(int nbli,int nbco,Type val){  // (re) allocate + initialisation
    allocateMem(nbli,nbco);
    init(val);
  }


  template <class Type>
  void ipcv_Image<Type>::multiply(float scalar)
  {
    cvScale(m_pImage, m_pImage, scalar);
  }

  template <class Type>
  void ipcv_Image<Type>::add(float scalar)
  {
    cvAddS(m_pImage, cvScalar(scalar), m_pImage);
  }


  template <class Type>
  void ipcv_Image<Type>::setROI(int topleft_x, int topleft_y, int width, int height)
  {
    /*
    log_DEBUG("ipcv_Image::setROI: ", integer(topleft_x), " ", integer(topleft_y), " ", integer(width), " ", integer(height));
    fflush(stdout);
    fflush(stderr);
    */
    cvSetImageROI(m_pImage, cvRect(topleft_x, topleft_y, width, height));
  }

  template <class Type>
  void ipcv_Image<Type>::resetROI()
  {
    cvResetImageROI(m_pImage);
  }


  template <class Type>
  inline void ipcv_Image<Type>::fill(Type value)
  {
    cvSet(m_pImage, cvScalar(value));
  }

  template <>
  inline void ipcv_Image<ip_ColorElement8u>::fill(ip_ColorElement8u value)
  {
    cvSet(m_pImage, getCV_RGB(value));
  }

  template <class Type>
  void ipcv_Image<Type>::AND(ip_Image<Type>* image)
  {
    ipcv_Image<Type>* cvi = dynamic_cast<ipcv_Image<Type>*>(image);
    if (cvi)
      cvAnd(m_pImage, cvi->m_pImage, m_pImage);
    else
      log_ERROR("ipcv_Image::AND(): types do not match");
  }

  template <class Type>
  void ipcv_Image<Type>::max(ip_Image<Type>* image)
  {
    ipcv_Image<Type>* cvi = dynamic_cast<ipcv_Image<Type>*>(image);
    if (cvi)
      cvMax(m_pImage, cvi->m_pImage, m_pImage);
    else
      log_ERROR("ipcv_Image::max(): types do not match");
  }

  template <class Type>
  void ipcv_Image<Type>::min(ip_Image<Type>* image)
  {
    ipcv_Image<Type>* cvi = dynamic_cast<ipcv_Image<Type>*>(image);
    if (cvi)
      cvMin(m_pImage, cvi->m_pImage, m_pImage);
    else
      log_ERROR("ipcv_Image::min(): types do not match");
  }

  /*
  template <class Type>
  int ipcv_Image<Type>::save(const char* filename)
  {
    cvSaveImage(filename, this->m_pImage);
    return 0;
  }
  */

  template <>
  inline int ipcv_Image<ip_ColorElement8u>::save(const char* filename)
  {
    cvSaveImage(filename, this->m_pImage);
    return 0;
  }

  template <>
  inline int ipcv_Image<unsigned char>::save(const char* filename)
  {
    cvSaveImage(filename, this->m_pImage);
    return 0;
  }

  template<>
  inline int ipcv_Image<float>::save(const char* filename)
  {
    IplImage* tmp_image = cvCreateImage(cvGetSize(m_pImage), IPL_DEPTH_8U, 3);
    IplImage* tmp_image1 = cvCreateImage(cvGetSize(m_pImage), IPL_DEPTH_8U, 1);

    cvConvertScale(m_pImage, tmp_image1, 255);
    cvCvtColor(tmp_image1, tmp_image, CV_GRAY2RGB);
    cvSaveImage(filename, tmp_image);

    cvReleaseImage(&tmp_image);
    cvReleaseImage(&tmp_image1);
    return 0;
  }

  template<>
  inline int ipcv_Image<short int>::save(const char* filename)
  {
    /*
    IplImage* tmp_image = cvCreateImage(cvGetSize(m_pImage), IPL_DEPTH_8U, 3);
    IplImage* tmp_image1 = cvCreateImage(cvGetSize(m_pImage), IPL_DEPTH_8U, 1);

    cvConvertScale(m_pImage, tmp_image1, 255);
    cvCvtColor(tmp_image1, tmp_image, CV_GRAY2RGB);
    cvSaveImage(filename, tmp_image);

    cvReleaseImage(&tmp_image);
    cvReleaseImage(&tmp_image1);
    */
    fprintf(stderr, "ipcv_Image<short int>::save() not implemented!\n");
    return 0;
  }



  //=======================================================
  //
  //
  //
  //=======================================================

  template <class Type>
    class ipcv_ImageAllocator :
    public  ip_ImageAllocator<Type> {

    public:
    virtual ip_Image<Type> * newIpImage(){
      return new ipcv_Image<Type> (0,0);
    }

  };


}

#endif
