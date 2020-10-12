/*
 * Copyright (c) 2010-2020 Idiap Research Institute
 *
 * See file COPYING for the licence associated with this software.
 */

#include "image_processing/ipcv_Image.h"

namespace ImageProcessing {

  //-----

  /**

      @author Jean-marc Odobez (Jean-Marc.Odobez@idiap.ch)
      @author Daniel Gatica-Perez (gatica@idiap.ch)
  */
  //===================================================
  //
  //    class         ipcv_image
  //
  //    class for image representation using opencv
  //       image representation
  //
  //    Correspond to one band unsigned char, or float
  //
  //===================================================

  void  TypeIpl(uchar a,int & depth,int & nbbands){ nbbands=1; depth= IPL_DEPTH_8U;  }

  void  TypeIpl(char a,int & depth,int & nbbands){  nbbands=1; depth= IPL_DEPTH_8S;  }

  void  TypeIpl(short int a,int & depth,int & nbbands){ nbbands=1; depth= IPL_DEPTH_16S;  }

  void  TypeIpl(int a,int & depth,int & nbbands){ nbbands=1; depth= IPL_DEPTH_32S;  }

  void  TypeIpl(float a,int & depth,int & nbbands){ nbbands=1; depth= IPL_DEPTH_32F;  }



}
