/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu
  35042 Rennes Cedex

  http://www.irisa.fr

*/

/*!
  \file CMotion2DWarping.cpp
  \brief Definition of CMotion2DWarping.
*/

/*!

  \class CMotion2DWarping

  \brief The CMotion2DWarping class provides motion-compensation.


*/

#ifdef __SunOS_
#  include <iostream.h>
#  include <string.h>
#else
#  include <iostream>
#  include <string>
#endif
#include <stdlib.h>

#include "type.h"
#include "constant.h"
#include "macro.h"


#include "../estimation/para_mvt.h"
#include "../compense/compense.h"


#include <cmotion2d/CMotion2DWarping.h>

/*!

  Constructor.

*/
CMotion2DWarping::CMotion2DWarping()
{
  imFx.ad = NULL;
  imFy.ad = NULL;
}

/*!

  Destructor. Frees the allocated memory.

  \sa initBackWarp()

*/
CMotion2DWarping::~CMotion2DWarping()
{
  if ((imFx.ad != NULL) || (imFy.ad != NULL)){
    free_recalage_film(&imFx, &imFy);
    imFx.ad = imFy.ad = NULL;
  }
}
/*!

  This function allocates the memory necessary to store the combination of the
  instantaneous motion displacements between two successive images and defines
  the reference image which respect to the motion compensation is performed.

  The size of the images to compensate is given by [\e src_nrows, \e
  src_ncols].

  The size of the motion compensated image computed by warp() or backWarp() is
  given by [\e dst_nrows, \e dst_ncols]. His size can be greater than the image
  size.

  The parameters \e dst_row_offset and \e dst_col_offset, specify the position
  of the reference image in the motion compensated image. The considered origin
  is the upper-left corner of the source image.

  return true if the warping initialization succeed, false otherwise.

  \sa warp(), backWarp()

*/
bool CMotion2DWarping::initBackWarp(int dst_nrows, int dst_ncols,
				    int dst_row_offset, int dst_col_offset)
{
  bool ret;

  // Quelques initialisations
  Nbli_ima_out  = 0;	// Nombre de lignes de l'image recalee.
  Nbcol_ima_out = 0;	// Nombre de colonnes de l'image recalee.
  Offset_li  = 0;		// Decalage en lignes par rapport a l'origine.
  Offset_co  = 0;		// Decalage en colonn par rapport a l'origine.
  Ivariation = 0.0;

  if ((imFx.ad != NULL) || (imFy.ad != NULL)){
    free_recalage_film(&imFx, &imFy);
    imFx.ad = imFy.ad = NULL;
  }

  // Init cumulated motion
  ret = set_ctes_warp_seq(dst_nrows, dst_ncols,
			  dst_row_offset, dst_col_offset, 1,
			  &Nbli_ima_out, &Nbcol_ima_out,
			  &Offset_li, &Offset_co, &Ivariation,
			  &imFx, &imFy);

  return ret;
}


/*!

  Generate a backwarped image in \e dst from the original image \e src, using
  the motion model \e model.

  The size of the images to backwarped is given by [\e src_nrows, \e
  src_ncols ].

  The size of the backwarped image is to set by calling initBackWarp().

  The 2D parametric motion model to use to backwarp the image \e src is
  specified by \e model.

  \warning Before using this function, the warping must be initialized by a
  call to initBackWarp().

  \warning The table containing the compensated image \e dst must be allocated
  before a call to this function.

  Return true if success, false otherwise.

  \sa initBackWarp()

*/
bool CMotion2DWarping::backWarp(unsigned char *src,
				int src_nrows, int src_ncols,
				unsigned char *dst,
				CMotion2DModel model)
{
  Para	paramet;
  int	i;

  paramet.nb_para = model.getNParameters();

  if (!paramet.nb_para)
  {
    printf("Motion model not implemented.\n");
    return false;
  }

  paramet.var_light = model.getVarLight();
  paramet.id_model  = model.getIdModel();

  model.getOrigin(paramet.li_c, paramet.co_c);

  for (i = 0; i < MAXCOEFSMODEL; i++)
    paramet.thet[i] = 0.0;

  model.getParameters(paramet.thet);

  /* Recalage en utilisant le mvt estime entre les 2 dernieres images.	*/
  if (Actualise_deplacements_para(&paramet,
				  &Nbli_ima_out, &Nbcol_ima_out,
				  &Ivariation,
				  &imFx, &imFy) == false)
    return false;

  Recale_image(src, dst, src_nrows, src_ncols,
	       &Nbli_ima_out, &Nbcol_ima_out, &Ivariation,
	       &imFx, &imFy);

  return true;
}

/*!

  Generate a backwarped image in \e dst from the original image \e src, using
  the motion model \e model.

  The size of the images to backwarped is given by [\e src_nrows, \e
  src_ncols ].

  The size of the backwarped image is to set by calling initBackWarp().

  The 2D parametric motion model to use to backwarp the image \e src is
  specified by \e model.

  \warning Before using this function, the warping must be initialized by a
  call to initBackWarp().

  \warning The table containing the compensated image \e dst must be allocated
  before a call to this function.

  Return true if success, false otherwise.

  \sa initBackWarp()

*/
bool CMotion2DWarping::backWarp(short *src,
				int src_nrows, int src_ncols,
				short *dst,
				CMotion2DModel model)
{
  Para	paramet;
  int	i;

  paramet.nb_para = model.getNParameters();

  if (!paramet.nb_para)
  {
    printf("Motion model not implemented.\n");
    return false;
  }

  paramet.var_light = model.getVarLight();
  paramet.id_model  = model.getIdModel();

  model.getOrigin(paramet.li_c, paramet.co_c);

  for (i = 0; i < MAXCOEFSMODEL; i++)
    paramet.thet[i] = 0.0;

  model.getParameters(paramet.thet);

  /* Recalage en utilisant le mvt estime entre les 2 dernieres images.	*/
  if (Actualise_deplacements_para(&paramet,
				  &Nbli_ima_out, &Nbcol_ima_out,
				  &Ivariation,
				  &imFx, &imFy) == false)
    return false;

  Recale_image(src, dst, src_nrows, src_ncols,
	       &Nbli_ima_out, &Nbcol_ima_out, &Ivariation,
	       &imFx, &imFy);

  return true;
}

/*!

  Warp the image \e src into \e dst using the motion model \e model. The
  parameter \e nrows refers to the number of lines in the images. \e ncols
  indicates the number of columns.

  \warning The table containing the warped image \e dst must be
  allocated before a call to this function.

  Return true if success, false otherwise.

  \sa CMotion2DModel

*/
bool CMotion2DWarping::warp(unsigned char *src, unsigned char *dst,
			    int nrows, int ncols, CMotion2DModel model)
{

  Para para;
  bool ret;
  unsigned char val = 0;

  model.getParameters(para.thet);

  para.nb_para   = model.getNParameters();
  para.id_model  = model.getIdModel();
  para.var_light = model.getVarLight();

  model.getOrigin(para.li_c, para.co_c);

  ret = Warp_image(src, dst, nrows, ncols, &para, val);

  return ret;

}



/*!

  Gives the cumulated displacements of a pixel which position is defined by \e
  row, \e col, with respect to the reference image specified by initBackWarp().

  The cumulated displacement along the row axis is stored in \e d_row, along
  the column axis in \e d_cols.

  \sa initBackWarp(), backWarp()

*/
void CMotion2DWarping::getBackWarpPosition(int row, int col,
					   float &d_row, float &d_col)
{
  get_deplacement (col, row, &d_col, &d_row, &Nbcol_ima_out, &imFx, &imFy);
}

