/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu 
  35042 Rennes Cedex

  http://www.irisa.fr

*/

#ifndef Motion2D_h
#define Motion2D_h

/*!

  \file Motion2D.h

  \brief File included when using CMotion2DModel, CMotion2DPyramid,
  CMotion2DEstimator or CMotion2DWarping.

*/

/*!

  Maximal number of parameters for a given 2D polynomial motion model. This
  number is equal CMotion2DModel::MDL_NMAX_COEF + 1, to take into account
  global illumination changes.

*/
#ifndef MAXCOEFSMODEL
#  define MAXCOEFSMODEL 13
#endif


/*! \enum EIdModel The implemented 2D polynomial motion models.

  Using matrix notation, these models can always be stated in the general way
  by:

  \f[
  \vec{w}_{A}(p_{i}) = \left[
  \begin{array}{l} u(p_{i}) \\ v(p_{i}) \end{array} \right] = B(p_i) A
  \f]

  which is linear with respect to the \e n motion parameters \f$ A^t =
  (c_1, c_2, a_1, ..., a_4, q_1, ..., q_6) \f$, and where \f$ p_i = (x_i, y_i)
  \f$ denotes the spatial position of a point in \f$(C, \vec{x}, \vec{y})\f$,
  \f$ \vec{w}_{A}(p_{i}) \f$ the flow
  vector modeled at point \f$ p_i\f$; \f$ B \f$ is a matrix, the form of wich
  depends on the chosen model, but the coefficients of which depend only on the
  point coordinates.

*/
enum EIdModel {
  NO_MDL, /*!< No model is defined. */

  MDL_TX, /*!< 2D translation motion model along the \f$x\f$ axis with 1
  parameter \f$c_1\f$: \f[ \vec{w}_{A}(p_{i}) = \left[ \begin{array}{l}
  u(p_{i}) \end{array} \right] = \left( \begin{array}{l} c_{1} \end{array}
  \right) \f] */

  MDL_TY, /*!< 2D translation motion model along the \f$y\f$ axis with 1
  parameter \f$c_2\f$: \f[ \vec{w}_{A}(p_{i}) = \left[ \begin{array}{l}
  u(p_{i}) \end{array} \right] = \left( \begin{array}{l} c_{2} \end{array}
  \right) \f] */

  MDL_TR, /*!< 2D translation motion model with 2 parameters \f$(c_1,c_2)\f$ :
  \f[ \vec{w}_{A}(p_{i}) = \left[ \begin{array}{l} u(p_{i}) \\ v(p_{i})
  \end{array} \right] = \left( \begin{array}{l} c_{1}\\ c_{2} \end{array}
  \right) \f] */

  MDL_AFF_TX_DIV, /*!< 2D affine motion model with 2 parameters \f$(c_1,a_1)\f$
  taking into account the translation along the \f$x\f$ axis and the 2D
  divergence: \f[ \vec{w}_{A}(p_{i}) = \left[ \begin{array}{l} u(p_{i}) \\
  v(p_{i}) \end{array} \right] = \left( \begin{array}{l} c_{1}\\ \; \end{array}
  \right) + \left( \begin{array}{rl} a_{1} & 0\\ 0 & a_{1} \end{array} \right)
  \cdot \left( \begin{array}{l} x_{i} \\ y_{i} \end{array} \right) \f] */

  MDL_AFF_TR_DIV, /*!< 2D affine motion model with 3 parameters
  \f$(c_1,c_2,a_1)\f$ taking into account the 2D translation and the 2D
  divergence: \f[ \vec{w}_{A}(p_{i}) = \left[ \begin{array}{l} u(p_{i}) \\
  v(p_{i}) \end{array} \right] = \left( \begin{array}{l} c_{1}\\ c_{2}
  \end{array} \right) + \left( \begin{array}{rl} a_{1} & 0\\ 0 & a_{1}
  \end{array} \right) \cdot \left( \begin{array}{l} x_{i}\\ y_{i} \end{array}
  \right) \f] */
  
  MDL_AFF_TR_ROT,  /*!< 2D affine motion model with 3 parameters
  \f$(c_1,c_2,a_1)\f$ taking into account the 2D translation and the 2D
  rotation: \f[ \vec{w}_{A}(p_{i}) = \left[ \begin{array}{l} u(p_{i}) \\
  v(p_{i}) \end{array} \right] = \left( \begin{array}{l} c_{1}\\ c_{2}
  \end{array} \right) + \left( \begin{array}{rl} 0 & a_{1} \\ -a_{1} & 0
  \end{array} \right) \cdot \left( \begin{array}{l} x_{i} \\ y_{i} \end{array}
  \right) \f] */
  
  MDL_AFF_TX_NULL, /*!< 2D affine motion model with 5 parameters
  \f$(c_2,a_1,a_2,a_3,a_4)\f$ with: \f[ \vec{w}_{A}(p_{i}) = \left[
  \begin{array}{l} u(p_{i}) \\ v(p_{i}) \end{array} \right] = \left(
  \begin{array}{l} \; \\ c_2 \end{array} \right) + \left( \begin{array}{lr}
  a_{1} & a_{2}\\ a_{3} & a_{4} \end{array} \right) \cdot \left(
  \begin{array}{l} x_{i}\\ y_{i} \end{array} \right) \f]

  If we express the parameter vector \f$A^t\f$ in another basis of elementary
  motion sub-fields \f$\phi^t=(c_1,c_2,div,rot,hyp_1,hyp_2)\f$ with:
  \f$div=\frac{1}{2}(a_1+a_4)\f$, \f$rot=\frac{1}{2}(a_3-a_2)\f$,
  \f$hyp1=\frac{1}{2}(a_1-a_4)\f$ and \f$hyp2=\frac{1}{2}(a_2+a_3)\f$, this
  motion model has the particularity to consider the translation along the
  \f$x\f$ axis as null (ie. \f$c_1=0\f$)*/

  MDL_AFF_TY_NULL, /*!< 2D affine motion model with 5 parameters
  \f$(c_1,a_1,a_2,a_3,a_4)\f$ with: \f[ \vec{w}_{A}(p_{i}) = \left[
  \begin{array}{l} u(p_{i}) \\ v(p_{i}) \end{array} \right] = \left(
  \begin{array}{l} c_1\\ \; \end{array} \right) + \left( \begin{array}{lr}
  a_{1} & a_{2}\\ a_{3} & a_{4} \end{array} \right) \cdot \left(
  \begin{array}{l} x_{i}\\ y_{i} \end{array} \right) \f]

  If we express the parameter vector \f$A^t\f$ in another basis of elementary
  motion sub-fields \f$\phi^t=(c_1,c_2,div,rot,hyp_1,hyp_2)\f$ with:
  \f$div=\frac{1}{2}(a_1+a_4)\f$, \f$rot=\frac{1}{2}(a_3-a_2)\f$,
  \f$hyp1=\frac{1}{2}(a_1-a_4)\f$ and \f$hyp2=\frac{1}{2}(a_2+a_3)\f$, this
  motion model has the particularity to consider the translation along the
  \f$y\f$ axis as null (ie. \f$c_2=0\f$)*/

  MDL_AFF_DIV_NULL, /*!< 2D affine motion model with 5 parameters
  \f$(c_1,c_2,a_1,a_2,a_3)\f$ with: \f[ \vec{w}_{A}(p_{i}) = \left[
  \begin{array}{l} u(p_{i}) \\ v(p_{i}) \end{array} \right] = \left(
  \begin{array}{l} c_{1}\\ c_{2} \end{array} \right) + \left( \begin{array}{lr}
  a_{1} & a_{2}\\ a_{3} & -a_{1} \end{array} \right) \cdot \left(
  \begin{array}{l} x_{i}\\ y_{i} \end{array} \right) \f]

  If we express the parameter vector \f$A^t\f$ in another basis of elementary
  motion sub-fields \f$\phi^t=(c_1,c_2,div,rot,hyp_1,hyp_2)\f$ with:
  \f$div=\frac{1}{2}(a_1+a_4)\f$, \f$rot=\frac{1}{2}(a_3-a_2)\f$,
  \f$hyp1=\frac{1}{2}(a_1-a_4)\f$ and \f$hyp2=\frac{1}{2}(a_2+a_3)\f$, this
  motion model has the particularity to consider the divergence term at
  zero (ie. \f$div=0\f$)*/
  
  MDL_AFF_ROT_NULL, /*!< 2D affine motion model with 5 parameters
  \f$(c_1,c_2,a_1,a_2,a_4)\f$ with: \f[ \vec{w}_{A}(p_{i}) = \left[
  \begin{array}{l} u(p_{i}) \\ v(p_{i}) \end{array} \right] = \left(
  \begin{array}{l} c_{1}\\ c_{2} \end{array} \right) + \left( \begin{array}{r}
  a_{1} \;\; a_{2}\\ a_{2} \;\; a_{4} \end{array} \right) \cdot \left(
  \begin{array}{l} x_{i}\\ y_{i} \end{array} \right) \f]

  If we express the parameter vector \f$A^t\f$ in another basis of elementary
  motion sub-fields \f$\phi^t=(c_1,c_2,div,rot,hyp_1,hyp_2)\f$ with:
  \f$div=\frac{1}{2}(a_1+a_4)\f$, \f$rot=\frac{1}{2}(a_3-a_2)\f$,
  \f$hyp1=\frac{1}{2}(a_1-a_4)\f$ and \f$hyp2=\frac{1}{2}(a_2+a_3)\f$, this
  motion model has the particularity to consider the curl term at
  zero (ie. \f$rot=0\f$)*/
  
  MDL_AFF_HYP1_NULL, /*!< 2D affine motion model with 5 parameters
  \f$(c_1,c_2,a_1,a_2, a_3)\f$ with: \f[ \vec{w}_{A}(p_{i}) = \left[
  \begin{array}{l} u(p_{i}) \\ v(p_{i}) \end{array} \right] = \left(
  \begin{array}{l} c_{1}\\ c_{2} \end{array} \right) + \left( \begin{array}{r}
  a_{1} \;\; a_{2}\\ a_{3} \;\; a_{1} \end{array} \right) \cdot \left(
  \begin{array}{l} x_{i}\\ y_{i} \end{array} \right) \f]

  If we express the parameter vector \f$A^t\f$ in another basis of elementary
  motion sub-fields \f$\phi^t=(c_1,c_2,div,rot,hyp_1,hyp_2)\f$ with:
  \f$div=\frac{1}{2}(a_1+a_4)\f$, \f$rot=\frac{1}{2}(a_3-a_2)\f$,
  \f$hyp1=\frac{1}{2}(a_1-a_4)\f$ and \f$hyp2=\frac{1}{2}(a_2+a_3)\f$, this
  motion model has the particularity to consider the first hyperbolic term at
  zero (ie. \f$hyp_1=0\f$).*/
  
  MDL_AFF_HYP2_NULL, /*!< 2D affine motion model with 5 parameters
  \f$(c_1,c_2,a_1, a_2, a_4)\f$ with: \f[ \vec{w}_{A}(p_{i}) = \left[
  \begin{array}{l} u(p_{i}) \\ v(p_{i}) \end{array} \right] = \left(
  \begin{array}{l} c_{1}\\ c_{2} \end{array} \right) + \left( \begin{array}{r}
  a_{1} \;\; a_{2}\\ -a_{2} \;\; a_{4} \end{array} \right) \cdot \left(
  \begin{array}{l} x_{i}\\ y_{i} \end{array} \right) \f]

  If we express the parameter vector \f$A^t\f$ in another basis of elementary
  motion sub-fields \f$\phi^t=(c_1,c_2,div,rot,hyp_1,hyp_2)\f$ with:
  \f$div=\frac{1}{2}(a_1+a_4)\f$, \f$rot=\frac{1}{2}(a_3-a_2)\f$,
  \f$hyp1=\frac{1}{2}(a_1-a_4)\f$ and \f$hyp2=\frac{1}{2}(a_2+a_3)\f$, this
  motion model has the particularity to consider the second hyperbolic term at
  zero (ie. \f$hyp_2=0\f$).*/

  MDL_AFF_TR_ROT_DIV, /*!< 2D affine motion model with 4 parameters
  \f$(c_1,c_2,a_1,a_2)\f$ able to consider motion like 2D translation with 2D
  rotation and divergence: \f[ \vec{w}_{A}(p_{i}) = \left[ \begin{array}{l}
  u(p_{i}) \\ v(p_{i}) \end{array} \right] = \left( \begin{array}{l} c_{1}\\
  c_{2} \end{array} \right) + \left( \begin{array}{rl} a_{1} & a_{2}\\ - a_{2}
  & a_{1} \end{array} \right) \cdot \left( \begin{array}{l} x_{i}\\ y_{i}
  \end{array} \right) \f]

  If we express the parameter vector \f$A^t\f$ in another basis of elementary
  motion sub-fields \f$\phi^t=(c_1,c_2,div,rot,hyp_1,hyp_2)\f$ with:
  \f$div=\frac{1}{2}(a_1+a_4)\f$, \f$rot=\frac{1}{2}(a_3-a_2)\f$,
  \f$hyp1=\frac{1}{2}(a_1-a_4)\f$ and \f$hyp2=\frac{1}{2}(a_2+a_3)\f$, this
  motion model has the particularity to consider the hyperbolic terms at zero
  (ie. \f$hyp_1=hyp_2=0\f$).*/
  
  MDL_AFF_COMPLET, /*!< 2D affine motion model with 6 parameters
  \f$(c_1,c_2,a_1,...,a_2)\f$ taking into account constant and affine
  parameters: \f[ \vec{w}_{A}(p_{i}) = \left[ \begin{array}{l} u(p_{i}) \\
  v(p_{i}) \end{array} \right] = \left( \begin{array}{l} c_{1}\\ c_{2}
  \end{array} \right) + \left( \begin{array}{l} a_{1} \;\; a_{2}\\ a_{3} \;\;
  a_{4} \end{array} \right) \cdot \left( \begin{array}{l} x_{i}\\ y_{i}
  \end{array} \right) \f]

  This model is a good tradeoff between complexity and representativeness. It
  can take into account many kind of motion (translation, rotation, scaling,
  deformation), and even if a rigid 3D motion gives rise to a quadratic model
  in the image plane, the affine flow recovers the essential part.  */
  
  MDL_QUA_PAN_DIV, /*!< 2D quadratic motion model with 4 parameters
  \f$(c_1,a_1,q_1,q_2)\f$ taking into account the translation along the \f$x\f$
  axis, some affine and quadratic parameters: \f[ \vec{w}_{A}(p_{i}) = \left[
  \begin{array}{l} u(p_{i}) \\ v(p_{i}) \end{array} \right] = \left(
  \begin{array}{l} c_{1}\\ \; \end{array} \right) + \left( \begin{array}{l}
  a_{1} \;\; 0\\ 0 \;\; a_{1} \end{array} \right) \cdot \left( \begin{array}{l}
  x_{i} \\ y_{i} \end{array} \right) + \left( \begin{array}{lll} q_{1} & q_{2}
  & 0\\ 0 & q_{1} & q_{2} \end{array} \right) \cdot \left( \begin{array}{l}
  {x_{i}}^2\\ x_{i} y_{i} \\ {y_{i} }^2 \end{array} \right) \f] */
  
  MDL_QUA_PAN_TILT,  /*!< 2D quadratic motion model with 4 parameters \f$(c_1,
  c_2, q_1,q_2)\f$ taking into account the 2D translation and some quadratic
  parameters: \f[ \vec{w}_{A}(p_{i}) = \left[ \begin{array}{l} u(p_{i}) \\
  v(p_{i}) \end{array} \right] = \left( \begin{array}{l} c_{1}\\ c_{2}
  \end{array} \right) + \left( \begin{array}{lll} q_{1} & q_{2} & 0\\ 0 & q_{1}
  & q_{2} \end{array} \right) \cdot \left( \begin{array}{l} {x_{i}}^2\\ x_{i}
  y_{i} \\ {y_{i}}^2 \end{array} \right) \f] */
  
  MDL_QUA_PAN_TILT_DIV, /*!< Quadratic 2D motion model with 5 parameters
  \f$(c_1,c_2,a_1,q_1,q_2)\f$ taking into account the 2D translation, some
  affine and quadratic parameters: \f[ \vec{w}_{A}(p_{i}) = \left[
  \begin{array}{l} u(p_{i}) \\ v(p_{i}) \end{array} \right] = \left(
  \begin{array}{l} c_{1}\\ c_{2} \end{array} \right) + \left( \begin{array}{l}
  a_{1} \;\; 0\\ 0 \;\; a_{1} \end{array} \right) \cdot \left( \begin{array}{l}
  x_{i}\\ y_{i} \end{array} \right) + \left( \begin{array}{lll} q_{1} & q_{2} &
  0\\ 0 & q_{1} & q_{2} \end{array} \right) \cdot \left( \begin{array}{l}
  {x_{i}}^2\\ x_{i} y_{i} \\ {y_{i}}^2 \end{array} \right) \f] */
  
  MDL_QUA_2D, /*!< 2D quadratic motion model with 8 parameters
  \f$(c_1,c_2,a_1,...,a_4,q_1,q_2)\f$ taking into account the constants, the
  linear and some quadratic parameters: \f[ \vec{w}_{A}(p_{i}) = \left[
  \begin{array}{l} u(p_{i}) \\ v(p_{i}) \end{array} \right] = \left(
  \begin{array}{l} c_{1}\\ c_{2} \end{array} \right) + \left( \begin{array}{l}
  a_{1} \;\; a_{2}\\ a_{3} \;\; a_{4} \end{array} \right) \cdot \left(
  \begin{array}{l} x_{i}\\ y_{i} \end{array} \right) + \left(
  \begin{array}{lll} q_{1} & q_{2} & 0\\ 0 & q_{1} & q_{2} \end{array} \right)
  \cdot \left( \begin{array}{l} {x_{i}}^2\\ x_{i} y_{i} \\ {y_{i}}^2
  \end{array} \right) \f] */
  
  MDL_QUA_COMPLET /*!< 2D quadratic motion model with 12 parameters
  \f$(c_1,c_2,a_1,...,a_4,q_1,...,q_6)\f$ taking into account the constants,
  the linear and all the quadratic parameters: \f[ \vec{w}_{A}(p_{i}) = \left[
  \begin{array}{l} u(p_{i}) \\ v(p_{i}) \end{array} \right] = \left(
  \begin{array}{l} c_{1}\\ c_{2} \end{array} \right) + \left( \begin{array}{l}
  a_{1} \;\; a_{2}\\ a_{3} \;\; a_{4} \end{array} \right) \cdot \left(
  \begin{array}{l} x_{i}\\ y_{i} \end{array} \right) + \left( \begin{array}{l}
  q_{1} \;\; q_{2} \;\; q_{3}\\ q_{4} \;\; q_{5} \;\; q_{6} \end{array} \right)
  \cdot \left( \begin{array}{l} {x_{i}}^2\\ x_{i} y_{i} \\ {y_{i}}^2
  \end{array} \right) \f] */
};


/*

  Internal image bitmap structure used for pixels coded in short.
  
 */

typedef struct {
  short  *ad;	         /* Address of the bitmap.		*/
  int	  nbli;		 /* Number of rows for the bitmap.	*/
  int	  nbco;		 /* Number of columns for the bitmap.	*/
} TImageShort;

/*

  Internal image bitmap structure used for pixels coded in float.
  
 */
typedef struct {
  float	  *ad;		/* Address of the bitmap.		*/ 
  int	  nbli;		/* Number of rows for the bitmap.	*/
  int	  nbco;		/* Number of columns for the bitmap.	*/
} TImageFloat;


#endif	/* Motion2D_h */
