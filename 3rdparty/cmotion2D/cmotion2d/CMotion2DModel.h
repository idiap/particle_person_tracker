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
  \file CMotion2DModel.h
  \brief File to include in order to use the CMotion2DModel class.
*/

#ifndef CMotion2DModel_h
#define CMotion2DModel_h

#include <stdio.h>
#include <string>
#include "Motion2D.h"

using namespace std;

#if defined (WIN32)
#  if defined MOTION2D_DLL_EXPORTS
#     define MOTION2D_API __declspec( dllexport )
#  elif defined MOTION2D_DLL_IMPORTS
#     define MOTION2D_API __declspec( dllimport )
#  else
#     define MOTION2D_API
#  endif
#else
#     define MOTION2D_API
#endif


class MOTION2D_API CMotion2DModel
{
 public:
  /*!
    Maximal number of parameters for a 2D polynomial motion model.
  */
 enum {
   MDL_NMAX_COEF = 12  /*!< Maximal number of parameters */
 };

 public:
  CMotion2DModel();
  CMotion2DModel(const CMotion2DModel & model);
  ~CMotion2DModel();

  // Specify the caracteristics of the model
  void setOrigin(double row, double col);
  void setIdModel(EIdModel id);
  void setIdModel(string id);
  void setVarLight(bool state, double parameter=0.0);

  void setParameters(const double parameters[MDL_NMAX_COEF]);

  bool compose(CMotion2DModel model);
  int  getNParameters();
  int  getDegre();
  void getParameters(double parameters[MDL_NMAX_COEF]);
  void reset();
  bool invert(CMotion2DModel &inv);
  string idToString();
  bool changeRepere(double row, double col);
  bool getDisplacement(double row, double col, double &d_row, double &d_col);

  /*! Return the id of the 2D polynomial motion model. The complete list of the
  2D polynomial motion models available is given by the EIdModel() enum */
  EIdModel getIdModel() { return id_model; };
  bool getVarLight();
  bool getVarLight(double & parameter);
  CMotion2DModel getModelLevel(int delta_level);
  /*!

    Returns the origin coordinates: \f$ O = (x_c, y_c) = (col, row)
    \f$.

    \sa setOrigin()

  */
  void getOrigin(double &row, double &col) {row = li_c; col = co_c; };

  /*!

    Returns the parameter \e n of the 2D polynomial motion model. The number of
    parameters associated to a given 2D motion model is accessible with
    getNParameters().

    \warning If the global illumination variation is estimated, the
    corresponding value is given by getVarLight().

    The general form of the model is given by:
    \f[
    \vec{w}_{A}(p_{i}) = \left[
    \begin{array}{l} u(p_{i}) \\ v(p_{i}) \end{array} \right] = B(p_i) A
    \f]

    \f[ \vec{w}_{A}(p_{i}) = \left( \begin{array}{l} c_{1}\\ c_{2} \end{array}
    \right) + \left( \begin{array}{l} a_{1} \;\; a_{2}\\ a_{3} \;\; a_{4}
    \end{array} \right) \cdot \left( \begin{array}{l} x_{i}\\ y_{i} \end{array}
    \right) + \left( \begin{array}{l} q_{1} \;\; q_{2} \;\; q_{3}\\ q_{4} \;\;
    q_{5} \;\; q_{6} \end{array} \right) \cdot \left( \begin{array}{l}
    {x_{i}}^2\\ x_{i} y_{i} \\ {y_{i}}^2 \end{array} \right) \f]

    The table below gives the correspondence between the value of \e n and the
    corresponding parameter of the model.

    \f[
    \begin{tabular}{|l||c|c|c|c|c|c|c|c|c|c|c|c|}
    \hline
    n         & 0     & 1     & 2 & 3 & 4 & 5 & 6 & 7 & 8 & 9& 10& 11\\
    \hline
    A(n) & $c_1$ & $c_2$ & $a_1$ &$a_2$ &$a_3$ &$a_4$ &$q_1$ &$q_2$ &$q_3$ &$q_4$ & $q_5$ &$q_6$ \\
    \hline
    \end{tabular}
    \f]

    \sa getVarLight(), getNParameters(), getParameters()

  */
  double operator[](int n) {return theta[n];}
  void operator=(const CMotion2DModel &model);

 private:
  double theta[MAXCOEFSMODEL];	// Param�tres du modele de mouvement.
  double li_c, co_c;	// Point de r�f�rence du mod�le (peut �tre != du cdg).
  bool var_light;        // booleen. 1 si on veut un modele gerant la variation d'eclairage
  EIdModel id_model;    // identification du model :
};

#endif
