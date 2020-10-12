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
  \file CMotion2DModel.cpp
  \brief Definition of the CMotion2DModel class.
*/

/*!

  \class CMotion2DModel

  \brief The CMotion2DModel class provides 2D parametric motion models.

  Using matrix notation, these models can always be written as:

  \f[
  \vec{w}_{A}(p_{i}) = \left[
  \begin{array}{l} u(p_{i}) \\ v(p_{i}) \end{array} \right] = B(p_i) A
  \f]

  which is linear with respect to the \e n motion parameters \f$ A^t = (c_1,
  c_2, a_1, ..., a_4, q_1, ..., q_6) \f$, and where \f$ p_i = (x_i, y_i) \f$
  denotes the spatial position of a point in \f$(O, \vec{x}, \vec{y})\f$, \f$
  \vec{w}_{A}(p_{i}) \f$ the flow vector given by the estimated motion model at
  point \f$ p_i\f$ ; \f$ B \f$ is a matrix, the form of which depends on the
  chosen model, but the coefficients of which depend only on the point
  coordinates.

  The complete list of the 2D polynomial motion models available is given by
  the EIdModel() enum. It involves constant, affine, quadratic (complete or
  not), motion models.

  Moreover, to take into account global illumination changes between two
  successive images, we have introduced the parameter \f$ \zeta \f$.

  Finally, the resulting implemented motion models are specified by parameters
  \f$\Theta^t = (A^t, \zeta)\f$.

  To initialize a motion model, use setOrigin() to define the reference point
  \f$ O \f$ of the model, setIdModel() to specify the 2D polynomial model
  \f$\vec{w}_{A}(p_{i})\f$ to use, eventually setParameters() to initialize the
  parameters \f$ A^t \f$ of the model and setVarLight() to initialize
  \f$\zeta\f$.

  \sa setOrigin(), setIdModel(), setParameters(), setVarLight()

*/

#ifdef __SunOS_
# include <iostream.h>
#else
# include <iostream>
#endif
#include <stdlib.h>
#include <string>

#include "type.h"
#include "constant.h"
#include "macro.h"

#include "../estimation/para_mvt.h"
#include "../compense/compense.h"

#include <cmotion2d/CMotion2DModel.h>

/*!

  Default constructor for 2D polynomial motion models.

*/
CMotion2DModel::CMotion2DModel()
{
  for(int i = 0; i < MAXCOEFSMODEL; i++)
    theta[i] = 0.0;	// Valeur Initiale des param�tres � estimer.

  li_c = co_c = -1;

  // Default parameter
  var_light = false; // Global illumination variation is not estimated
  id_model  = MDL_AFF_COMPLET; // Affine motion model
}

/*!

  Copy constructor for 2D polynomial motion models.

*/
CMotion2DModel::CMotion2DModel(const CMotion2DModel & model)
{
  for(int i = 0; i < MAXCOEFSMODEL; i++)
    theta[i] = model.theta[i];	// Valeur Initiale des param�tres � estimer.

  li_c = model.li_c;
  co_c = model.co_c;

  var_light = model.var_light;
  id_model  = model.id_model;
}

/*!

  Destructor.

*/
CMotion2DModel::~CMotion2DModel()
{
}

/*!

  A model is defined for a given coordinated system origin \f$O=(x_c,y_c)\f$,
  by its id and eventually by an additional parameter corresponding to the
  global illumination variation. This method is used to specify the image
  coordinate system origin. If the origin is not specified (i.e., \e row = -1
  and \e col = -1) the origin corresponds to the center of the
  support, otherwise \f$p_c=(x_c,y_c)=(col, row)\f$.

  \sa getOrigin(), setIdModel(), setVarLight()

*/
void CMotion2DModel::setOrigin(double row, double col)
{
  li_c = row;
  co_c = col;
}

/*!

  A model is defined for a given coordinated system origin \f$O=(x_c,y_c)\f$,
  by its id and eventually by an additional parameter corresponding to the
  global illumination variation. This method is used to specify the model id.
  The complete list of the 2D polynomial motion models available is given by
  the EIdModel() enum.

  \sa setIdModel(string), getIdModel(), setOrigin(), setVarLight()

*/
void CMotion2DModel::setIdModel(EIdModel id)
{
  id_model = id;
}

/*!

  A model is defined for a given coordinated system origin \f$O=(x_c,y_c)\f$,
  by its id and eventually by an additional parameter corresponding to the
  global illumination variation. This method is used to specify the model id.
  The complete list of the 2D polynomial motion models available is given by
  the EIdModel() enum.

  The relation between the string \e id and the polynomial motion model id
  is given below:

  <center>
  <table border=0 width=100%>
  <tr><td align=center><strong>string id</strong> <td align=center> <strong>EIdModel model Id</strong>
  <tr><td>TX   or MDL_TX               <td>  EIdModel::MDL_TX
  <tr><td>TY   or MDL_TY               <td>  EIdModel::MDL_TY
  <tr><td>TR   or MDL_TR               <td>  EIdModel::MDL_TR
  <tr><td>AXD  or MDL_AFF_TX_DIV       <td>  EIdModel::MDL_AFF_TX_DIV
  <tr><td>ARD  or MDL_AFF_TR_DIV       <td>  EIdModel::MDL_AFF_TR_DIV
  <tr><td>ARR  or MDL_AFF_TR_ROT       <td>  EIdModel::MDL_AFF_TR_ROT
  <tr><td>AXN  or MDL_AFF_TX_NULL      <td>  EIdModel::MDL_AFF_TX_NULL
  <tr><td>AYN  or MDL_AFF_TY_NULL      <td>  EIdModel::MDL_AFF_TY_NULL
  <tr><td>ADN  or MDL_AFF_DIV_NULL     <td>  EIdModel::MDL_AFF_DIV_NULL
  <tr><td>ARN  or MDL_AFF_ROT_NULL     <td>  EIdModel::MDL_AFF_ROT_NULL
  <tr><td>AH1N or MDL_AFF_HYP1_NULL    <td>  EIdModel::MDL_AFF_HYP1_NULL
  <tr><td>AH2N or MDL_AFF_HYP2_NULL    <td>  EIdModel::MDL_AFF_HYP2_NULL
  <tr><td>ARRD or MDL_AFF_TR_ROT_DIV   <td>  EIdModel::MDL_AFF_TR_ROT_DIV
  <tr><td>AC   or MDL_AFF_COMPLET      <td>  EIdModel::MDL_AFF_COMPLET
  <tr><td>QPD  or MDL_QUA_PAN_DIV      <td>  EIdModel::MDL_QUA_PAN_DIV
  <tr><td>QPT  or MDL_QUA_PAN_TILT     <td>  EIdModel::MDL_QUA_PAN_TILT
  <tr><td>QPTD or MDL_QUA_PAN_TILT_DIV <td>  EIdModel::MDL_QUA_PAN_TILT_DIV
  <tr><td>Q2D  or MDL_QUA_2D           <td>  EIdModel::MDL_QUA_2D
  <tr><td>QC   or MDL_QUA_COMPLET      <td>  EIdModel::MDL_QUA_COMPLET
  </table>
  </center>

  \sa setIdModel(EIdModel), getIdModel(), setOrigin(), setVarLight()

*/
void CMotion2DModel::setIdModel(string id)
{
  if (id == "TX" || id == "MDL_TX")
    id_model = MDL_TX;
  else
  if (id == "TY" || id == "MDL_TY")
    id_model = MDL_TY;
  else
  if (id == "TR" || id == "MDL_TR")
    id_model = MDL_TR;
  else
  if (id == "AXD" || id == "MDL_AFF_TX_DIV")
    id_model = MDL_AFF_TX_DIV;
  else
  if (id == "ARD" || id == "MDL_AFF_TR_DIV")
    id_model = MDL_AFF_TR_DIV;
  else
  if (id == "ARR" || id == "MDL_AFF_TR_ROT")
    id_model = MDL_AFF_TR_ROT;
  else
  if (id == "AXN" || id == "MDL_AFF_TX_NULL")
    id_model = MDL_AFF_TX_NULL;
  else
  if (id == "AYN" || id == "MDL_AFF_TY_NULL")
    id_model = MDL_AFF_TY_NULL;
  else
  if (id == "ADN" || id == "MDL_AFF_DIV_NULL")
    id_model = MDL_AFF_DIV_NULL;
  else
  if (id == "ARN" || id == "MDL_AFF_ROT_NULL")
    id_model = MDL_AFF_ROT_NULL;
  else
  if (id == "AH1N" || id == "MDL_AFF_HYP1_NULL")
    id_model = MDL_AFF_HYP1_NULL;
  else
  if (id == "AH2N" || id == "MDL_AFF_HYP2_NULL")
    id_model = MDL_AFF_HYP2_NULL;
  else
  if (id == "ARRD" || id == "MDL_AFF_TR_ROT_DIV")
    id_model = MDL_AFF_TR_ROT_DIV;
  else
  if (id == "AC" || id == "MDL_AFF_COMPLET")
    id_model = MDL_AFF_COMPLET;
  else
  if (id == "QPD" || id == "MDL_QUA_PAN_DIV")
    id_model = MDL_QUA_PAN_DIV;
  else
  if (id == "QPT" || id == "MDL_QUA_PAN_TILT")
    id_model = MDL_QUA_PAN_TILT;
  else
  if (id == "QPTD" || id == "MDL_QUA_PAN_TILT_DIV")
    id_model = MDL_QUA_PAN_TILT_DIV;
  else
  if (id == "Q2D" || id == "MDL_QUA_2D")
    id_model = MDL_QUA_2D;
  else
  if (id == "QC" || id == "MDL_QUA_COMPLET")
    id_model = MDL_QUA_COMPLET;
  else
    id_model = NO_MDL;

}

/*!

  A model is defined for a given coordinated system origin \f$O=(x_c,y_c)\f$,
  by its id and eventually by an additional parameter corresponding to the
  global illumination variation. This method is used to specify if the model
  involves or not the global illumination parameter and to eventually set the
  global illumination variation parameter.

  \sa setOrigin(), setIdModel(), setParameters(), getVarLight(),
  getVarLight(double &)

*/
void CMotion2DModel::setVarLight(bool state, double parameter)
{
  var_light = state;
  theta[12] = parameter;
}

/*!

  Computes the motion composition between the current motion model and another
  motion model passed as parameter to the function.

  \f[
  \Theta = "\Theta \circ \Theta_2" \makebox[1.5cm]{with} \left\{
  \begin{array}{l} A = "A \circ A_2" \\ \zeta = \zeta + \zeta_2 \end{array} \right.
  \f]

  where \f$\Theta\f$ denotes the current motion model parameters, and
  \f$\Theta_2\f$ is the model parameter vector passed as parameter to the
  function.

  The motion composition is only performed if the degree of the 2D polynomial
  motion models to manipulate are 0 or 1.

  Returns true if success, false otherwise.

  \sa getDegre()

*/
bool CMotion2DModel::compose(CMotion2DModel model)
{
  Para	para1, para2, para_out;
  int	i;

  // etablit le nombre de parametre du modele

  para1.nb_para = Nb_Para_Modele(id_model);
  // test si le modele existe
  if (!para1.nb_para)
  {
    printf("Motion model not implemented.\n");
    return false;
  }

  para1.var_light = var_light;
  para1.id_model = id_model;

  para2.nb_para = Nb_Para_Modele(model.id_model);
  // test si le modele existe
  if (!para2.nb_para)
  {
    printf("Motion model not implemented.\n");
    return false;
  }

  para2.var_light = model.var_light;
  para2.id_model = model.id_model;

  // Verification si la composition est realisable.
  if ( model_degree(para1.id_model)==2 || model_degree(para2.id_model)==2 ) {
    printf("\nRMRmComposePara()::Error\n");
    printf("    No motion composition with quadratic motion models.\n\n");
    return false;
  }

  // Mise a jour des variables locales.
  for (i = 0; i < MAXCOEFSMODEL; i++) {
    para1.thet[i] = theta[i];
    para2.thet[i] = model.theta[i];
    para_out.thet[i] = 0.0;
  }

  // Mise a jour eventuelle de la variation d'eclairage.
  if (var_light) {
    para1.thet[12] = theta[para1.nb_para];
    para2.thet[12] = model.theta[para2.nb_para];
    para1.thet[para1.nb_para] = 0.0;
    para2.thet[para1.nb_para] = 0.0;
  }

  para1.li_c = li_c;
  para1.co_c = co_c;

  para2.li_c = model.li_c;
  para2.co_c = model.co_c;

  para_out.nb_para = para2.nb_para;
  para_out.id_model = para2.id_model;
  para_out.var_light = para2.var_light;
  para_out.li_c = model.li_c;
  para_out.co_c = model.co_c;


  // Composition des mouvements.
  if (compose_para(&para1, &para2, &para_out) == false)
    return false;

  // Mise a jour du resultat
  for (i = 0; i < MAXCOEFSMODEL; i++) {
    theta[i] = para_out.thet[i];
  }

  var_light = para_out.var_light;
  id_model = para_out.id_model;
  li_c = para_out.li_c;
  co_c = para_out.co_c;
  if (var_light) {
    theta[12] = 0.0;
    theta[para1.nb_para] = para_out.thet[12];
  }
  return true;
}

/*!

  Returns the number of parameters describing the 2D polynomial motion model.

  \warning If the motion model involves the global illumination parameter, the
  value returned does not take it into account.

  \sa EIdModel()

*/
int CMotion2DModel::getNParameters()
{
  return Nb_Para_Modele(id_model);
}

/*!

  Returns the degree of the 2D polynomial motion model, either :

  - 0 for translation models (like EIdModel::MDL_TX, EIdModel::MDL_TR),

  - 1 for affine models (like EIdModel::MDL_AFF_TX_DIV,
  EIdModel::MDL_AFF_TR_DIV, EIdModel::MDL_AFF_TR_ROT,
  EIdModel::MDL_AFF_TR_ROT_DIV, EIdModel::MDL_AFF_COMPLET),

  - 2 for quadratic models (like EIdModel::MDL_QUA_PAN_DIV,
  EIdModel::MDL_QUA_PAN_TILT, EIdModel::MDL_QUA_PAN_TILT_DIV,
  EIdModel::MDL_QUA_2D, EIdModel::MDL_QUA_COMPLET).

  Returns -1 if the model is not recognized.

  \sa EIdModel

*/
int CMotion2DModel::getDegre()
{
  int degre;

  switch (id_model) {
  case MDL_TX:
  case MDL_TR:
    degre = 0; break;
  case MDL_AFF_TX_DIV:
  case MDL_AFF_TR_DIV:
  case MDL_AFF_TR_ROT:
  case MDL_AFF_TR_ROT_DIV:
  case MDL_AFF_COMPLET:
    degre = 1; break;
  case MDL_QUA_PAN_DIV:
  case MDL_QUA_PAN_TILT:
  case MDL_QUA_PAN_TILT_DIV:
  case MDL_QUA_2D:
  case MDL_QUA_COMPLET:
    degre = 2; break;
  default:
    degre = -1;
  }

  return degre;
}


/*!

  Returns a string describing the id motion model.

*/
string CMotion2DModel::idToString()
{
  string s;

  switch (id_model) {
  case MDL_TX:               s = "MDL_TX"; break;
  case MDL_TY:               s = "MDL_TY"; break;
  case MDL_TR:               s = "MDL_TR"; break;
  case MDL_AFF_TX_DIV:       s = "MDL_AFF_TX_DIV"; break;
  case MDL_AFF_TR_DIV:       s = "MDL_AFF_TX_DIV"; break;
  case MDL_AFF_TR_ROT:       s = "MDL_AFF_TR_ROT"; break;
  case MDL_AFF_TR_ROT_DIV:   s = "MDL_AFF_TR_ROT_DIV"; break;
  case MDL_AFF_TX_NULL:      s = "MDL_AFF_TX_NULL"; break;
  case MDL_AFF_TY_NULL:      s = "MDL_AFF_TY_NULL"; break;
  case MDL_AFF_DIV_NULL:     s = "MDL_AFF_DIV_NULL"; break;
  case MDL_AFF_ROT_NULL:     s = "MDL_AFF_ROT_NULL"; break;
  case MDL_AFF_HYP1_NULL:    s = "MDL_AFF_HYP1_NULL"; break;
  case MDL_AFF_HYP2_NULL:    s = "MDL_AFF_HYP2_NULL"; break;
  case MDL_AFF_COMPLET:      s = "MDL_AFF_COMPLET"; break;
  case MDL_QUA_PAN_DIV:      s = "MDL_QUA_PAN_DIV"; break;
  case MDL_QUA_PAN_TILT:     s = "MDL_QUA_PAN_TILT"; break;
  case MDL_QUA_PAN_TILT_DIV: s = "MDL_QUA_PAN_TILT_DIV"; break;
  case MDL_QUA_2D:           s = "MDL_QUA_2D"; break;
  case MDL_QUA_COMPLET:      s = "MDL_QUA_COMPLET"; break;
  default:                   s = "Unkown model"; break;
  }

  return (s);
}

/*!

  Transforms all the parameters \f$ A^t \f$ of the 2D polynomial motion model
  from \f$ (O, \vec{x}, \vec{y}) \f$ to \f$ (O^{'}, \vec{X}, \vec{Y}) \f$ where
  \f$ O = (X_c, Y_c) = (col, row) \f$ are the coordinates of the origin
  in \f$ (O^{'}, \vec{X}, \vec{Y}) \f$.

  \warning If used, the global illumination parameter \f$ \zeta \f$ is not
  affected by this function.

  Returns true if success, false otherwise.

*/

bool CMotion2DModel::changeRepere(double row, double col)
{
  double new_para[MAXCOEFSMODEL];
  Para para;
  int i;

  para.id_model = id_model;
  para.var_light = var_light;
  para.nb_para = Nb_Para_Modele(id_model);

  if (!para.nb_para)
  {
    printf("Le modele de mouvement specifie n'est pas implemente.\n");
    return false;
  }

  para.co_c = co_c;
  para.li_c = li_c;

  for (i=0; i<MAXCOEFSMODEL; i++)
    para.thet[i] = theta[i];

  if (chgt_repere(&para, new_para, row, col) == false)
    return false;

  // Mise a jour du nouveau modele
  // Le terme de variation d'eclairage n'est pas modifie, donc pas mis a jour.
  for (i=0; i<MDL_NMAX_COEF; i++)
    theta[i] = new_para[i];

  co_c = col;
  li_c = row;

  return true;
}


/*!

  For a given motion model, supplies the displacement in pixels \f[
  \vec{w}_{A}(p_{i}) = \left[ \begin{array}{l} u(p_{i}) \\ v(p_{i})
  \end{array} \right] = \left[ \begin{array}{l} d\_col \\ d\_row \end{array}
  \right] \f]

  for the specified point \f$ p_i = (col, row) \f$.

  \return true if success, false otherwise.

*/

bool CMotion2DModel::getDisplacement(double row, double col,
				     double &d_row, double &d_col)
{

  switch (model_degree(id_model)) {
  case 0: // Constant model
    d_col = theta[0];
    d_row = theta[1];
    break;

  case 1: { // Affine model
    double x_xg = col - co_c;
    double y_yg = row - li_c;
    d_col = theta[0] + theta[2] * x_xg + theta[3] * y_yg;
    d_row = theta[1] + theta[4] * x_xg + theta[5] * y_yg;
    break;
  }

  case 2: { // Quadratic model
    double x_xg = col - co_c;
    double y_yg = row - li_c;
    double x_xg2 = x_xg * x_xg;
    double y_yg2 = y_yg * y_yg;
    double x_xg_y_yg = x_xg * y_yg;
    d_col = theta[0] + theta[2] * x_xg + theta[3] * y_yg
      + theta[6] *x_xg2 + theta[7] * x_xg_y_yg + theta[8] *y_yg2;
    d_row = theta[1] + theta[4] * x_xg + theta[5] * y_yg
      + theta[9] *x_xg2 + theta[10] * x_xg_y_yg + theta[11] *y_yg2;
    break;
  }
  default:
    return false;
  }

  return true;
}

/*!

  Returns all the CMotion2DModel::MDL_NMAX_COEF parameter values \f$A\f$
  corresponding to the 2D polynomial motion model. The unused parameters are
  set to zero.

  \warning For a given motion model, the number of parameters
  of the polynomial model is accessible with getNParameters().

  \warning If the global illumination variation is estimated, the corresponding
  value is given by getVarLight().

  The general form of the model is given by:
  \f[
  \vec{w}_{A}(p_{i}) = \left[
  \begin{array}{l} u(p_{i}) \\ v(p_{i}) \end{array} \right] = B(p_i) A
  \f]

  \f[
  \vec{w}_{A}(p_{i}) = \left(
  \begin{array}{l} c_{1}\\ c_{2} \end{array} \right) + \left( \begin{array}{l}
  a_{1} \;\; a_{2}\\ a_{3} \;\; a_{4} \end{array} \right) \cdot \left(
  \begin{array}{l} x_{i}\\ y_{i} \end{array} \right) + \left(
  \begin{array}{l} q_{1} \;\; q_{2} \;\; q_{3}\\ q_{4} \;\; q_{5} \;\; q_{6}
  \end{array} \right) \cdot \left( \begin{array}{l} {x_{i}}^2\\
  x_{i} y_{i} \\ {y_{i}}^2 \end{array} \right)
  \f]

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

  \sa getVarLight(), getNParameters(), operator[]()

*/

void CMotion2DModel::getParameters(double parameters[MDL_NMAX_COEF])
{
  int i;

  for(i = 0; i < MDL_NMAX_COEF; i++)
    parameters[i] = theta[i];
}

/*!

  Indicates if the global illumination variation parameter \f$ \zeta \f$ is
  used. Returns true if the motion model involves this illumination parameter,
  and false otherwise.

  \sa setVarLight()

*/

bool CMotion2DModel::getVarLight()
{
  return (var_light);
}
/*!

  Indicates if the global illumination variation parameter \f$ \zeta \f$ is
  used. Returns true if the motion model involves it, and false otherwise. If
  true, the parameter \e parameter contains the value of \f$ \zeta \f$.

  \sa setVarLight()

*/

bool CMotion2DModel::getVarLight(double & parameter)
{
  // Test si estimation var eclairage.
  if (var_light)
    parameter = theta[12];    // Mise a jour du coef d'eclairage.
  else
    parameter = 0.0;

  return (var_light);
}

/*!

  Returns a motion model where the 2D parametric motion model parameters and
  the model origin are transformed considering a relative pyramid level given
  as input by \e delta_level.

  For example,
  - if \e delta_level is set to 0, returns the same motion model.

  - if \e delta_level is set to 1, returns a transformed motion model where the
    constant parameters are divided by 2 and the quadratic terms are multiplied
    by 2. The model origin coordinates are divided by two.

  - if \e delta_level is set to -1, returns a transformed motion model where
    the constant parameters are multiplied by 2 and the quadratic terms are
    divided by 2. The model origin coordinates are multiplied by two.

*/

CMotion2DModel CMotion2DModel::getModelLevel(int delta_level)
{
  CMotion2DModel newmodel = *this;
  float	 coef = 0.0;

  if (delta_level > 0)
    coef = 0.5;
  else if (delta_level < 0)
    coef = 2.0;
  else
    return newmodel;

  delta_level = abs(delta_level);

  for(int i=0; i < delta_level; i++) {
    newmodel.li_c *= coef;
    newmodel.co_c *= coef;

    newmodel.theta[0] *= coef;	/* constant parameters */
    newmodel.theta[1] *= coef;	/* constant parameters */

    for(int j = 6;j < 12; j++)
      newmodel.theta[j] /= coef;	/* quadratic parameters */
  }

  return (newmodel);
}


/*!

  Sets the 2D parametric motion model parameters. The number of parameters can
  be known by using getNParameters().

  The general form of the model is given by:
  \f[
  \vec{w}_{A}(p_{i}) = \left[
  \begin{array}{l} u(p_{i}) \\ v(p_{i}) \end{array} \right] = B(p_i) A
  \f]

  \f[
  \vec{w}_{A}(p_{i}) = \left(
  \begin{array}{l} c_{1}\\ c_{2} \end{array} \right) + \left( \begin{array}{l}
  a_{1} \;\; a_{2}\\ a_{3} \;\; a_{4} \end{array} \right) \cdot \left(
  \begin{array}{l} x_{i}\\ y_{i} \end{array} \right) + \left(
  \begin{array}{l} q_{1} \;\; q_{2} \;\; q_{3}\\ q_{4} \;\; q_{5} \;\; q_{6}
  \end{array} \right) \cdot \left( \begin{array}{l} {x_{i}}^2\\
  x_{i} y_{i} \\ {y_{i}}^2 \end{array} \right)
  \f]

  The table below gives the correspondence between the index of the table \e
  parameters[index] and the corresponding parameter of the model.

  \f[
  \begin{tabular}{|l||c|c|c|c|c|c|c|c|c|c|c|c|}
  \hline
  index  & 0     & 1     & 2 & 3 & 4 & 5 & 6 & 7 & 8 & 9& 10& 11\\
  \hline
  A(index) & $c_1$ & $c_2$ & $a_1$ &$a_2$ &$a_3$ &$a_4$ &$q_1$ &$q_2$ &$q_3$ &$q_4$ & $q_5$ &$q_6$ \\
  \hline
  \end{tabular}
  \f]

  \sa getNParameters(), setVarLight()

*/

void CMotion2DModel::setParameters(const double parameters[MDL_NMAX_COEF])
{
  int i;

  for(i = 0; i < MDL_NMAX_COEF; i++)
    theta[i] = parameters[i];
}


/*!

  Sets all the 2D polynomial motion model parameters and the global
  illumination variation parameter to zero.

  \sa setParameters(), setVarLight()

*/

void CMotion2DModel::reset()
{
  for(int i = 0; i < MAXCOEFSMODEL; i++)
    theta[i] = 0.0;
}

/*!

  Computes the inverse motion model. This transformation is only available for
  constant or affine motion models.

  \return true if success, false otherwise.

*/

bool CMotion2DModel::invert(CMotion2DModel &inv)
{
#define  EPSILON  1.E-10

  Para		paramet;
  double	coefs[MAXCOEFSMODEL];
  double u,v;              /* translation vector */
  double a, b, c, d;      /* matrix coefficients */
  double delta;            /* matrix determinant */
  double u_,  v_;          /* inverse  translational coeficient */
  double a_, b_, c_,  d_;  /* inverse  matrix coefficients */

  // Updates the motion model structure
  paramet.nb_para = getNParameters();

  if (!paramet.nb_para) {
    cout << "Error: Motion model not implemented.\n";
    return false;
  }

  paramet.var_light = getVarLight();
  paramet.id_model  = getIdModel();

  if (paramet.id_model == 2) {
    cout << "Error: Could not invert a quadratic motion model.\n";
    return false;
  }

  getOrigin(paramet.li_c, paramet.co_c);

  for (int i = 0; i < MAXCOEFSMODEL; i++)
    paramet.thet[i] = 0.0;

  getParameters(paramet.thet);

  /* obtient les parametres dans le repere (0, 0)	*/
  if (chgt_repere(&paramet, coefs, 0, 0) == false)
    return false;

  u = coefs[0];  v = coefs[1];
  a = coefs[2];  b = coefs[3];
  c = coefs[4];  d = coefs[5];

  a += 1.0;  d+= 1.0; /* on veut le champ des deplacements, pas des vitesses */

  delta = a*d - b*c;

  if ((-EPSILON < delta) && (delta < EPSILON)) {
    printf("\nCan not warp image...\n");
    return false;
  }

  /* coefficients of the inverse transform */
  a_ =  d /delta;      b_ = - b /delta;        u_ = (b*v - d*u)/delta;
  c_ = -c /delta;      d_ =   a /delta;        v_ = (c*u - a*v)/delta;

  /* coefficients of the direct speed transform */
  a_ -= 1.0;	d_ -= 1.0;

  paramet.thet[0] = u_;
  paramet.thet[1] = v_;
  paramet.thet[2] = a_;
  paramet.thet[3] = b_;
  paramet.thet[4] = c_;
  paramet.thet[5] = d_;
  paramet.li_c = 0.0;
  paramet.co_c = 0.0;

  /* obtient les parametres dans le repere (0, 0)	*/
  if (chgt_repere(&paramet, coefs, li_c, co_c) == false)
    return false;

  inv.setParameters(coefs);
  inv.setOrigin(li_c, co_c);
  inv.setIdModel(id_model);

#undef EPSILON
  return true;
}

/*!

  Assigns a copy of \e model to this motion model and returns a reference to
  this motion model.

*/
void CMotion2DModel::operator=(const CMotion2DModel &model)
{
  for(int i = 0; i < MAXCOEFSMODEL; i++)
    theta[i] = model.theta[i];	// Valeur Initiale des param�tres � estimer.

  li_c = model.li_c;
  co_c = model.co_c;

  var_light = model.var_light;
  id_model  = model.id_model;
}
