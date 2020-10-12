/*
 * Copyright (c) 2010-2020 Idiap Research Institute
 *
 * See COPYING file for the complete license text.
 */

//#include <pantheios/pantheios.hpp>
//#include <pantheios/inserters.hpp>
//#include "Pattern.h"
#include "image_processing/ipcv_Image.h"
#include "image_processing/ip_TrackingMemory.h"
#include "image_processing/ip_CreateObject2.h"

#include <cmath>

#define ATAN_FAST(x)  (M_PI_4*x - x*(fabs(x) - 1)*(0.2447 + 0.0663*fabs(x)))
//#define ACCU_DETECTIONS 1

using namespace OpenCvPlus;
//using namespace pantheios;

/////////////////////////// GLOBAL CONSTANTS /////////////////////////////////

// value for facetrack
static const float TRACKER_MEMORY_THRESHOLD_VALUE = 0.5;
// value for facetrack_cs
//static const float TRACKER_MEMORY_THRESHOLD_VALUE = 0.1;

//////////////////////////////// PUBLIC //////////////////////////////////////

namespace ImageProcessing
{

ip_BoundingBox
roiWindow2BoundingBox(ip_RoiWindow rw)
{
  ip_BoundingBox bb;
  bb.initPosAndSize(rw.m_iFirstColumn, rw.m_iFirstRow, rw.m_iWidth, rw.m_iHeight);
  return bb;
}

ip_CreateObject2::
ip_CreateObject2(ip_TrackingMemoryImage *trackmem,
                 int width,
                 int height,
                 float update_factor)
{
    //m_pDetector = det;
    m_pTrackingMemory = trackmem;
    m_rUpdateFactor = update_factor;
    m_iUpdatesWithoutDetector = 0;

    m_ImageBB.initPosAndSize(0,0,width,height);
    m_pImage = new ipcv_Image<float>(height, width);
#ifdef RESEARCH
#ifdef SIMPLE_ADD
    m_pOldImage = new ipcv_Image<float>(height, width);
    m_pOldImage->fill(0);
    m_pOldOldImage = new ipcv_Image<float>(height, width);
    m_pOldOldImage->fill(0);
    m_pNewImage = new ipcv_Image<float>(height, width);
    m_pNewImage->fill(0);
#endif
    m_pDetectionsImage = new ipcv_Image<float>(height, width);
    m_pFacePosterior = new ipcv_Image<float>(height, width);
    m_pNotFacePosterior = new ipcv_Image<float>(height, width);

    m_pFacePosterior->fill(0.000001);
    m_pNotFacePosterior->fill(0.999999);
#endif // RESEARCH

    //det->setLongtermDetectionImage(m_pImage);
  }

ip_CreateObject2::~ip_CreateObject2()
{
  delete m_pImage;
#ifdef RESEARCH
#ifdef SIMPLE_ADD
  delete m_pOldImage;
  delete m_pOldOldImage;
  delete m_pNewImage;
#endif
  delete m_pDetectionsImage;
  delete m_pFacePosterior;
  delete m_pNotFacePosterior;
#endif // RESEARCH
}

void
ip_CreateObject2::
init()
{
  m_pImage->init(0.0);
}


void
ip_CreateObject2::
update(bool face_detector_reported,
const std::list<cvp_FaceDescriptor>& faces)
{
  m_iUpdatesWithoutDetector++;

  if(!face_detector_reported)
    return;

  //int nb_det = faces.size();
  std::list<cvp_FaceDescriptor>::const_iterator it;

  m_pDetectionsImage->fill(0);
  for(it = faces.begin(); it != faces.end(); ++it)
    {
      ip_BoundingBox bb =
        roiWindow2BoundingBox(ip_RoiWindow::from_CvRect(it->m_FaceRegion));
      bb.intersection(m_ImageBB);
      if (bb.m_iWidth>0 && bb.m_iHeight>0)
	m_pDetectionsImage->setROI(bb.m_iFirstColumn,
                                   bb.m_iFirstLine,
                                   bb.m_iWidth,
                                   bb.m_iHeight);
      m_pDetectionsImage->fill(1);
    }
  m_pDetectionsImage->resetROI();

  //////////////////////////////////////////////////////////////////////
  // See article [1] (defined in header of ip_CreateObject2.h)
  //////////////////////////////////////////////////////////////////////
  p_d_v = 0.9999; // 1-fa where fa=0.0001
  pd_v =  1.0 - p_d_v;
  p_dv =  0.4; // md=0.4 in article, section III.A.2
  pdv =   1.0 - p_dv;
  //////////////////////////////////////////////////////////////////////

  // if (m_iUpdatesWithoutDetector>10)
  //   {
  //     // std::cout << "IF" << std::endl;
  //     p_d_v = 0.9999;
  //     pd_v =  0.0001;
  //     p_dv =  0.4;
  //     pdv =   0.6;
  //   }
  // else if (m_iUpdatesWithoutDetector>2)
  //   {
  //     // std::cout << "ELSE IF" << std::endl;
  //     // See article [1] (defined in header of ip_CreateObject2.h)
  //     p_d_v = 0.9999; // 1-fa where fa=0.0001
  //     pd_v =  1.0 - p_d_v;
  //     p_dv =  0.4; // md=0.4 in article, section III.A.2
  //     pdv =   1.0 - p_dv;
  //   }
  // else
  //   {
  //     // std::cout << "ELSE" << std::endl;
  //     p_d_v = 0.99;
  //     pd_v =  0.01;
  //     p_dv =  0.4;
  //     pdv =   0.6;
  //   }

  m_iUpdatesWithoutDetector=0;

  float vdl, _vdl, den, plv, pl_v;
  float pmmv, pmm_v;
  float a_lv, mu_lv, sigma_lv, a_l_v, mu_l_v, sigma_l_v;
  /*
  // old values used for FG paper
  a_lv =  1.6848;
  mu_lv =  0.049701;
  sigma_lv =  0.23865;
  a_l_v = 732.58;
  mu_l_v = -0.14282;
  sigma_l_v = 0.049358;
  */

  /*
  // values for facetrack
  a_lv =  4.5436e+07;
  mu_lv = -2.6841;
  sigma_lv =  0.48851;
  a_l_v =  3.0663e+04;
  mu_l_v = -0.27237;
  sigma_l_v =  0.067659;
  */

  // values for facetrack_cs
  //    a_lv =  1.6848;
  //    mu_lv =  0.049701;
  //    sigma_lv =  0.23865;
  //    a_l_v = 732.58;
  //    mu_l_v = -0.14282;
  //    sigma_l_v = 0.049358;

  float minvalue = 1e-2;
  float maxvalue = 1.0/minvalue;
  float oneoverpi = 1.0/M_PI;
  float a = oneoverpi;

  float tmval;
  for(int i=0; i<m_pDetectionsImage->nbLines(); i++)
    for(int j=0; j<m_pDetectionsImage->nbColumns(); j++) {
      vdl = m_pFacePosterior->getVal(i,j);
      _vdl = m_pNotFacePosterior->getVal(i,j);

      tmval = m_pTrackingMemory->value(j, i);
      plv = a * (ATAN_FAST(1.8 * (tmval - TRACKER_MEMORY_THRESHOLD_VALUE))) + 0.5;
      pl_v = 1 - plv;
      pmmv = 1.0;
      pmm_v = 1.0;

      if (m_pDetectionsImage->getVal(i,j)>0) {
        // detection at this image point
        den = pdv*plv*pmmv*vdl + pd_v*pl_v*pmm_v*_vdl;
        m_pFacePosterior->value(i,j)= pdv * plv * pmmv * vdl / den;
        m_pNotFacePosterior->value(i,j)= pd_v * pl_v * pmm_v * _vdl / den;
      } else {
        // no detection at this image point
        den = p_dv*plv*pmmv*vdl + p_d_v*pl_v*pmm_v*_vdl;
        m_pFacePosterior->value(i,j)= p_dv * plv * pmmv * vdl / den;
        m_pNotFacePosterior->value(i,j)= p_d_v * pl_v * pmm_v * _vdl / den;
      }

      float ratio = m_pFacePosterior->value(i,j)/m_pNotFacePosterior->value(i,j);
      if (ratio<minvalue) {
        m_pFacePosterior->value(i,j) = minvalue*m_pNotFacePosterior->value(i,j);
        ratio = m_pFacePosterior->value(i,j)/m_pNotFacePosterior->value(i,j);
      }
      if (ratio>maxvalue) {
        m_pNotFacePosterior->value(i,j) = m_pFacePosterior->value(i,j)/maxvalue;
        ratio = m_pFacePosterior->value(i,j)/m_pNotFacePosterior->value(i,j);
      }
      m_pImage->value(i,j) = ratio;
    }

  m_pImage->resetROI();
}

void
ip_CreateObject2::
save(const char* filename)
{
  m_pImage->save(filename);
}

}
