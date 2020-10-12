/*
 * Copyright (c) 2010-2020 Idiap Research Institute
 *
 * See file COPYING for the licence associated with this software.
 */

/*
#include <pantheios/pantheios.hpp>
#include <pantheios/inserters.hpp>
*/
//#include "Pattern.h"
#include "image_processing/ipcv_Image.h"
#include "image_processing/ip_BoundingBox.h"
#include "image_processing/ip_TrackingMemory.h"


//#define ACC_DETECTIONS 1

//using namespace Torch;
//using namespace pantheios;

namespace ImageProcessing
{

  ip_TrackingMemory::ip_TrackingMemory(int width, int height, float update_factor)
  {
    m_rUpdateFactor = update_factor;

    m_pImage = new ipcv_Image<float>(height, width);
    m_ImageBB.initPosAndSize(0,0,width,height);
  }

  ip_TrackingMemory::~ip_TrackingMemory()
  {
    delete m_pImage;
  }

  void ip_TrackingMemory::init()
  {
    m_pImage->init(0.0);
  }


  void ip_TrackingMemory::update(ip_BoundingBox* bb, int nb_bb)
  {
    // "fade out" old detections
    m_pImage->multiply(1.0-m_rUpdateFactor);

    // add new regions
    for(int i=0; i<nb_bb; i++)
    {
      bb[i].enlarge(2.5, 2);
      bb[i].intersection(m_ImageBB);
      if (bb[i].m_iWidth>1 && bb[i].m_iHeight>1)
      {
	m_pImage->setROI(bb[i].m_iFirstColumn, bb[i].m_iFirstLine, bb[i].m_iWidth, bb[i].m_iHeight);
	m_pImage->add(m_rUpdateFactor);
      }
    }

    m_pImage->resetROI();
  }

  void ip_TrackingMemory::save(const char* filename)
  {
    m_pImage->save(filename);
  }

}
