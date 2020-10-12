/*
 * Copyright (c) 2010-2020 Idiap Research Institute
 *
 * See file COPYING for the licence associated with this software.
 */

#ifndef IP_TRACKING_MEMORY_H
#define IP_TRACKING_MEMORY_H

#include "Parameters.h"
#include "ip_Image.h"
//#include "ip_ColorDef.h"
#include "ip_BoundingBox.h"


namespace ImageProcessing
{
class ip_BoundingBox;

class ip_TrackingMemory
{
  private:
    ip_Image<float>* m_pImage;
    float m_rUpdateFactor;
    ip_BoundingBox m_ImageBB;

  public:
    ip_TrackingMemory(int width, int height, float update_factor);
    ~ip_TrackingMemory();

    ip_Image<float>* getImage() { return m_pImage; };
    void init();
    void update(ip_BoundingBox* bb, int nb_bb);
    void save(const char* filename);
    float getValue(int x, int y) { return m_pImage->value(y, x); };
    

};



}

#endif
