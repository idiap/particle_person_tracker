/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu
  35042 Rennes Cedex

  http://www.irisa.fr

*/

#include <cmotion2d/CReader.h>

/*!
  \file CReader.cpp
  \brief Definition of the CReader class.
*/

/*!

  \class CReader

  \brief The CReader class implements an image or video stream reader.

*/

/*!

  Image or video stream reader constructor.

*/
CReader::CReader()
{
  frame = 1;
  nbsubsample = 0;
}

/*!

  Set the image or video stream filename to read.
  \sa getFileName()

*/
void CReader::setFileName(const char *filename)
{
  streamName = filename;
}

/*!

  Set the image or video stream filename to read.
  \sa getFileName()

*/
void CReader::setFileName(string filename)
{
  streamName = filename;
}

/*!

  Set the image or video stream frame number to read.

  \return Allways true.

*/
bool CReader::setFrameNumber(unsigned long framenumber)
{
  frame = framenumber;

  return true;
}



