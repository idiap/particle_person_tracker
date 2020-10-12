/*

  Copyright (c) 1995-2005 by INRIA.
  All Rights Reserved.

  This software was developed at:
  IRISA/INRIA Rennes
  Campus Universitaire de Beaulieu
  35042 Rennes Cedex

  http://www.irisa.fr

*/

#include <cmotion2d/CWriter.h>

/*!
  \file CWriter.cpp
  \brief Definition of the CWriter class.
*/

/*!

  \class CWriter

  \brief The CWriter class implements an image or video stream writer.

*/

/*!

  Image or video stream writer constructor.

*/
CWriter::CWriter()
{
  frame = 1;
}

/*!

  Set the image or video stream filename to save to disk.
  \sa getFileName()

*/
void CWriter::setFileName(const char *filename)
{
  streamName = filename;
}

/*!

  Set the image or video stream filename to save to disk.
  \sa getFileName()

*/
void CWriter::setFileName(string filename)
{
  streamName = filename;
}

/*!
  
  Set the image or video stream frame number to write.

  \return Allways true.

*/
bool CWriter::setFrameNumber(unsigned long framenumber)
{
  frame = framenumber;

  return true;
}



