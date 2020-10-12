// Copyright (c) 2019-2020 Idiap Research Institute
//
// ModelDumperCs_CSVFormat - class to dump main model data in MOT challenge format
//
// Authors: Olivier Canévet (olivier.canevet@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef _ModelDumper_MOTFormat_h_
#define _ModelDumper_MOTFormat_h_

#include "ModelDumper.h"

class ModelDumper_MOTFormat: public ModelDumper
{
public:
  ModelDumper_MOTFormat(const std::string& dir_name, const MainModel* model);
  virtual ~ModelDumper_MOTFormat();
  virtual std::string name() const { return "[ModelDumper_MOTFormat]"; }
  virtual void update();

private:
  /// Each face will have a different ID for the detection file
  int m_face_id;

  /// File to save the detection from face detector
  std::ofstream m_detection_file;

  /// File to save the output of the tracker
  std::ofstream m_tracking_file;

  /// File to save the output of the VFOA
  std::ofstream m_vfoa_file;

};



#endif /* _ModelDumper_MOTFormat_h_ */
