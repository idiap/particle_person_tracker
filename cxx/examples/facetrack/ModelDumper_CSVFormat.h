// Copyright (c) 2019-2020 Idiap Research Institute
//
// ModelDumperCs_CSVFormat - class to dump main model data in .csv file
//
// Authors: Olivier Canévet (olivier.canevet@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef _ModelDumper_CSVFormat_h_
#define _ModelDumper_CSVFormat_h_

#include "ModelDumper.h"

class ModelDumper_CSVFormat: public ModelDumper
{
public:
  ModelDumper_CSVFormat(const std::string& dir_name, const MainModel* model);
  virtual ~ModelDumper_CSVFormat();
  virtual std::string name() const { return "[ModelDumper_CSVFormat]"; }
  virtual void update();

private:
  /// File to save the output of the tracker
  std::ofstream m_tracking_file;
};



#endif /* _ModelDumper_CSVFormat_h_ */
