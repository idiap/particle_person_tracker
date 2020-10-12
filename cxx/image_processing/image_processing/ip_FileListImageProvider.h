// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_FileListImageProvider - image provider using a list of files
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __IP_FILELISTIMAGEPROVIDER_H__
#define __IP_FILELISTIMAGEPROVIDER_H__

// SYSTEM INCLUDES
#include <string>                    // STL string
#include <list>                      // STL list
#include <highgui.h>                 // for CvCapture

// LOCAL INCLUDES
#include "image_processing/ip_ResizableImageProvider.h"

namespace ImageProcessing {

/// @brief Class to represent an image provider for file list data
///
/// This class defines an image provider that uses a list of files
/// as data source.
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    27.11.2011

class ip_FileListImageProvider: public ip_ResizableImageProvider
{

public:

  /// Constructor to initialize an image provider based on a list of files
  /// @param filename_template  Image file name template
  /// @param fps    Preferred fps rate, 25 fps by default
  /// @param width  Preferred image width, -1 for the default device width
  /// @param height Preferred image height, -1 for the default device height
  ip_FileListImageProvider(const std::string& filename_template,
                           int width = -1,
                           int height = -1,
                           double scale = 1.0,
                           int fps = 25,
                           bool offline = true);

  virtual std::string name() { return "[ip_FileListImageProvider]"; }

  /// Overrides base class method, returns IP_IMG_PROVIDER_WEBCAM_DATA
  /// @return Provider's ID
  /// @see ip_ImageProviderType
  virtual ip_ImageProviderType id() const { return IP_IMG_PROVIDER_FILE_LIST_DATA; };

  /// Overrides base class method, returns FPS (frames per second) rate
  /// @return FPS (frames per second) rate.
  virtual float fps() const { return mFps; }

  /// Overrides base class method.
  /// Returns current image ID. The ID is equal to image number in the image
  /// file sequence (image 1-based index) in both online and offline modes.
  /// @return Current image ID.
  virtual unsigned image_id() const;

protected:

  /// Overrides base class method, obtains the most recent image
  /// @param image Image to write results to
  virtual void recompute_image(IplImage* image, const ip_RoiWindow& roi,
                               boost::posix_time::ptime& time);


private:
  virtual void initialize_size(const std::string& device_name="");
  virtual int get_device_width() const;
  virtual int get_device_height() const;

  // assigns the preferred size for input images
  // void set_preferred_image_size(IplImage * image);

  /// Fills the list of input files based on the provided pattern or
  /// a text file containing their names
  /// @param source Text file name containing a list of files or
  ///               a string template that defines a set of files
  /// @param fname_list File name list to store image file names to
  void derive_input_files(const std::string& source,
                          std::list<std::string> & fname_list);

  /// Read image file names from a text file into a string list
  /// @param fname Text file name to read image file names from
  /// @param fname_list File name list to store image file names to
  void read_input_files_txt(const std::string& fname,
                            std::list<std::string> & fname_list);

  /// Derive image file names from a pattern and store them into a string list
  /// @param pattern String pattern to derive a set of image files
  /// @param fname_list File name list to store image file names to
  void derive_input_files_pattern(const std::string& pattern,
                                  std::list<std::string> & fname_list);

  std::list<std::string> mFiles; // files list
  std::list<std::string>::iterator mFileToPrepare;
  boost::posix_time::ptime mLastImageTimeStamp;
  bool mFirstFrame;              // current frame is the first one?
  float mFrameDelay_ms;          // time delay between two consecutive frames
  float mFps;                   // FPS rate
  unsigned mImageId;             // current image ID
  const bool mOfflineMode;       // process every frame if true

};

} // namespace ImageProcessing

#endif // __IP_FILELISTIMAGEPROVIDER_H__
