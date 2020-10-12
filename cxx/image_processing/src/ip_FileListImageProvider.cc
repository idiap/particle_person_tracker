// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_FileListImageProvider - image provider using a list of files
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// SYSTEM INCLUDES
#include <boost/filesystem/operations.hpp>   // file system
#include <boost/version.hpp>                 // boost version
#include <sstream>                           // string streams
#include <iostream>                          // IO streams
#include <fstream>                           // STL input file stream

// LOCAL INCLUDES
#include "image_processing/ip_FileListImageProvider.h"
#include "image_processing/ip_Exceptions.h"  // exceptions


using namespace std;
namespace fs = boost::filesystem;


namespace ImageProcessing {


ip_FileListImageProvider::
ip_FileListImageProvider(const string& filename_template,
                         int width,
                         int height,
                         double scale,
                         int fps,
                         bool offline):
  ip_ResizableImageProvider(width, height, scale),
  mFps(fps),
  mImageId(0),
  mOfflineMode(offline)
{
  derive_input_files(filename_template, mFiles);
  initialize_size(mFiles.front());
  mFrameDelay_ms = 1000.0 / mFps;
  mFirstFrame = true;

  if(!mFiles.empty())
    {
      mFileToPrepare = mFiles.begin();

      string first_fname = mFiles.front();
      IplImage* pFrame = cvLoadImage(first_fname.c_str());

      // set_preferred_image_size(pFrame);
      std::cout << name() << " Allocate memory with first image" << std::endl;
      std::cout << name() << " " << mPreferredWidth << "x" << mPreferredHeight
                << std::endl;

      IplImage *buffer_image = cvCreateImage(cvSize(mPreferredWidth,
                                                    mPreferredHeight),
                                             pFrame->depth,
                                             pFrame->nChannels);
      image(buffer_image);
    }
}


unsigned
ip_FileListImageProvider::
image_id() const
{
  return mImageId;
}


int
ip_FileListImageProvider::
get_device_width() const
{
  int width = -1;
  if(!mFiles.empty())
    {
      std::string image_name = mFiles.front();
      IplImage *frame = cvLoadImage(image_name.c_str());
      width = frame->width;
      cvReleaseImage(&frame);
    }
  return width;
}


int
ip_FileListImageProvider::
get_device_height() const
{
  int height = -1;
  if(!mFiles.empty())
    {
      std::string image_name = mFiles.front();
      IplImage *frame = cvLoadImage(image_name.c_str());
      height = frame->height;
      cvReleaseImage(&frame);
    }
  return height;
}


void
ip_FileListImageProvider::
initialize_size(const std::string& device_name)
{
  mResize = !set_preferred_image_size();

  if(!mFiles.empty())
    {
      std::string image_name = mFiles.front();
      IplImage *frame = cvLoadImage(image_name.c_str());
      IplImage *buffer_image = cvCreateImage(cvSize(mPreferredWidth,
                                                    mPreferredHeight),
                                             frame->depth,
                                             frame->nChannels);
      image(buffer_image);
      cvReleaseImage(&frame);
    }

}

void
ip_FileListImageProvider::
recompute_image(IplImage *buffer_image,
                const ip_RoiWindow& roi,
                boost::posix_time::ptime& time)
{
  if (!mOfflineMode) {
    if (!mFirstFrame) {
      boost::posix_time::time_duration time_delay = (time - mLastImageTimeStamp);
      long time_delay_ms = time_delay.total_milliseconds();
      int num_frames_to_skip = static_cast<int>(time_delay_ms /
                                                mFrameDelay_ms + 0.5);
      if (num_frames_to_skip > 0) {
        for (int i = 0; (i < num_frames_to_skip - 1) &&
               (mFileToPrepare != mFiles.end()); ++i)
          {
            std::cout << name() << " Skipping frame" << std::endl;
            mFileToPrepare++;
            mImageId++;
        }
      }
    }
  }

  if (!mFirstFrame)
    {
      mFileToPrepare++;
      mImageId++;
    }

  if (mFileToPrepare == mFiles.end()) {
    throw ip_Exception("No more frames available!");
  }

  // std::cout << name() << " Loading " << mFileToPrepare->c_str() << std::endl;

  IplImage *frame = cvLoadImage(mFileToPrepare->c_str());

  boost::filesystem::path p(mFileToPrepare->c_str());
  // std::cout << "p " << p << std::endl;
  std::istringstream iss(p.stem().c_str());
  // std::cout << "iss.str() " << iss.str() << std::endl;
  try { iss >> mDataTime; }
  catch (...) { mDataTime = 0; }

  mFirstFrame = false;
  mLastImageTimeStamp = time;

  resize_or_copy(frame, buffer_image);

  // try
  //   {
  //     cvResize(frame, buffer_image, CV_INTER_LINEAR);
  //   }
  // catch (...)
  //   {
  //     cout << "Exception caught in file list data provider!" << endl << flush;
  //   }

  // // std::cout << name() << " get image " << mImageId << std::endl;
  // std::ostringstream oss;
  // oss << std::setfill('0') << std::setw(6) << mImageId << ".jpg";
  // cvSaveImage(oss.str().c_str(), frame);

  cvReleaseImage(&frame);
}


// void
// ip_FileListImageProvider::
// set_preferred_image_size()
// {
//   int device_width = image->width;
//   int device_height = image->height;

//   mPreferredHeight = device_height;
//   mPreferredWidth = device_width;

//   std::cout << "[FileListImageProvider] Image dimensions "
//             << device_width << "x" << device_height
//             << std::endl;

//   // if (mPreferredWidth == -1) {
//   //   if (mPreferredHeight == -1) {
//   //     mPreferredWidth = device_width;
//   //     mPreferredHeight = device_height;
//   //   } else {
//   //     mPreferredWidth = mPreferredHeight * device_width / device_height;
//   //   }
//   // } else if (mPreferredHeight == -1) {
//   //   mPreferredHeight = mPreferredWidth * device_height / device_width;
//   // }

//   mPreferredWidth = static_cast<int>(mPreferredWidth*mScale);
//   mPreferredHeight = static_cast<int>(mPreferredHeight*mScale);

//   std::cout << "[FileListImageProvider] Selected dimensions "
//             << mPreferredWidth << "x" << mPreferredHeight
//             << std::endl;
// }


void
ip_FileListImageProvider::
derive_input_files(const string& source,
                   list<std::string> & fname_list)
{
    fs::path source_path(source);
    if (source_path.extension() == ".txt") {
        read_input_files_txt(source, fname_list);
    } else {
        derive_input_files_pattern(source, fname_list);
    }
}


void
ip_FileListImageProvider::
read_input_files_txt(const string& fname,
                     list<string> & fname_list)
{
  ifstream ifs(fname.c_str());
  ostringstream oss;
  if (!ifs) {
    oss << endl << "ERROR: Cannot open image provider file " << fname
        << "!" << endl;
    throw ip_Exception(oss.str());
  }

  // get full path of the list file
  fs::path fpath(fname);
#if BOOST_VERSION > 104500
  fs::path full_fpath = fs::absolute(fpath);
#else
  fs::path full_fpath = fs::complete(fpath);
#endif
  fs::path basedir_fpath = full_fpath.parent_path();

  string s;
  while (getline(ifs, s))
    {
      fs::path img_fpath(s);
      //        if (img_fpath.is_absolute()) {
      //            fname_list.push_back(img_fpath.string());
      //        } else {
      // fs::path full_img_fpath = basedir_fpath / img_fpath;
      fs::path full_img_fpath = img_fpath;
      fname_list.push_back(full_img_fpath.string());
  }

}


void
ip_FileListImageProvider::
derive_input_files_pattern(const std::string& pattern,
                           std::list<std::string> & fname_list)
{
  fs::path file_path( pattern.c_str() /*, fs::native*/ );
  fs::path dir_name = file_path.parent_path();
  fs::path fname_template = file_path.filename();
  string fname_template_str = fname_template.string();

  if (!fs::exists(dir_name)) {
    ostringstream oss;
    oss << "Input file directory " << dir_name;
    oss << " does not exist!";
    throw ip_Exception(oss.str());
  }

  size_t param_pos = fname_template_str.find('%');
  if (param_pos != string::npos) {
    string prefix(fname_template_str.substr(0, param_pos));
    istringstream iss(fname_template_str.substr(param_pos + 1));
    unsigned num_digits = 0;
    iss >> num_digits;
    char c;
    iss >> c;
    if (c != 'd' || !num_digits) {
      ostringstream oss;
      oss << "Invalid pattern provided: " << fname_template_str;
      oss << ". Expected %0Nd specified for some positive number N.";
      throw ip_Exception(oss.str());
    }
    string suffix;
    iss >> suffix;

    // cout << prefix << " " << num_digits << " " << suffix << endl;
    // cout << '\"' << prefix << "\" \"" << suffix << '\"' << endl;

    fs::directory_iterator end_iter;
    for (fs::directory_iterator dir_iter(dir_name); dir_iter != end_iter;
         ++dir_iter) {
      if (fs::is_regular_file(dir_iter->status())) {
        fs::path listed_fpath = dir_iter->path();
        fs::path listed_fname = listed_fpath.filename();
        string listed_fname_str = listed_fname.string();
        // cout << '\"' << listed_fname_str.substr(0, param_pos) <<
        //     "\" \"" << listed_fname_str.substr(param_pos+num_digits)
        //     << '\"' << endl;
        if ((listed_fname_str.substr(0, param_pos) == prefix) &&
            (listed_fname_str.substr(param_pos+num_digits) == suffix)) {
          // cout << listed_fpath.string() << endl;
          fname_list.push_back(listed_fpath.string());
        }
      }
    }

    fname_list.sort();
    ostream_iterator<string> out_it(cout, "\n");
    copy(fname_list.begin(), fname_list.end(), out_it);
  }

}

} // namespace ImageProcessing
