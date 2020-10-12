// Copyright (c) 2011-2020 Idiap Research Institute
//
// MainWindow_ColourSegmentationTracker - class to visualise main model data
//                                        for colour segmentation-based tracking
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __MAINWINDOW_COLOURSEGMENTATIONTRACKER_H__
#define __MAINWINDOW_COLOURSEGMENTATIONTRACKER_H__

// LOCAL INCLUDES
#include "MainModel_ColourSegmentationTracker.h"            // main model

/// @brief Class to visualise colour segmentation-based tracking main model data
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    09.02.2011

class MainWindow_ColourSegmentationTracker {

public:
    // LIFECYCLE

    /// Constructor
    MainWindow_ColourSegmentationTracker(
            const MainModel_ColourSegmentationTracker& model);
    /// Destructor
    ~MainWindow_ColourSegmentationTracker();

    // OPERATIONS
    const IplImage * update();

private:

    static void blend_rect(IplImage * image8u3,
            const ImageProcessing::ip_RoiWindow& roi);

    void render_tracker(BICV::bicv_ColourSegmentationTracker * tracker,
            IplImage * source_image, IplImage * image8u3,
            const CvScalar & colour, unsigned tracker_number);

    void show_tracker_colour_classification(
            IplImage * src_image8u3, IplImage * dst_image8u3,
            BICV::bicv_ColourSegmentationTracker * tracker);

    void show_tracker_colour_probability(
            BICV::bicv_ColourSegmentationTracker * tracker,
            IplImage * image8u3, bool isPrior = false, bool isMap = false);

    void show_detected_faces(IplImage * image8u3);

    void show_motion_support(
            BICV::bicv_ColourSegmentationTracker * tracker,
            IplImage * image8u3,
            const CvScalar & colour, float coeff);

    void show_fps(IplImage * image8u3);

    void show_pose_probabilities(
            BICV::bicv_ColourSegmentationTracker * tracker,
            IplImage * image8u3);

    void show_palettes(BICV::bicv_ColourSegmentationTracker * tracker,
            IplImage * image8u3);
    void show_pims(BICV::bicv_ColourSegmentationTracker * tracker,
            IplImage * image8u3);

    void render_palette(const OpenCvPlus::cvp_Palette* palette,
            IplImage * output_image);
    void render_discrete_palette(const OpenCvPlus::cvp_Palette* palette,
            IplImage * output_image);
    void render_continuous_palette(const OpenCvPlus::cvp_Palette* palette,
            IplImage * output_image);
    void render_color_histogram(const std::vector<double>& histogram,
            unsigned bins_num, IplImage * output_image);
    void render_pim(const OpenCvPlus::cvp_PimModel* pim,
            IplImage * output_image);

    const MainModel_ColourSegmentationTracker& mModel;
    IplImage * mImage8u3;
    IplImage * mImageBuf8u3;
    IplImage * mRenderBuf8u3;

};

#endif // __MAINWINDOW_COLOURSEGMENTATIONTRACKER_H__
