/**
 * @file cxx/bayes_image/bayes_image/bicv_HeadPoseHogModel.h
 * @date 02 March 2011
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Stores HOG models for various head orientations
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __BICV_HEADPOSEHOGMODEL_H__
#define __BICV_HEADPOSEHOGMODEL_H__

// SYSTEM INCLUDES
#include <list>                                      // STL list

// PROJECT INCLUDES
#include <opencvplus/cvplus.h>                       // for real typedef
#include <opencvplus/cvp_HeadPoseDiscreteDomain.h>   // head pose domain
#include <image_processing/ip_ImageProvider.h>       // for data provider
#include <image_processing/ip_RoiWindow.h>           // ROI declaration
#include <image_processing/ip_HogFeatureParameters.h>  // HOG feature params
#include <image_processing/ip_HogFeatureProducerRounded.h> // HOG feature producer
#include <image_processing/ip_HistogramTemplate.h>   // HOG feature

// LOCAL INCLUDES
#include <bayes_image/bicv_TrainingData.h>           // annotation data

namespace BICV {

/// @brief Class to work with HOG models for different head orientations
///
/// This class is responsible for loading, storing, saving and training
/// head HOG model for various head orientations. Every HOG model is
/// represented by a mean and standard deviation of multiscale HOG features.
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    02.03.2011

class bicv_HeadPoseHogModel {

    public:

    typedef OpenCvPlus::cvp_HeadPoseDiscreteDomain::HeadPose HeadPose;

    // OPERATIONS

    /// Trains HOG model based on annotated data
    /// @param head_pose_domain Head pose domain
    /// @param parameters Parameters of a HOG model to train
    /// @param example_image_files Example image file names
    /// @param example_data Annotation data -- should be of the same size as
    /// example_image_files and synchronised by indices
    static bicv_HeadPoseHogModel *
    train(
        const OpenCvPlus::cvp_HeadPoseDiscreteDomain * head_pose_domain,
        const ImageProcessing::ip_HogFeatureParameters& parameters,
        const std::vector<std::string>& example_image_files,
        const std::vector<bicv_TrainingData>& example_annotations);

    /// Saves HOG model into the provided file
    /// @param filename Name of the file to save the HOG model
    void save(const std::string& filename) const;

    /// Loads trained head pose HOG model from a file
    /// @param filename Name of a file to load the model from
    /// @return Loaded head pose HOG model
    static bicv_HeadPoseHogModel *
    load(const std::string& filename);

    /// Returns head pose HOG model mean templates
    /// @return Head pose HOG template mean values
    const std::vector<ImageProcessing::ip_HistogramTemplate>&
    mean_templates() const {
        return m_MeanHogTemplates;
    }

    /// Returns descriptors for mean templates of the head pose HOG model
    /// @return Descriptors for mean templates of the head pose HOG model
    const std::vector<HeadPose>&
    mean_template_descriptors() const {
        return m_MeanHogTemplateDescriptors;
    }

    /// Returns head pose HOG model standard deviation templates
    /// @return Head pose HOG model standard deviation templates
    const std::vector<ImageProcessing::ip_HistogramTemplate>&
    stddev_templates() const {
        return m_StddevHogTemplates;
    }

    /// Returns descriptors for stddev templates of the head pose HOG model
    /// @return Descriptors for stddev templates of the head pose HOG model
    const std::vector<HeadPose>&
    stddev_template_descriptors() const {
        return m_StddevHogTemplateDescriptors;
    }

    /// Returns HOG model feature parameters
    /// @return HOG model feature parameters
    const ImageProcessing::ip_HogFeatureParameters& parameters() const {
        return m_HogFeatureParams;
    }

    private:

    // LIFECYCLE

    /// Constructor
    /// @param head_pose_domain Head pose discrete domain
    /// @param params HOG feature parameters
    bicv_HeadPoseHogModel(
        const ImageProcessing::ip_HogFeatureParameters& params,
        const std::vector<ImageProcessing::ip_HistogramTemplate>& mean_hog_templates,
        const std::vector<HeadPose>& mean_hog_template_descriptors,
        const std::vector<ImageProcessing::ip_HistogramTemplate>& stddev_hog_templates,
        const std::vector<HeadPose>& stddev_hog_template_descriptors
        );

    /// Creates a number of providers required to compute HOG features
    /// @param filename Name of the source image file
    /// @param image_width Preferred image width
    /// @param image_height Preferred image height
    /// @param parameters HOG feature parameters
    /// @param provider_list List to store constructed providers chain
    /// @param is_flipped_x Flag to indicate whether to flip the input image
    /// around X axis
    /// @return Pointer to HOG feature provider
    static ImageProcessing::ip_HogFeatureProducerRounded *
    create_hog_feature_provider_chain(
        const std::string& filename, int image_width, int image_height,
        const ImageProcessing::ip_HogFeatureParameters& parameters,
        std::list<ImageProcessing::ip_ImageProvider*>& provider_list,
        bool is_flipped_x = false);

    /// Deletes providers chain that computes HOG features
    /// @param provider_list List containing constructed providers chain
    /// @param hog_feature_producer Pointer to the constructed HOG feature
    static void
    delete_hog_feature_provider_chain(
        std::list<ImageProcessing::ip_ImageProvider*>& provider_list,
        ImageProcessing::ip_HogFeatureProducerRounded * hog_feature_producer);

    // HOG feature parameters
    ImageProcessing::ip_HogFeatureParameters m_HogFeatureParams;
    // mean ip_HistogramTemplates
    std::vector<ImageProcessing::ip_HistogramTemplate> m_MeanHogTemplates;
    // descriptors to keep head pose value explicit
    std::vector<HeadPose> m_MeanHogTemplateDescriptors;

    // stddev ip_HistogramTemplates
    std::vector<ImageProcessing::ip_HistogramTemplate> m_StddevHogTemplates;
    // descriptors to keep head pose value explicit
    std::vector<HeadPose> m_StddevHogTemplateDescriptors;

};

} // namespace BICV

#endif // __BICV_HEADPOSEHOGMODEL_H__
