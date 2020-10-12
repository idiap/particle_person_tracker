/**
 * @file cxx/bayes_image/bayes_image/bicv_HeadPoseSkinModel.h
 * @date 02 March 2011
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Stores skin colour models for various head orientations
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __BICV_HEADPOSESKINMODEL_H__
#define __BICV_HEADPOSESKINMODEL_H__

// PROJECT INCLUDES
#include <opencvplus/cvplus.h>                       // for real typedef
#include <image_processing/ip_ImageProvider.h>       // for data provider
#include <image_processing/ip_RoiWindow.h>           // ROI declaration
#include <image_processing/ip_SkinFeatureProducer.h> // skin feature producer
#include <opencvplus/FaceColorModel.h>               // face color model

// LOCAL INCLUDES
#include <bayes_image/bicv_TrainingData.h>           // annotation data

namespace BICV {

    /// @brief Class to work with skin colour patterns for different head
    /// orientations
    ///
    /// This class is responsible for loading, storing, saving and training
    /// head skin colour patterns for various head orientations.
    /// Every pattern is represented by a mean and standard deviation of
    /// skin colour maps.
    ///
    /// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
    /// @version 2.0
    /// @date    02.03.2011

class bicv_HeadPoseSkinModel {

    public:

    typedef OpenCvPlus::cvp_HeadPoseDiscreteDomain::HeadPose HeadPose;

    // OPERATIONS

    /// Trains skin model based on annotated data
    /// @param head_pose_domain Head pose domain
    /// @param parameters Parameters of a HOG model to train
    /// @param example_image_files Example image file names
    /// @param example_annotations Annotation data, should be of the same size
    /// as example_image_files and synchronised by indices
    /// @param face_colour_model_config Configuration for face colour model
    /// @return Trained head pose skin model
    static bicv_HeadPoseSkinModel *
    train(const OpenCvPlus::cvp_HeadPoseDiscreteDomain * head_pose_domain,
        const ImageProcessing::ip_SkinFeatureParameters& parameters,
        const std::vector<std::string>& example_image_files,
        const std::vector<bicv_TrainingData>& example_annotations,
        const FaceColorModel::FaceColorModelConfig& face_colour_model_config);

    /// Saves the trained head pose skin model into the file
    /// @param filename Name of the file to save the head pose skin model
    void save(const std::string& filename) const;

    /// Loads the trained head pose skin model from the file
    /// @param filename Name of the file to load the model from
    /// @return Loaded head pose skin model
    static bicv_HeadPoseSkinModel *
    load(const std::string& filename);

    /// Returns trained mean templates
    /// @return Skin template mean values
    const std::vector<ImageProcessing::ip_SkinTemplate>&
    mean_templates() const {
        return m_MeanSkinTemplates;
    }

    /// Returns descriptors for mean templates of the head pose skin model
    /// @return Descriptors for mean templates of the head pose skin model
    const std::vector<HeadPose>&
    mean_template_descriptors() const {
        return m_MeanSkinTemplateDescriptors;
    }


    /// Returns trained stddev templates
    /// @return Skin template standard deviations
    const std::vector<ImageProcessing::ip_SkinTemplate>&
    stddev_templates() const {
        return m_StddevSkinTemplates;
    }

    /// Returns descriptors for stddev templates of the head pose skin model
    /// @return Descriptors for stddev templates of the head pose skin model
    const std::vector<HeadPose>&
    stddev_template_descriptors() const {
        return m_StddevSkinTemplateDescriptors;
    }

    /// Returns training parameters
    /// @return Training parameters
    const ImageProcessing::ip_SkinFeatureParameters& parameters() const {
        return m_Params;
    }

    private:

    // LIFECYCLE

    /// Hidden constructor of head pose skin model
    /// @param params Skin model training parameters
    /// @param mean_skin_templates Learnt mean for head pose skin templates
    /// @param mean_skin_template_descrptors Associated descriptors to mean
    /// head pose skin templates
    /// @param stddev_skin_templates Learnt standard deviation for head pose
    /// skin templates
    /// @param stddev_skin_template_descrptors Associated descriptors to
    /// standard deviation head pose skin templates
    bicv_HeadPoseSkinModel(
        const ImageProcessing::ip_SkinFeatureParameters& params,
        const std::vector<ImageProcessing::ip_SkinTemplate>& mean_skin_templates,
        const std::vector<HeadPose>& mean_skin_template_descriptors,
        const std::vector<ImageProcessing::ip_SkinTemplate>& stddev_skin_templates,
        const std::vector<HeadPose>& stddev_skin_template_descriptors);

    // OPERATIONS

    /// Copies skin templates from one vector into the other
    /// @param from Vector of skin templates to copy data from
    /// @param to Vector of skin templates to copy data to
    static void
    copy_templates(
        const std::vector<ImageProcessing::ip_SkinTemplate>& from,
        std::vector<ImageProcessing::ip_SkinTemplate>& to);

    /// Computes mean skin template based on a vector of templates
    /// @param res_templ Skin templates to store result to
    /// @param tmpls Vector of skin templates on which to compute the mean
    static void
        compute_mean(ImageProcessing::ip_SkinTemplate& res_tmpl,
        const std::vector<ImageProcessing::ip_SkinTemplate>& tmpls);

    // trained mean head pose skin templates
    std::vector<ImageProcessing::ip_SkinTemplate> m_MeanSkinTemplates;
    // descriptors associating head poses to mean skin templates
    std::vector<HeadPose> m_MeanSkinTemplateDescriptors;
    // trained stddev head pose skin templates
    std::vector<ImageProcessing::ip_SkinTemplate> m_StddevSkinTemplates;
    // descriptors associating head poses to stddev skin templates
    std::vector<HeadPose> m_StddevSkinTemplateDescriptors;
    // skin feature parameters
    ImageProcessing::ip_SkinFeatureParameters m_Params;
};

} // namespace BICV

#endif // __BICV_HEADPOSESKINMODEL_H__
