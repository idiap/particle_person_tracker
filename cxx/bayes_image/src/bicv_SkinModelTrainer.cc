/**
 * @file cxx/bayes_image/src/bicv_HeadPoseSkinModel.cc
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

// SYSTEM INCLUDES
#include <boost/foreach.hpp>                        // foreach loop
#include <boost/filesystem/operations.hpp>          // path operations
#include <boost/archive/xml_iarchive.hpp>           // serialization, XML input
#include <boost/archive/xml_oarchive.hpp>           // serialization, XML output
#include <boost/serialization/access.hpp>           // private access for ser.
#include <boost/serialization/valarray.hpp>         // ser. for STL valarray
#include <vector>                                   // STL vector
#include <list>                                     // STL list
#include <string>                                   // STL string
#include <fstream>                                  // STL file streams
#include <iostream>                                 // STL IO streams
#include <utility>                                  // for STL pair
#include <map>                                      // for STL map

// PROJECT INCLUDES
#include <image_processing/ip_SkinColourProcessor.h> // skin colour provider
#include <image_processing/ip_IntegralSkinMaskProcessor.h> // integral provider
#include <image_processing/ip_SkinFeatureProducer.h> // skin feature producer
#include <image_processing/ip_Exceptions.h>          // image proc exceptions
#include <image_processing/ip_SingleImageProvider.h> // single image provider
#include <image_processing/ip_FlippedImageProcessor.h> // flipped image provider
#include <render/FcmColourDistributionVisualiser.h>  // render utils

// LOCAL INCLUDES
#include <bayes_image/bicv_SkinModelTrainer.h>      // declaration of this
#include <bayes_image/bicv_Exceptions.h>            // exceptions

using namespace std;
using namespace ImageProcessing;
using namespace OpenCvPlus;
namespace bf=boost::filesystem;

/////////////////////////// LOCAL DECLARATIONS ///////////////////////////////

namespace BICV {

typedef cvp_HeadPoseDiscreteDomain::HeadPose HeadPose;
typedef cvp_HeadPoseDiscreteDomain::real real;

typedef pair<int, int> TrainingGroupId;
typedef pair<string, bicv_TrainingData> TrainingExample;
typedef struct _TrainingGroup {
    TrainingExample m_Adaptation;
    list<TrainingExample> m_Data;
} TrainingGroup;

typedef map<TrainingGroupId, TrainingGroup> Id2TrainingGroupMap;
typedef OpenCvPlus::cvp_HeadPoseDiscreteDomain::HeadPose HeadPose;
typedef HeadPose::value_type HeadPoseValue;

static void
prepare_training_groups(
        const std::vector<std::string>& example_image_files,
        const std::vector<bicv_TrainingData>& example_annotations,
        Id2TrainingGroupMap& id2group);

static void
train_on_group(const OpenCvPlus::cvp_HeadPoseDiscreteDomain * head_pose_domain,
        const ip_SkinFeatureParameters& skin_feature_parameters,
        const FaceColorModel::FaceColorModelConfig& face_colour_model_config,
        vector<float>& sum_weights,
        vector<ip_SkinTemplate>& mean_skin_templates,
        vector<ip_SkinTemplate>& stddev_skin_templates,
        const TrainingGroup& group);

static void
train_on_example(const TrainingExample& example,
        IplImage * adaptation_image, const CvRect& adaptation_roi,
        const ip_SkinFeatureParameters& skin_feature_parameters,
        const FaceColorModel::FaceColorModelConfig& face_colour_model_config,
        vector<float>& sum_weights,
        vector<ip_SkinTemplate>& mean_skin_templates,
        vector<ip_SkinTemplate>& stddev_skin_templates,
        const OpenCvPlus::cvp_HeadPoseDiscreteDomain * head_pose_domain);

/// Creates a number of providers required to compute skin features
/// @param filename Name of the source image file
/// @param image_width Preferred image width
/// @param image_height Preferred image height
/// @param parameters Skin feature parameters
/// @param provider_list List to store constructed providers chain
/// @param is_flipped_x Flag to indicate whether to flip the input image
/// around X axis
/// @return Pointer to HOG feature provider
static ip_SkinFeatureProducer *
create_skin_feature_provider_chain(
    const std::string& filename, int image_width, int image_height,
    const ImageProcessing::ip_SkinFeatureParameters& parameters,
    std::list<ImageProcessing::ip_ImageProvider*>& provider_list,
    const FaceColorModel::FaceColorModelConfig& face_colour_model_config,
    FaceColorModel::FaceColorModel *& face_colour_model,
    bool is_flipped_x = false);

/// Deletes providers chain that computes skin features
/// @param provider_list List containing constructed providers chain
/// @param hog_feature_producer Pointer to the skin feature producer
static void
delete_skin_feature_provider_chain(
    std::list<ImageProcessing::ip_ImageProvider*>& provider_list,
    ImageProcessing::ip_SkinFeatureProducer * skin_feature_producer,
    FaceColorModel::FaceColorModel * face_colour_model);

///////////////////////////////// PUBLIC /////////////////////////////////////

//#define DUMP_IMAGES

/* static */ bicv_HeadPoseSkinModel *
bicv_HeadPoseSkinModel::train(
        const cvp_HeadPoseDiscreteDomain * head_pose_domain,
        const ip_SkinFeatureParameters& skin_feature_parameters,
        const vector<string>& example_image_files,
        const vector<bicv_TrainingData>& example_annotations,
        const FaceColorModel::FaceColorModelConfig& face_colour_model_config) {

    // structures containing trained data
    vector<float> sum_weights(head_pose_domain->size(), 0);
    ip_SkinTemplate feature_prototype =
            ip_SkinFeatureProducer::feature_prototype(
            skin_feature_parameters);
    vector<ip_SkinTemplate> mean_skin_templates(
            head_pose_domain->size(), feature_prototype);
    vector<ip_SkinTemplate> stddev_skin_templates(
            head_pose_domain->size(), feature_prototype);
    vector<HeadPose> mean_skin_template_descriptors(
            head_pose_domain->values());
    vector<HeadPose> stddev_skin_template_descriptors(
            head_pose_domain->values());

    // prepare groups of data using person and session IDs
    Id2TrainingGroupMap id2group;
    prepare_training_groups(example_image_files, example_annotations, id2group);

    const Id2TrainingGroupMap::const_iterator training_group_begin =
            id2group.begin();
    const Id2TrainingGroupMap::const_iterator training_group_end =
            id2group.end();
    Id2TrainingGroupMap::const_iterator training_group_it;

    // train on every group, adapting colour model to frontal face image each
    // time
    for (training_group_it = training_group_begin;
            training_group_it != training_group_end;
            ++training_group_it) {
        const TrainingGroup& group = training_group_it->second;

        train_on_group(head_pose_domain, skin_feature_parameters,
                face_colour_model_config, sum_weights,
                mean_skin_templates, stddev_skin_templates, group);

    } // loop over training groups

    for (unsigned head_pose_idx = 0; head_pose_idx < head_pose_domain->size();
            ++head_pose_idx) {
        float weight = sum_weights[head_pose_idx];
        if (weight > 0) {
            mean_skin_templates[head_pose_idx] /= weight;
            stddev_skin_templates[head_pose_idx] /= weight;
            stddev_skin_templates[head_pose_idx] -=
                    mean_skin_templates[head_pose_idx] *
                    mean_skin_templates[head_pose_idx];
        }
    } // loop over head pose domain elements

    return new bicv_HeadPoseSkinModel(skin_feature_parameters,
        mean_skin_templates, mean_skin_template_descriptors,
        stddev_skin_templates, stddev_skin_template_descriptors);

} // train

/////////////////////////// LOCAL DEFINITIONS /////////////////////////////////

// SERIALIZATION HELPERS

/// @brief Class to serialize and deserialize skin features
///
/// This class is responsible for serialization and deserialization of skin
/// features and their descriptors. Head pose is taken as a feature descriptor
/// in the current implementation.
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    02.03.2011

struct SkinFeatureSerializationHelper {

    unsigned mIndex;
    HeadPose mHeadPose;
    ip_SkinTemplate mSkinFeature;

    SkinFeatureSerializationHelper(unsigned index,
            const ip_SkinTemplate& skin_feature,
            const HeadPose& head_pose) :
            mIndex(index), mHeadPose(head_pose),
            mSkinFeature(skin_feature) {
    }

private:
    // give access to private fields to serialize properly
    friend class boost::serialization::access;

    // hide the serialization functionality

    template<class Archive>
    void serialize(Archive & ar, const unsigned int ver) {
        ar & boost::serialization::make_nvp("index", mIndex);
        ar & boost::serialization::make_nvp("head_pose", mHeadPose);
        ar & boost::serialization::make_nvp("data", mSkinFeature);
    }

};

/// @brief Class to serialize and deserialize a vector of skin features
///
/// This class is responsible for serialization and deserialization of a vector
/// of skin features and their descriptors. Each skin feature corresponds to a
/// head pose, which is taken as a descriptor in the current implementation.
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    02.03.2011

struct SkinFeatureVectorSerializationHelper {

    /// skin features to serialise
    vector<ip_SkinTemplate> mSkinFeatures;
    /// feature descriptors to serialise
    vector<HeadPose> mSkinFeatureDescriptors;
    /// feature parameters to use when loading serialised features
    ImageProcessing::ip_SkinFeatureParameters mSkinFeatureParams;

    /// Constructor of serialization helper
    ///
    ///@param skin features A vector of skin features to serialize
    ///@param skin_feature_descriptors The corresponding vector of skin feature
    /// descriptors
    ///@param params Skin feature parameters
    SkinFeatureVectorSerializationHelper(
        const vector<ip_SkinTemplate>& skin_features,
        const vector<HeadPose>& skin_feature_descriptors,
        const ip_SkinFeatureParameters& params) :
            mSkinFeatures(skin_features),
            mSkinFeatureDescriptors(skin_feature_descriptors),
            mSkinFeatureParams(params) {}

private:

    // give access to private fields to serialize properly
    friend class boost::serialization::access;

    // hide the serialization functionality

    template<class Archive>
    void save(Archive & ar, const unsigned int ver) const {
        const unsigned skin_templates_size = mSkinFeatures.size();
        ar & boost::serialization::make_nvp("size", skin_templates_size);
        for (unsigned i = 0; i < skin_templates_size; ++i) {
            SkinFeatureSerializationHelper helper(i, mSkinFeatures[i],
                mSkinFeatureDescriptors[i]);
            ar & boost::serialization::make_nvp("feature", helper);
        }
    }

    template<class Archive>
    void load(Archive & ar, const unsigned int ver) {
        unsigned skin_templates_size;
        ar & boost::serialization::make_nvp("size", skin_templates_size);
        ip_SkinTemplate template_feature =
            ip_SkinFeatureProducer::feature_prototype(mSkinFeatureParams);
        // create a vector of empty skin templates
        mSkinFeatures = vector<ip_SkinTemplate>(skin_templates_size);
        mSkinFeatureDescriptors.resize(skin_templates_size);
        for (unsigned i = 0; i < skin_templates_size; ++i) {
            mSkinFeatures[i].resize(template_feature.size());
            SkinFeatureSerializationHelper helper(
                    i, mSkinFeatures[i], mSkinFeatureDescriptors[i]);
            ar & boost::serialization::make_nvp("feature", helper);
            mSkinFeatures[helper.mIndex].resize(helper.mSkinFeature.size());
            mSkinFeatures[helper.mIndex] = helper.mSkinFeature;
            if (mSkinFeatures[helper.mIndex].size() != template_feature.size()) {
                ostringstream oss;
                oss << "Error loading skin model: feature " << helper.mIndex
                    << " expected to have size " <<  template_feature.size()
                    << " but the actual size is "
                    << mSkinFeatures[helper.mIndex].size();
                throw bicv_Exception(oss.str());
            }
            mSkinFeatureDescriptors[helper.mIndex] = helper.mHeadPose;
        }
    }

    BOOST_SERIALIZATION_SPLIT_MEMBER()
};

///////////////////////////////// PUBLIC /////////////////////////////////////

void
bicv_HeadPoseSkinModel::save(const std::string& filename) const {
    // prepare the output XML archive to serialize the trained model
    ofstream ofs(filename.c_str());
    if (!ofs) {
        throw bicv_Exception("Cannot open output file " + filename);
    }
    boost::archive::xml_oarchive output_xml_archive(ofs);

    output_xml_archive & boost::serialization::make_nvp("parameters",
        m_Params);

    SkinFeatureVectorSerializationHelper helper_means(
            m_MeanSkinTemplates, m_MeanSkinTemplateDescriptors, m_Params);
    output_xml_archive & boost::serialization::make_nvp("mean_skin_templates",
            helper_means);

    SkinFeatureVectorSerializationHelper helper_stddevs(
            m_StddevSkinTemplates, m_StddevSkinTemplateDescriptors, m_Params);
    output_xml_archive & boost::serialization::make_nvp("stddev_skin_templates",
            helper_stddevs);
} // save

/* static */ bicv_HeadPoseSkinModel *
bicv_HeadPoseSkinModel::load(const std::string& filename) {
    // prepare the input XML archive to deserialize the model
    ifstream ifs(filename.c_str());
    if (!ifs) {
        throw bicv_Exception("Cannot open input file " + filename);
    }
    boost::archive::xml_iarchive input_xml_archive(ifs);

    ip_SkinFeatureParameters skin_feature_parameters;

    try {
        input_xml_archive & boost::serialization::make_nvp("parameters",
                skin_feature_parameters);
    } catch (std::exception& e) {
        ostringstream oss;
        oss << "Error loading parameters of a skin model from file \""
            << filename << "\": " << e.what();
        throw bicv_Exception(oss.str());
    }

    vector<ip_SkinTemplate> skin_templates;
    vector<HeadPose> skin_template_descriptors;

    // load mean skin features
    SkinFeatureVectorSerializationHelper helper_means(skin_templates,
            skin_template_descriptors, skin_feature_parameters);
    try {
        input_xml_archive & boost::serialization::make_nvp("mean_skin_templates",
            helper_means);
    } catch (std::exception& e) {
        ostringstream oss;
        oss << "Error loading mean skin templates from file \""
            << filename << "\": " << e.what();
        throw bicv_Exception(oss.str());
    }

    // load standard deviation skin features
    SkinFeatureVectorSerializationHelper helper_stddevs(skin_templates,
        skin_template_descriptors, skin_feature_parameters);
    try {
        input_xml_archive & boost::serialization::make_nvp("stddev_skin_templates",
                helper_stddevs);
    } catch (std::exception& e) {
        ostringstream oss;
        oss << "Error loading standard deviation skin templates from file \""
            << filename << "\": " << e.what();
        throw bicv_Exception(oss.str());
    }

    return new bicv_HeadPoseSkinModel(skin_feature_parameters,
            helper_means.mSkinFeatures,
            helper_means.mSkinFeatureDescriptors,
            helper_stddevs.mSkinFeatures,
            helper_stddevs.mSkinFeatureDescriptors);

} // load

///////////////////////////////// PRIVATE /////////////////////////////////////

bicv_HeadPoseSkinModel::bicv_HeadPoseSkinModel(
        const ImageProcessing::ip_SkinFeatureParameters& params,
        const std::vector<ImageProcessing::ip_SkinTemplate>& mean_skin_templates,
        const std::vector<HeadPose>& mean_skin_template_descriptors,
        const std::vector<ImageProcessing::ip_SkinTemplate>& stddev_skin_templates,
        const std::vector<HeadPose>& stddev_skin_template_descriptors) :
        m_MeanSkinTemplateDescriptors(mean_skin_template_descriptors),
        m_StddevSkinTemplateDescriptors(stddev_skin_template_descriptors),
        m_Params(params) {

    copy_templates(mean_skin_templates, m_MeanSkinTemplates);
    copy_templates(stddev_skin_templates, m_StddevSkinTemplates);

} // bicv_HogModelTrainer

/* static */
void bicv_HeadPoseSkinModel::copy_templates(
        const std::vector<ImageProcessing::ip_SkinTemplate>& from,
        std::vector<ImageProcessing::ip_SkinTemplate>& to) {
    unsigned num_templates = from.size();
    to.resize(num_templates);
    if (num_templates > 0) {
        unsigned size_template = from[0].size();
        for (unsigned i = 0; i < num_templates; ++i) {
            to[i].resize(size_template);
            to[i] = from[i];
        }
    }
}

/* static */
void bicv_HeadPoseSkinModel::compute_mean(ip_SkinTemplate& res_tmpl,
        const vector<ip_SkinTemplate>& tmpls) {
    res_tmpl.resize(tmpls[0].size());
    res_tmpl = 0;
    BOOST_FOREACH(const ip_SkinTemplate& tmpl, tmpls) {
        res_tmpl += tmpl;
    }
    res_tmpl /= tmpls.size();
}

/////////////////////////// LOCAL DEFINITIONS /////////////////////////////////

/* static */ void
prepare_training_groups(
        const std::vector<std::string>& example_image_files,
        const std::vector<bicv_TrainingData>& example_annotations,
        Id2TrainingGroupMap& id2group) {

    // fill groups with data
    for (unsigned id = 0; id < example_image_files.size(); ++id) {
        const bicv_TrainingData& annotation = example_annotations[id];
//        const CvRect& face_rect = annotation.m_FaceRect;
//        const HeadPose& head_pose = annotation.m_HeadPose;
        const TrainingGroupId group_id(
                annotation.m_PersonId, annotation.m_SessionId);
        TrainingExample example(example_image_files[id], annotation);

        TrainingGroup& group = id2group[group_id];
        group.m_Data.push_back(example);
    }

    // select adaptation data
    Id2TrainingGroupMap::iterator training_group_it;
    const Id2TrainingGroupMap::iterator training_group_begin = id2group.begin();
    const Id2TrainingGroupMap::iterator training_group_end = id2group.end();
    const HeadPose zero_head_pose;

    // traverse every group
    for (training_group_it = training_group_begin;
            training_group_it != training_group_end;
            ++training_group_it) {
//        const TrainingGroupId& group_id = training_group_it->first;
        TrainingGroup& group = training_group_it->second;
        // traverse all examples in each group
        const list<TrainingExample>::const_iterator training_example_begin =
                group.m_Data.begin();
        const list<TrainingExample>::const_iterator training_example_end =
                group.m_Data.end();
        list<TrainingExample>::const_iterator training_example_it =
                training_example_begin;
        // NOTE: we assume here that training example list is not empty
        assert(training_example_begin != training_example_end);
        TrainingExample min_distance_example = *training_example_it;
        HeadPoseValue min_distance = distance_Linf(
                min_distance_example.second.m_HeadPose, zero_head_pose);
        HeadPoseValue cur_distance;

        // select a value with minimal distance to zero head pose
        for (training_example_it = training_example_begin;
            training_example_it != training_example_end;
            ++training_example_it) {
            cur_distance = distance_Linf(
                    training_example_it->second.m_HeadPose, zero_head_pose);
            if (cur_distance < min_distance) {
                min_distance_example = *training_example_it;
                min_distance = cur_distance;
            }
        }
        // and save it as an adaptation example
        group.m_Adaptation = min_distance_example;
    }
} // prepare_training_groups

/* static */ void
train_on_group(const OpenCvPlus::cvp_HeadPoseDiscreteDomain * head_pose_domain,
        const ip_SkinFeatureParameters& skin_feature_parameters,
        const FaceColorModel::FaceColorModelConfig& face_colour_model_config,
        vector<float>& sum_weights,
        vector<ip_SkinTemplate>& mean_skin_templates,
        vector<ip_SkinTemplate>& stddev_skin_templates,
        const TrainingGroup& group) {

    FaceColorModel::FaceColorModel * face_colour_model =
            new FaceColorModel::FaceColorModel(face_colour_model_config);

    // prepare adaptation information - image and ROI
    TrainingExample example_adaptation = group.m_Adaptation;
    IplImage * adaptation_image = cvLoadImage(example_adaptation.first.c_str());
    if (!adaptation_image) {
        ostringstream oss;
        oss << "Error loading image \"" << example_adaptation.first << "\"!";
        throw bicv_Exception(oss.str());
    }
    // annotation contains head ROI, face colour model requires face ROI
    // convertion from head to face ROI using empirical coefficients
    CvRect adaptation_head_rect = example_adaptation.second.m_FaceRect;
    CvRect adaptation_face_rect = adaptation_head_rect;
    adaptation_face_rect.x += adaptation_face_rect.width *
            (52.0 - 42.0) / 42.0;
    adaptation_face_rect.y += adaptation_face_rect.height *
            (48.0 - 40.0) / 42.0;
    adaptation_face_rect.width *= 25.0 / 42.0;
    adaptation_face_rect.height *= 20.0 / 42.0;
    delete face_colour_model;

    // train on all examples in a group
    const list<TrainingExample>::const_iterator training_example_begin =
            group.m_Data.begin();
    const list<TrainingExample>::const_iterator training_example_end =
            group.m_Data.end();
    list<TrainingExample>::const_iterator training_example_it;
    for (training_example_it = training_example_begin;
            training_example_it != training_example_end;
            ++training_example_it) {
        train_on_example(*training_example_it,
                adaptation_image, adaptation_face_rect,
                skin_feature_parameters, face_colour_model_config,
                sum_weights, mean_skin_templates, stddev_skin_templates,
                head_pose_domain);
    }
} // train_on_group

/* static */ void
train_on_example(const TrainingExample& example,
        IplImage * adaptation_image, const CvRect& adaptation_roi,
        const ip_SkinFeatureParameters& skin_feature_parameters,
        const FaceColorModel::FaceColorModelConfig& face_colour_model_config,
        vector<float>& sum_weights,
        vector<ip_SkinTemplate>& mean_skin_templates,
        vector<ip_SkinTemplate>& stddev_skin_templates,
        const OpenCvPlus::cvp_HeadPoseDiscreteDomain * head_pose_domain) {

    const string& example_fname = example.first;
    const bicv_TrainingData& example_data = example.second;

    // get example image width and height
    IplImage * train_image = cvLoadImage(example_fname.c_str());
    if (!train_image) {
        ostringstream oss;
        oss << "Error loading image \"" << example_fname << "\"!";
        throw bicv_Exception(oss.str());
    }
    const int train_image_width = train_image->width;
    const int train_image_height = train_image->height;

    // prepare list of scales
    list<float> scales;
    scales.push_back(1.0);
    scales.push_back(0.5);
    scales.push_back(0.25);

    // prepare head pose indices
    const float weight = 1;
    const CvRect train_head_rect = example_data.m_FaceRect;
    const HeadPose train_head_pose = example_data.m_HeadPose;
    const cvp_HeadPoseDiscreteDomain::ElementId head_pose_idx =
        head_pose_domain->id(head_pose_domain->discretize(train_head_pose));
    const HeadPose head_pose_mirrored(
        -train_head_pose.pan(), train_head_pose.tilt(), train_head_pose.roll());
    const cvp_HeadPoseDiscreteDomain::ElementId head_pose_mirrored_idx =
        head_pose_domain->id(head_pose_domain->discretize(head_pose_mirrored));

    // create providers cache
    list<ip_ImageProvider*> providers;

    BOOST_FOREACH(float scale, scales) {
        const int image_width = static_cast<int>(train_image_width * scale);
        const int image_height = static_cast<int>(train_image_height * scale);
        const CvRect face_rect = cvRect(
            static_cast<int>(train_head_rect.x * scale),
            static_cast<int>(train_head_rect.y * scale),
            static_cast<int>(train_head_rect.width * scale),
            static_cast<int>(train_head_rect.height * scale));

        FaceColorModel::FaceColorModel * face_colour_model;

        // train on original cropped image
        ip_SkinFeatureProducer * skin_feature_producer =
            create_skin_feature_provider_chain(
                example_fname, image_width, image_height,
                skin_feature_parameters, providers, face_colour_model_config,
                face_colour_model, false);

//        IplImage * img_first = (*(providers.begin()))->image();
//        face_colour_model->cache_probability_maps(adaptation_image);
//        const FaceColorModel::FaceColorModel::PimFeatureType * proba_maps;
//        IplImage * proba_map_image;

//        proba_maps = face_colour_model->pim_feature();
//        proba_map_image = RenderUtils::visualize_pim_feature_classification(
//                *proba_maps);
//        cvSaveImage("model_proba_maps.png", proba_map_image);
//        cvReleaseImage(&proba_map_image);
//        proba_maps = face_colour_model->cached_probability_maps();
//        proba_map_image = RenderUtils::visualize_pim_feature_classification(
//                *proba_maps);
//        cvSaveImage("first_image_proba_maps.png", proba_map_image);
//        cvReleaseImage(&proba_map_image);

        face_colour_model->adapt_to(adaptation_image, adaptation_roi);

//        proba_maps = face_colour_model->pim_feature();
//        proba_map_image = RenderUtils::visualize_pim_feature_classification(
//                *proba_maps);
//        cvSaveImage("model_proba_maps_adapted.png", proba_map_image);
//        cvReleaseImage(&proba_map_image);
//        face_colour_model->cache_probability_maps(adaptation_image);
//        proba_maps = face_colour_model->cached_probability_maps();
//        proba_map_image = RenderUtils::visualize_pim_feature_classification(*proba_maps);
//        cvSaveImage("first_image_proba_maps_adapted.png", proba_map_image);
//        cvReleaseImage(&proba_map_image);
//
//        throw 0;

        ip_RoiWindow base_window = ip_RoiWindow::from_CvRect(face_rect);
        const int step_local = static_cast<int>(4 * scale);
        for (int drow = 0; drow < step_local; drow += step_local) {
            for (int dcol = 0; dcol < step_local; dcol += step_local) {
                ip_RoiWindow window(base_window);
                window.m_iFirstColumn += dcol;
                window.m_iFirstRow += drow;
                skin_feature_producer->compute_feature(window);

                #ifdef DUMP_IMAGES
                ostringstream oss;
                bf::path p("dump_head_pose");
                oss << "pose_" << head_pose_idx;
                p /=  oss.str();
                bf::create_directories(p);
                int counter = 1;
                oss.str("");
                oss << "image_" << counter << ".png";
                while (bf::exists(p / oss.str())) {
                    counter++;
                    oss.str("");
                    oss << "image_" << counter << ".png";
                }
                IplImage * img_first = (*(++providers.begin()))->image();
                IplImage * img_temp = cvCreateImage(
                        cvSize(window.m_iWidth, window.m_iHeight),
                        img_first->depth, img_first->nChannels);
                CvMat * mat = cvCreateMat(2, 3, CV_32FC1);
                cvSet2D(mat, 0, 0, cvScalar(1.0));
                cvSet2D(mat, 0, 1, cvScalar(0.0));
                cvSet2D(mat, 0, 2, cvScalar(window.m_iFirstColumn));
                cvSet2D(mat, 1, 0, cvScalar(0.0));
                cvSet2D(mat, 1, 1, cvScalar(1.0));
                cvSet2D(mat, 1, 2, cvScalar(window.m_iFirstRow));
                cvWarpAffine(img_first, img_temp, mat,
                    CV_INTER_LINEAR | CV_WARP_FILL_OUTLIERS | CV_WARP_INVERSE_MAP);
                cvSaveImage((p / oss.str()).c_str(), img_temp);
                cvReleaseImage(&img_temp);

                IplImage * img_second = (*(++++providers.begin()))->image();
                img_temp = cvCreateImage(
                        cvSize(window.m_iWidth, window.m_iHeight),
                        img_second->depth, img_second->nChannels);
                cvWarpAffine(img_second, img_temp, mat,
                    CV_INTER_LINEAR | CV_WARP_FILL_OUTLIERS | CV_WARP_INVERSE_MAP);
                oss.str("");
                oss << "mask_" << counter << ".png";
                cvSaveImage((p / oss.str()).c_str(), img_temp);
                //                    cout << (p / oss.str()) << " " << window << endl;
                cvReleaseMat(&mat);
                cvReleaseImage(&img_temp);
                #endif

                sum_weights[head_pose_idx] += weight;
                mean_skin_templates[head_pose_idx] += weight *
                        skin_feature_producer->get_cached_feature();
                stddev_skin_templates[head_pose_idx] += weight *
                        skin_feature_producer->get_cached_feature() *
                        skin_feature_producer->get_cached_feature();
            }
        }
        delete_skin_feature_provider_chain(providers,
                skin_feature_producer, face_colour_model);

        // train on mirrored image
        ip_SkinFeatureProducer * skin_feature_producer_mirrored =
            create_skin_feature_provider_chain(
                example_fname, image_width, image_height,
                skin_feature_parameters, providers,  face_colour_model_config,
                face_colour_model, true);

        face_colour_model->adapt_to(adaptation_image, adaptation_roi);

        ip_RoiWindow base_window_mirrored = ip_RoiWindow::from_CvRect(face_rect);
        base_window_mirrored.m_iFirstColumn =
                image_width - base_window_mirrored.m_iFirstColumn -
                base_window_mirrored.m_iWidth - 1;
        for (int drow = -step_local; drow <=step_local; drow += step_local) {
            for (int dcol = -step_local; dcol <=4; dcol += step_local) {
                ip_RoiWindow window(base_window_mirrored);
                window.m_iFirstColumn += dcol;
                window.m_iFirstRow += drow;
                skin_feature_producer_mirrored->compute_feature(window);

                #ifdef DUMP_IMAGES
                ostringstream oss;
                bf::path p("dump_head_pose");
                oss << "pose_" << head_pose_mirrored_idx;
                p /=  oss.str();
                bf::create_directories(p);
                int counter = 1;
                oss.str("");
                oss << "image_mirrored_" << counter << ".png";
                while (bf::exists(p / oss.str())) {
                    counter++;
                    oss.str("");
                    oss << "image_mirrored_" << counter << ".png";
                }
                IplImage * img_first = (*(++++providers.begin()))->image();
                IplImage * img_temp = cvCreateImage(
                        cvSize(window.m_iWidth, window.m_iHeight),
                        img_first->depth, img_first->nChannels);
                CvMat * mat = cvCreateMat(2, 3, CV_32FC1);
                cvSet2D(mat, 0, 0, cvScalar(1.0));
                cvSet2D(mat, 0, 1, cvScalar(0.0));
                cvSet2D(mat, 0, 2, cvScalar(window.m_iFirstColumn));
                cvSet2D(mat, 1, 0, cvScalar(0.0));
                cvSet2D(mat, 1, 1, cvScalar(1.0));
                cvSet2D(mat, 1, 2, cvScalar(window.m_iFirstRow));
                cvWarpAffine(img_first, img_temp, mat,
                    CV_INTER_LINEAR | CV_WARP_FILL_OUTLIERS | CV_WARP_INVERSE_MAP);
                cvSaveImage((p / oss.str()).c_str(), img_temp);
                cvReleaseImage(&img_temp);

                IplImage * img_second = (*(++++++providers.begin()))->image();
                img_temp = cvCreateImage(
                        cvSize(window.m_iWidth, window.m_iHeight),
                        img_second->depth, img_second->nChannels);
                cvWarpAffine(img_second, img_temp, mat,
                    CV_INTER_LINEAR | CV_WARP_FILL_OUTLIERS | CV_WARP_INVERSE_MAP);
                oss.str("");
                oss << "mask_mirrored_" << counter << ".png";
                cvSaveImage((p / oss.str()).c_str(), img_temp);

                cvReleaseMat(&mat);
                cvReleaseImage(&img_temp);
                #endif

                sum_weights[head_pose_mirrored_idx] += weight;
                mean_skin_templates[head_pose_mirrored_idx] += weight *
                        skin_feature_producer_mirrored->get_cached_feature();
                stddev_skin_templates[head_pose_mirrored_idx] += weight *
                        skin_feature_producer_mirrored->get_cached_feature() *
                        skin_feature_producer_mirrored->get_cached_feature();
            }
        }
        delete_skin_feature_provider_chain(providers,
                skin_feature_producer_mirrored, face_colour_model);
    } // foreach scale
} // train_on_example

/* static */ ip_SkinFeatureProducer *
create_skin_feature_provider_chain(
    const string& filename, int image_width, int image_height,
    const ip_SkinFeatureParameters& parameters,
    list<ip_ImageProvider*>& provider_list,
    const FaceColorModel::FaceColorModelConfig& face_colour_model_config,
    FaceColorModel::FaceColorModel *& face_colour_model,
    bool is_flipped_x) {

    // data provider
    ip_SingleImageProvider * data_provider =
        new ip_SingleImageProvider(filename, image_width, image_height);
    provider_list.push_back(data_provider);

    // insert flip processor if necessary
    ip_ImageProvider * input_provider = data_provider;
    if (is_flipped_x) {
        ip_FlippedImageProcessor * flipped_provider =
            new ip_FlippedImageProcessor(data_provider,
                    ip_FlippedImageProcessor::FLIP_X);
        provider_list.push_back(flipped_provider);
        input_provider = flipped_provider;
    }

    // create and insert skin colour processor
    face_colour_model = new FaceColorModel::FaceColorModel(
            face_colour_model_config);
    ip_SkinColourProcessor * skin_mask_provider =
            new ip_SkinColourProcessor(input_provider,
                    face_colour_model, parameters.mSkinMaskValue);
    provider_list.push_back(skin_mask_provider);

    // create and insert integral skin mask processor
    ip_IntegralSkinMaskProcessor * integral_skin_provider =
            new ip_IntegralSkinMaskProcessor(skin_mask_provider);
    provider_list.push_back(integral_skin_provider);


    ip_SkinFeatureProducer * skin_feature_producer = new ip_SkinFeatureProducer(
            integral_skin_provider, parameters);
    return skin_feature_producer;

} // create_skin_feature_provider_chain

/* static */ void
delete_skin_feature_provider_chain(
    list<ip_ImageProvider*>& provider_list,
    ip_SkinFeatureProducer * skin_feature_producer,
    FaceColorModel::FaceColorModel * face_colour_model) {

    // delete all the providers
    BOOST_FOREACH(ip_ImageProvider* provider, provider_list) {
        delete provider;
    }
    provider_list.clear();
    // delete HOG feature producer
    delete skin_feature_producer;
    delete face_colour_model;

} // delete_skin_feature_provider_chain

} // namespace BICV
