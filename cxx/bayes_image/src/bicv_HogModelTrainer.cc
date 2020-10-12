/**
 * @file cxx/bayes_image/src/bicv_HeadPoseHogModel.cc
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

// SYSTEM INCLUDES
#include <boost/foreach.hpp>                        // foreach loop
#include <boost/filesystem/operations.hpp>          // path operations
#include <boost/progress.hpp>                       // progress display
#include <boost/archive/xml_iarchive.hpp>           // serialization, XML input
#include <boost/archive/xml_oarchive.hpp>           // serialization, XML output
#include <boost/serialization/access.hpp>           // private access for ser.
#include <boost/serialization/vector.hpp>           // ser. for STL vector
#include <vector>                                   // STL vector
#include <list>                                     // STL list
#include <string>                                   // STL string
#include <fstream>                                  // STL file streams
#include <iostream>                                 // STL IO streams
#include <cv.h>                                     // OpenCV main library
#include <highgui.h>                                // OpenCV gui

// PROJECT INCLUDES
#include <image_processing/ip_SingleImageProvider.h>    // single image provider
//#include <image_processing/ip_FlippedImageProcessor.h>  // flipped image provider
#include <image_processing/ip_GrayscaleImageProvider.h> // grayscale image provider
#include <image_processing/ip_GradientImageProcessor.h> // gradient image provider
#include <image_processing/ip_IntegralGradientHistogramProcessor.h>  // integral HoG provider
#include <image_processing/ip_HogFeatureProducerRounded.h>           // HoG feature producer
#include <image_processing/ip_Exceptions.h>             // image proc exceptions

// LOCAL INCLUDES
#include <bayes_image/bicv_HogModelTrainer.h>            // declaration of this
#include <bayes_image/bicv_Exceptions.h>                 // exceptions

using namespace std;
using namespace ImageProcessing;
using namespace OpenCvPlus;
namespace bf=boost::filesystem;

/////////////////////////// LOCAL DECLARATIONS ///////////////////////////////

namespace BICV {

typedef cvp_HeadPoseDiscreteDomain::HeadPose HeadPose;
typedef cvp_HeadPoseDiscreteDomain::real real;

///////////////////////////////// PUBLIC /////////////////////////////////////

//#define DUMP_IMAGES

/* static */ bicv_HeadPoseHogModel *
bicv_HeadPoseHogModel::train(
    const cvp_HeadPoseDiscreteDomain * head_pose_domain,
    const ImageProcessing::ip_HogFeatureParameters& parameters,
    const std::vector<std::string>& example_image_files,
    const std::vector<bicv_TrainingData>& example_annotations) {

    list<float> scales;
    scales.push_back(1.0);
    scales.push_back(0.5);
    scales.push_back(0.25);

    list<ip_ImageProvider*> providers;

    // structures containing trained data
    vector<float> sum_weights(head_pose_domain->size(), 0);
    ip_HistogramTemplate feature_prototype =
            ip_HogFeatureProducerRounded::feature_prototype(parameters);
    vector<ip_HistogramTemplate> mean_hog_templates(
            head_pose_domain->size(), feature_prototype);
    vector<ip_HistogramTemplate> stddev_hog_templates(
            head_pose_domain->size(), feature_prototype);
    vector<HeadPose> mean_hog_template_descriptors(head_pose_domain->values());
    vector<HeadPose> stddev_hog_template_descriptors(head_pose_domain->values());

    // main training loop over examples
    for (unsigned id = 0; id < example_image_files.size(); ++id) {
//        cout << example_image_files[id] << endl;
        const string& data_fname = example_image_files[id];
        const bicv_TrainingData& data_annotation = example_annotations[id];

        IplImage * train_image = cvLoadImage(data_fname.c_str());
        if (!train_image) {
            ostringstream oss;
            oss << "Error loading image \"" << data_fname << "\"!";
            throw bicv_Exception(oss.str());
        }
        const int train_image_width = train_image->width;
        const int train_image_height = train_image->height;
        const CvRect train_face_rect = data_annotation.m_FaceRect;
        const HeadPose head_pose = data_annotation.m_HeadPose;
        const cvp_HeadPoseDiscreteDomain::ElementId head_pose_idx =
            head_pose_domain->id(head_pose_domain->discretize(head_pose));
//        cout << "Head pose: " << head_pose << ", idx=" << head_pose_idx << endl;
        const HeadPose head_pose_mirrored(-head_pose.pan(), head_pose.tilt(), head_pose.roll());
        const cvp_HeadPoseDiscreteDomain::ElementId head_pose_mirrored_idx =
            head_pose_domain->id(head_pose_domain->discretize(head_pose_mirrored));
//        cout << "Head pose: " << head_pose_mirrored << ", idx=" << head_pose_mirrored_idx << endl;
        // TODO: vary weight based on approximation error
        const float weight = 1;

        BOOST_FOREACH(float scale, scales) {
            const int image_width = static_cast<int>(train_image_width * scale);
            const int image_height = static_cast<int>(train_image_height * scale);
            const CvRect face_rect = cvRect(
                static_cast<int>(train_face_rect.x * scale),
                static_cast<int>(train_face_rect.y * scale),
                static_cast<int>(train_face_rect.width * scale),
                static_cast<int>(train_face_rect.height * scale));

            // train on original cropped image
            ip_HogFeatureProducerRounded * hog_feature_producer =
                create_hog_feature_provider_chain(
                    data_fname, image_width, image_height,
                    parameters, providers, false);

            ip_RoiWindow base_window = ip_RoiWindow::from_CvRect(face_rect);
            const int step_local = static_cast<int>(4 * scale);
            for (int drow = 0; drow < step_local; drow += step_local) {
                for (int dcol = 0; dcol < step_local; dcol += step_local) {
//            for (int drow = -step_local; drow <= step_local; drow += step_local) {
//                for (int dcol = -step_local; dcol <= step_local; dcol += step_local) {
                    ip_RoiWindow window(base_window);
                    window.m_iFirstColumn += dcol;
                    window.m_iFirstRow += drow;
                    hog_feature_producer->compute_feature(window);

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
                    IplImage * img_first = providers.front()->image();
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
                    cvReleaseMat(&mat);
                    cvSaveImage((p / oss.str()).c_str(), img_temp);
//                    cout << (p / oss.str()) << " " << window << endl;
                    cvReleaseImage(&img_temp);
                    #endif

                    sum_weights[head_pose_idx] += weight;
                    mean_hog_templates[head_pose_idx] += weight *
                            hog_feature_producer->get_cached_feature();
                    stddev_hog_templates[head_pose_idx] += weight *
                            hog_feature_producer->get_cached_feature() *
                            hog_feature_producer->get_cached_feature();
                }
            }
            delete_hog_feature_provider_chain(providers, hog_feature_producer);

            // train on mirrored image
            ip_HogFeatureProducerRounded * hog_feature_producer_mirrored =
                create_hog_feature_provider_chain(
                    data_fname, image_width, image_height,
                    parameters, providers, true);

            ip_RoiWindow base_window_mirrored = ip_RoiWindow::from_CvRect(face_rect);
            base_window_mirrored.m_iFirstColumn =
                    image_width - base_window_mirrored.m_iFirstColumn -
                    base_window_mirrored.m_iWidth - 1;
            for (int drow = -step_local; drow <=step_local; drow += step_local) {
                for (int dcol = -step_local; dcol <=4; dcol += step_local) {
                    ip_RoiWindow window(base_window_mirrored);
                    window.m_iFirstColumn += dcol;
                    window.m_iFirstRow += drow;
                    hog_feature_producer_mirrored->compute_feature(window);

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
                    cvReleaseMat(&mat);
                    cvSaveImage((p / oss.str()).c_str(), img_temp);
                    cvReleaseImage(&img_temp);
                    #endif

                    sum_weights[head_pose_mirrored_idx] += weight;
                    mean_hog_templates[head_pose_mirrored_idx] += weight *
                            hog_feature_producer_mirrored->get_cached_feature();
                    stddev_hog_templates[head_pose_mirrored_idx] += weight *
                            hog_feature_producer_mirrored->get_cached_feature() *
                            hog_feature_producer_mirrored->get_cached_feature();
                }
            }

            delete_hog_feature_provider_chain(providers, hog_feature_producer_mirrored);

        }
    } // loop ever example images

    for (unsigned head_pose_idx = 0; head_pose_idx < head_pose_domain->size();
            ++head_pose_idx) {
        float weight = sum_weights[head_pose_idx];
        if (weight > 0) {
            mean_hog_templates[head_pose_idx] /= weight;
            stddev_hog_templates[head_pose_idx] /= weight;
            stddev_hog_templates[head_pose_idx] -=
                    mean_hog_templates[head_pose_idx] *
                    mean_hog_templates[head_pose_idx];
        }
    } // loop over head pose domain elements

    return new bicv_HeadPoseHogModel(parameters,
        mean_hog_templates, mean_hog_template_descriptors,
        stddev_hog_templates, stddev_hog_template_descriptors);

} // train

/////////////////////////// LOCAL DEFINITIONS /////////////////////////////////

// SERIALIZATION HELPERS

/// @brief Class to serialize and deserialize HOG features
///
/// This class is responsible for serialization and deserialization of
/// a HOG feature and its descriptor. Head pose is taken as a descriptor
/// in the current implementation.
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    02.03.2011

struct HogFeatureSerializationHelper {

    /// feature index in a vector
    unsigned mIndex;
    /// feature to serialise
    ip_HistogramTemplate mHogFeature;
    /// feature descriptor to serialise
    HeadPose mHogFeatureDescriptor;

    HogFeatureSerializationHelper(unsigned index,
            const ip_HistogramTemplate& hog_feature,
            const HeadPose& hog_feature_descriptor) :
            mIndex(index), mHogFeature(hog_feature),
            mHogFeatureDescriptor(hog_feature_descriptor) {
    }
private:
    // give access to private fields to serialize properly
    friend class boost::serialization::access;

    // hide the serialization functionality

    template<class Archive>
    void save(Archive & ar, const unsigned int ver) const {
        ar & boost::serialization::make_nvp("index", mIndex);
        ar & boost::serialization::make_nvp("head_pose", mHogFeatureDescriptor);
        unsigned template_size = mHogFeature.size();
        ar & boost::serialization::make_nvp("size", template_size);
        for (unsigned i = 0; i < template_size; ++i) {
            ip_HistogramTemplate::Histogram histogram =
                    mHogFeature.get_histogram(i);
            ar & boost::serialization::make_nvp("histogram", histogram);
        }
    }

    template<class Archive>
    void load(Archive & ar, const unsigned int ver) {
        ar & boost::serialization::make_nvp("index", mIndex);
        ar & boost::serialization::make_nvp("head_pose", mHogFeatureDescriptor);
        unsigned template_size;
        ar & boost::serialization::make_nvp("size", template_size);

        if (template_size != mHogFeature.size()) {
            ostringstream oss;
            oss << "Inconsistent template size " << template_size <<
                    " when loading HoG model! Expected " <<
                    mHogFeature.size();
            throw bicv_Exception(oss.str());
        }
        for (unsigned i = 0; i < template_size; ++i) {
            ip_HistogramTemplate::Histogram histogram =
                    mHogFeature.get_histogram(i);
            ar & boost::serialization::make_nvp("histogram", histogram);
            ip_HistogramTemplate::Histogram& result =
                    mHogFeature.get_histogram(i);
            if (result.size() != histogram.size()) {
                ostringstream oss;
                oss << "Inconsistent histogram size " << histogram.size() <<
                        " when loading HoG model! Expected " <<
                        result.size();
                throw bicv_Exception(oss.str());
            }
            result = histogram;
        }
    }

    BOOST_SERIALIZATION_SPLIT_MEMBER()

}; // HogFeatureSerializationHelper

/// @brief Class to serialize and deserialize a vector of HOG features
///
/// This class is responsible for serialization and deserialization of
/// a vector of HOG features and their descriptors. Head poses are taken as
/// descriptors in the current implementation.
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    02.03.2011

struct HogFeatureVectorSerializationHelper {

    /// features to serialise
    vector<ip_HistogramTemplate> mHogFeatures;
    /// feature descriptors to serialise
    vector<HeadPose> mHogFeatureDescriptors;
    /// feature parameters to use when loading serialised features
    ip_HogFeatureParameters mHogFeatureParams;

    HogFeatureVectorSerializationHelper(
        const vector<ip_HistogramTemplate>& hog_features,
        const vector<HeadPose>& hog_feature_descriptors,
        const ip_HogFeatureParameters& params) :
            mHogFeatures(hog_features),
            mHogFeatureDescriptors(hog_feature_descriptors),
            mHogFeatureParams(params) {}
private:
    // give access to private fields to serialize properly
    friend class boost::serialization::access;

    // hide the serialization functionality
    template<class Archive>
    void save(Archive & ar, const unsigned int ver) const {
        unsigned hist_size = mHogFeatures.size();
        ar & boost::serialization::make_nvp("size", hist_size);
        for (unsigned i = 0; i < hist_size; ++i) {
            HogFeatureSerializationHelper helper(i, mHogFeatures[i],
                    mHogFeatureDescriptors[i]);
            ar & boost::serialization::make_nvp("feature", helper);
        }
    }

    template<class Archive>
    void load(Archive & ar, const unsigned int ver) {
        unsigned hist_size;
        ar & boost::serialization::make_nvp("size", hist_size);
        ip_HistogramTemplate htempl = ip_HogFeatureProducerRounded::feature_prototype(
                mHogFeatureParams);
        mHogFeatures = vector<ip_HistogramTemplate>(hist_size, htempl);
        mHogFeatureDescriptors = vector<HeadPose>(hist_size);
        for (unsigned i = 0; i < hist_size; ++i) {
            HogFeatureSerializationHelper helper(i, mHogFeatures[i],
                    mHogFeatureDescriptors[i]);
            ar & boost::serialization::make_nvp("feature", helper);
            mHogFeatures[helper.mIndex] = helper.mHogFeature;
            mHogFeatureDescriptors[helper.mIndex] = helper.mHogFeatureDescriptor;
        }
    }

    BOOST_SERIALIZATION_SPLIT_MEMBER()
}; // HogFeatureVectorSerializationHelper

///////////////////////////////// PUBLIC /////////////////////////////////////

void
bicv_HeadPoseHogModel::save(const std::string& filename) const {
    // prepare the output XML archive to serialize the trained model
    ofstream ofs(filename.c_str());
    if (!ofs) {
        throw bicv_Exception("Cannot open output file " + filename);
    }
    boost::archive::xml_oarchive output_xml_archive(ofs);

    output_xml_archive & boost::serialization::make_nvp("parameters",
            m_HogFeatureParams);

    HogFeatureVectorSerializationHelper helper_means(m_MeanHogTemplates,
            m_MeanHogTemplateDescriptors, m_HogFeatureParams);
    output_xml_archive & boost::serialization::make_nvp("mean_hog_templates",
            helper_means);

    HogFeatureVectorSerializationHelper helper_stddevs(m_StddevHogTemplates,
            m_StddevHogTemplateDescriptors, m_HogFeatureParams);
    output_xml_archive & boost::serialization::make_nvp("stddev_hog_templates",
            helper_stddevs);
} // save

/* static */ bicv_HeadPoseHogModel *
bicv_HeadPoseHogModel::load(const std::string& filename) {
    // prepare the input XML archive to deserialize the model
    ifstream ifs(filename.c_str());
    if (!ifs) {
        throw bicv_Exception("Cannot open input file " + filename);
    }
    boost::archive::xml_iarchive input_xml_archive(ifs);

    ip_HogFeatureParameters hog_feature_parameters;

    // load HOG feature parameters
    try {
        input_xml_archive & boost::serialization::make_nvp("parameters",
                hog_feature_parameters);
    } catch (std::exception& e) {
        ostringstream oss;
        oss << "Error loading parameters of a HOG model from file \""
            << filename << "\": " << e.what();
        throw bicv_Exception(oss.str());
    }

    // load mean HOG features
    vector<ip_HistogramTemplate> hog_templates;
    vector<HeadPose> hog_template_descriptors;

    HogFeatureVectorSerializationHelper helper_means(hog_templates,
            hog_template_descriptors, hog_feature_parameters);
    try {
        input_xml_archive & boost::serialization::make_nvp("mean_hog_templates",
            helper_means);
    } catch (std::exception& e) {
        ostringstream oss;
        oss << "Error loading mean HOG templates from file \""
            << filename << "\": " << e.what();
        throw bicv_Exception(oss.str());
    }

    // load standard deviation HOG features
    HogFeatureVectorSerializationHelper helper_stddevs(hog_templates,
        hog_template_descriptors, hog_feature_parameters);
    try {
        input_xml_archive & boost::serialization::make_nvp("stddev_hog_templates",
                helper_stddevs);
    } catch (std::exception& e) {
        ostringstream oss;
        oss << "Error loading standard deviation HOG templates from file \""
            << filename << "\": " << e.what();
        throw bicv_Exception(oss.str());
    }

    return new bicv_HeadPoseHogModel(hog_feature_parameters,
            helper_means.mHogFeatures,
            helper_means.mHogFeatureDescriptors,
            helper_stddevs.mHogFeatures,
            helper_stddevs.mHogFeatureDescriptors);

} // load

///////////////////////////////// PRIVATE /////////////////////////////////////

bicv_HeadPoseHogModel::bicv_HeadPoseHogModel(
        const ip_HogFeatureParameters& params,
        const vector<ip_HistogramTemplate>& mean_hog_templates,
        const vector<HeadPose>& mean_hog_template_descriptors,
        const vector<ip_HistogramTemplate>& stddev_hog_templates,
        const vector<HeadPose>& stddev_hog_template_descriptors) :
        m_HogFeatureParams(params),
        m_MeanHogTemplates(mean_hog_templates),
        m_MeanHogTemplateDescriptors(mean_hog_template_descriptors),
        m_StddevHogTemplates(stddev_hog_templates),
        m_StddevHogTemplateDescriptors(stddev_hog_template_descriptors) {
} // bicv_HeadPoseHogModel

/* static */ ip_HogFeatureProducerRounded *
bicv_HeadPoseHogModel::create_hog_feature_provider_chain(
    const std::string& filename, int image_width, int image_height,
    const ImageProcessing::ip_HogFeatureParameters& parameters,
    std::list<ip_ImageProvider*>& providers,
    bool is_flipped_x) {

    // data provider
    ip_SingleImageProvider * data_provider =
        new ip_SingleImageProvider(filename, image_width, image_height);
    providers.push_back(data_provider);

    // insert flip processor if necessary
    ip_ImageProvider * input_provider = data_provider;
//    if (is_flipped_x) {
//        ip_FlippedImageProcessor * flipped_provider =
//            new ip_FlippedImageProcessor(data_provider,
//                    ip_FlippedImageProcessor::FLIP_X);
//        providers.push_back(flipped_provider);
//        input_provider = flipped_provider;
//    }

    // data->grayscale processor
    ip_GrayscaleImageProvider * grayscale_provider =
        new ip_GrayscaleImageProvider(input_provider);
    providers.push_back(grayscale_provider);
    // grayscale->gradient processor
    ip_GradientImageProcessor * gradient_provider =
        new ip_GradientImageProcessor(grayscale_provider);
    providers.push_back(gradient_provider);
    // gradient->integral gradient processor
    ip_IntegralGradientHistogramProcessor * integral_HoG_provider =
        new ip_IntegralGradientHistogramProcessor(
            gradient_provider->getGradientAngleProvider(),
            gradient_provider->getGradientMagnitudeProvider(),
            parameters.mNumHistBins);
    providers.push_back(integral_HoG_provider);
    // integral gradient processor->hog feature processor
    ip_HogFeatureProducerRounded * hog_feature_producer =
        new ip_HogFeatureProducerRounded(
            integral_HoG_provider, parameters);
    return hog_feature_producer;

} // create_hog_feature_provider_chain

/* static */ void
bicv_HeadPoseHogModel::delete_hog_feature_provider_chain(
    std::list<ip_ImageProvider*>& providers,
    ip_HogFeatureProducerRounded * hog_feature_producer) {

    // delete all the providers
    BOOST_FOREACH(ip_ImageProvider* provider, providers) {
        delete provider;
    }
    providers.clear();
    // delete HOG feature producer
    delete hog_feature_producer;

} // delete_hog_feature_provider_chain

} // namespace BICV
