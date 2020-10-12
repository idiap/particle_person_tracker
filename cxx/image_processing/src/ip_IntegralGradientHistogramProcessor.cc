// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_IntegralGradientHistogramProcessor - computes integral gradient histogram
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// SYSTEM INCLUDES
#include <boost/foreach.hpp>                       // foreach loop
#include <iostream>                                // IO
#include <iterator>                                // for vector output
#include <iomanip>                                 // STL IO manipulators
#include <fstream>
#include <iterator>

// PROJECT INCLUDES
#include <opencvplus/cvp_gradient_histogram_features.h>

// LOCAL INCLUDES
#include <image_processing/ip_IntegralGradientHistogramProcessor.h> // declaration of this
#include <image_processing/ip_PartialImageProvider.h>               // for colour components

// minimal integer value of the gradient angle
static const int MIN_INT_ANGLE = 0;
// maximal integer value of the gradient angle
static const int MAX_INT_ANGLE = 314;
// step angle used when computing weight tables
static const int STEP_INT_ANGLE = 1;

using namespace std;

namespace ImageProcessing {

///////////////////////////// LOCAL DEFINITIONS //////////////////////////////

class ip_IntegralImageProvider : public ip_PartialImageProvider {
    public:
    ip_IntegralImageProvider(ip_ImageProvider * master_provider,
            IplImage * buffer_image, const vector<OpenCvPlus::real>& table) :
            ip_PartialImageProvider(master_provider, buffer_image),
            mTable(table) {
    }
    virtual ip_ImageProviderType id() const {
        return IP_IMG_PROVIDER_HISTOGRAM;
    }
    const vector<OpenCvPlus::real>& weight_table() const {
        return mTable;
    }
    vector<OpenCvPlus::real>& weight_table() {
        return mTable;
    }

    void compute_integral_histogram(IplImage* angle_image,
            IplImage* magnitude_image, const ip_RoiWindow& roi) {

        IplImage * img = image_buffer();
        cvSetZero(img);

        float integrateUp;
        float integrateUpLeft;
        float integrateLeft;
        float currentValue;

        for(int row = roi.m_iFirstRow; row < roi.m_iFirstRow + roi.m_iHeight; ++row) {
            integrateLeft = 0;
            for(int column = roi.m_iFirstColumn; column < roi.m_iFirstColumn + roi.m_iWidth; ++column) {
                integrateUp = (row ? CV_IMAGE_ELEM(img, float, row - 1, column) : 0);
                integrateUpLeft = (((row) && (column)) ?
                        CV_IMAGE_ELEM(img, float, row - 1, column - 1) : 0);
                // NOTE: the order of operations here is significant!!!
                // first should perform integrateLeft - integrateUpLeft
                // to verify that the result is nonnegative
                // then add integrateUp, so that the result stays nonnegative
                currentValue = CV_IMAGE_ELEM(magnitude_image, float, row, column) *
                        mTable[static_cast<int>(
                            CV_IMAGE_ELEM(angle_image, float, row, column))] +
                        integrateLeft - integrateUpLeft + integrateUp;
                CV_IMAGE_ELEM(img, float, row, column) = currentValue;
                integrateLeft = currentValue;
            } // Next column
        } // Next row
    }

    private:
    vector<OpenCvPlus::real> mTable;

}; // class ip_IntegralImageProvider

/////////////////////////////// PUBLIC ///////////////////////////////////////

ip_IntegralGradientHistogramProcessor::ip_IntegralGradientHistogramProcessor(
        ip_ImageProvider * gradient_angle_provider,
        ip_ImageProvider * gradient_magnitude_provider,
        int nbins) : mGradientAngleProvider(gradient_angle_provider),
                mGradientMagnitudeProvider(gradient_magnitude_provider),
                mNumBins(nbins) {

    const int width = gradient_angle_provider->image_buffer()->width;
    const int height = gradient_angle_provider->image_buffer()->height;
    const int depth = IPL_DEPTH_32F;
    IplImage * img;

    const float angle_step =
            static_cast<float>(MAX_INT_ANGLE - MIN_INT_ANGLE) / nbins;

    for (int i = 0; i < nbins; ++i) {
        img = cvCreateImage(cvSize(width, height), depth, 1);
        std::vector<OpenCvPlus::real> weight_table =
                initialize_weight_table(MIN_INT_ANGLE, STEP_INT_ANGLE,
                        MAX_INT_ANGLE, round(angle_step / 2 + i * angle_step));
        add_provider(new ip_IntegralImageProvider(this, img, weight_table));
    }

    normalize_weight_tables();

} // ip_IntegralGradientHistogramProcessor

/* virtual */
ip_IntegralGradientHistogramProcessor::~ip_IntegralGradientHistogramProcessor(){

    vector<ip_ImageProvider*>& provider_collection = providers();
    BOOST_FOREACH(ip_ImageProvider* provider, provider_collection) {
        IplImage * image = provider->image_buffer();
        cvReleaseImage(&image);
        delete provider;
    }
    provider_collection.clear();

} // ~ip_IntegralGradientHistogramProcessor

/////////////////////////////// PROTECTED ////////////////////////////////////

/* virtual */ void
ip_IntegralGradientHistogramProcessor::recompute_image(IplImage* image,
        const ip_RoiWindow& roi, boost::posix_time::ptime& time) {
    // obtain the most recent image from the supplier and convert it
    IplImage* source_angle_image = mGradientAngleProvider->image(roi);
    IplImage* source_magnitude_image = mGradientMagnitudeProvider->image(roi);
    time = mGradientAngleProvider->time();

    ip_IntegralImageProvider * histogram_provider;
    vector<ip_ImageProvider*>& provider_collection = providers();
    BOOST_FOREACH(ip_ImageProvider* provider, provider_collection) {
        histogram_provider = dynamic_cast<ip_IntegralImageProvider*>(provider);
        histogram_provider->compute_integral_histogram(source_angle_image,
            source_magnitude_image, roi);
    }

} // recompute_image

/////////////////////////////// PRIVATE //////////////////////////////////////

vector<OpenCvPlus::real>
ip_IntegralGradientHistogramProcessor::initialize_weight_table(int min_angle,
        int step_angle, int max_angle, OpenCvPlus::real mean_angle) {

    // The inverse length scale at which to smooth the bin-angle mapping
    static const OpenCvPlus::real SMOOTHING_BW = 3;

    int num_angles = (max_angle - min_angle) / step_angle + 1;
    vector<OpenCvPlus::real> table(num_angles);
    for (int angle = min_angle, i = 0; angle <= max_angle;
            angle += step_angle, ++i) {
        table[i] = exp(-SMOOTHING_BW * fabs(mean_angle - angle));
    }

    return table;

} // initialize_weight_table

void ip_IntegralGradientHistogramProcessor::normalize_weight_tables() {

    vector<ip_ImageProvider*>& provider_collection = providers();

    if (!provider_collection.empty()) {
        ip_IntegralImageProvider * histogram_provider =
                dynamic_cast<ip_IntegralImageProvider*>(
                        provider_collection.front());
        vector<OpenCvPlus::real> sum_weights(
                histogram_provider->weight_table().size(), 0);

        // compute sum of weights for every angle over the bins
        BOOST_FOREACH(ip_ImageProvider* provider, provider_collection) {
            histogram_provider = dynamic_cast<ip_IntegralImageProvider*>(
                    provider);
            const vector<OpenCvPlus::real>& weight_table =
                    histogram_provider->weight_table();
            transform(sum_weights.begin(), sum_weights.end(),
                    weight_table.begin(), sum_weights.begin(),
                    plus<OpenCvPlus::real>());
        }

        // divide weight by the sum of weights for every angle and all the bins
        BOOST_FOREACH(ip_ImageProvider* provider, provider_collection) {
            histogram_provider = dynamic_cast<ip_IntegralImageProvider*>(
                    provider);
            vector<OpenCvPlus::real>& weight_table =
                    histogram_provider->weight_table();
            transform(weight_table.begin(), weight_table.end(),
                    sum_weights.begin(), weight_table.begin(),
                    divides<OpenCvPlus::real>());
        }

    }

} // normalize_weight_tables

} // namespace ImageProcessing
