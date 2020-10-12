/**
 * @file cxx/opencvplus/src/cvp_Palette.cc
 * @date 03 February 2011
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Colour palette data representation
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

// SYSTEM INCLUDES
#include <iostream>
#include <iterator>                                      // for back_inserter
#include <sstream>                                       // for string streams
#include <fstream>                                       // for palette load
#include <numeric>                                       // for accumulate
#include <limits>                                        // for numeric_limits
#include <cmath>

#include <boost/math/distributions/students_t.hpp>       // for Student t
#include <boost/math/special_functions/digamma.hpp>
#include <boost/math/policies/policy.hpp>
#include <boost/static_assert.hpp>                       // for 32-bit int

// PROJECT INCLUDES
#include <utils/ut_digamma.h>                            // for digamma

// LOCAL INCLUDES
#include <opencvplus/cvp_Exceptions.h>                   // exceptions
#include <opencvplus/cvp_Palette.h>                      // declaration of this

using namespace TTrackUtils;
using namespace std;

//////////////////////////// LOCAL DECLARATIONS //////////////////////////////

// constant used for continuous palette precomputations: -1.5*log(2*pi)
static const double CONTINUOUS_PALETTE_PRECOMPUTE_CONSTANT = -2.756815599614018;
// log |A| of transformation matrix A used in BGR to YPbPr convertion
static const double LOG_DETERMINANT_BGR2YBR = -1.4427388438401436;


namespace OpenCvPlus {

#define NUM_BYTES_INT 4
#define NUM_BYTES_DOUBLE 8

// define 32bit integer and 64bit float types depending on target architecture
typedef int int_32;
typedef double double_64;

// check what is the size of int, 32-bit is used to read binary data from file
BOOST_STATIC_ASSERT(sizeof(int_32) == NUM_BYTES_INT);
BOOST_STATIC_ASSERT(sizeof(double_64) == NUM_BYTES_DOUBLE);

// policy for digamma function computation
using namespace boost::math::policies;

typedef boost::math::policies::policy<
    boost::math::policies::domain_error<errno_on_error>,
    boost::math::policies::pole_error<errno_on_error>,
    boost::math::policies::overflow_error<errno_on_error>,
    boost::math::policies::evaluation_error<errno_on_error>
> digamma_policy_t;

static digamma_policy_t digamma_policy;

/////////////////////////////// PUBLIC ///////////////////////////////////////

cvp_Palette::cvp_Palette(const cvp_Palette& other) :
        m_PaletteType(other.m_PaletteType),
        m_PaletteData(other.m_PaletteData) {
    if (m_PaletteType == FCM_DISCRETE) {
        DiscreteColorPrecompute * precompute_data =
                (DiscreteColorPrecompute*)other.m_PrecomputeData;
        m_PrecomputeData = new DiscreteColorPrecompute(*precompute_data);
    } else if (m_PaletteType == FCM_CONTINUOUS) {
        ContinuousColorPrecompute * precompute_data =
                (ContinuousColorPrecompute*)other.m_PrecomputeData;
        m_PrecomputeData = new ContinuousColorPrecompute(*precompute_data);
    } else {
        ostringstream oss;
        oss << "Invalid palette type ";
        oss << m_PaletteType;
        oss << " encountered when copy-constructing palette!";
        throw cvp_Exception(oss.str());
    }
} // cvp_Palette

cvp_Palette::~cvp_Palette() {
    switch (m_PaletteType) {
    case FCM_DISCRETE:
        delete (DiscreteColorPrecompute*) m_PrecomputeData;
        break;
    case FCM_CONTINUOUS:
        delete (ContinuousColorPrecompute*) m_PrecomputeData;
        break;
    }
} // ~cvp_Palette

/* static */ cvp_Palette*
cvp_Palette::load_from_file(const std::string& filename) {

    char buffer_int[NUM_BYTES_INT];
    cvp_Palette * palette = 0;

    ifstream colour_model_file;
    colour_model_file.open(filename.c_str(), ios::in | ios::binary);
    if (colour_model_file.is_open()) {

        try {

            // check palette type
            Type palette_type;
            int_32 palette_type_32bit;
            colour_model_file.read(buffer_int, NUM_BYTES_INT);
            palette_type_32bit = *(int_32*) buffer_int;

            ostringstream oss;
            switch (palette_type_32bit) {
            case 0:
                palette_type = FCM_DISCRETE;
                break;
            case 1:
                palette_type = FCM_CONTINUOUS;
                break;
            default:
                oss << "Unrecognized palette type " << palette_type_32bit
                    << " encountered in " << filename;
                throw cvp_Exception(oss.str());
            }

            // use an appropriate method to load the palette
            switch(palette_type) {
            case FCM_DISCRETE:
                palette = load_discrete_palette(colour_model_file, filename);
                break;
            case FCM_CONTINUOUS:
                palette = load_continuous_palette(colour_model_file, filename);
                break;
            }
            colour_model_file.close();
            return palette;
        } catch(...) {
            colour_model_file.close();
            throw;
        }

    } else {
        throw cvp_Exception("Colour model file " + filename +
            " could not be opened for read!");
    }

} // load_from_file

void cvp_Palette::save_to_file(const std::string& filename) const {
    ofstream colour_model_file;
    colour_model_file.open(filename.c_str(), ios::out | ios::binary);
    if (colour_model_file.is_open()) {
        try {
            ostringstream oss;
            // write palette type
            char buffer_int[NUM_BYTES_INT];
            int_32 palette_type_32bit;
            switch (m_PaletteType) {
            case FCM_DISCRETE:
                palette_type_32bit = 0;
                break;
            case FCM_CONTINUOUS:
                palette_type_32bit = 1;
                break;
            default:
                oss << "Unrecognized palette type " << m_PaletteType
                    << " encountered, cannot save to file " << filename;
                throw cvp_Exception(oss.str());
            }
            *(int_32*)buffer_int = palette_type_32bit;
            colour_model_file.write(buffer_int, NUM_BYTES_INT);

            // use an appropriate method to save the palette
            switch(m_PaletteType) {
            case FCM_DISCRETE:
                save_discrete_palette(colour_model_file, filename);
                break;
            case FCM_CONTINUOUS:
                save_continuous_palette(colour_model_file, filename);
                break;
            }
            colour_model_file.close();
        } catch(...) {
            colour_model_file.close();
            throw;
        }
    } else {
        throw cvp_Exception("Colour model file " + filename +
            " could not be opened for write!");
    }
} // save_to_file

std::ostream& operator<<(std::ostream& out, const cvp_Palette& palette) {
    out << "Palette: ";
    ostream_iterator<double> out_iter(out, ", ");
    switch (palette.type()) {
    case cvp_Palette::FCM_CONTINUOUS:
        out << "Continuous, ";
        copy(palette.data().begin(), palette.data().end(), out_iter);
        return out;
    case cvp_Palette::FCM_DISCRETE:
        out << "Discrete";
        return out;
    }
    return out;
} // operator<<

/////////////////////////////// PRIVATE //////////////////////////////////////

/* static */ cvp_Palette*
cvp_Palette::load_discrete_palette(ifstream& instream,
        const string& stream_descriptor) {
    char buffer_int[NUM_BYTES_INT];

    // read number of bins from the binary stream
    instream.read(buffer_int, NUM_BYTES_INT);
    const int bins_num = *(int_32*)buffer_int;
    if (bins_num < 0) {
        ostringstream oss;
        oss << "Negative number of bins (" << bins_num << ") "
            << "provided in discrete palette data \""
            << stream_descriptor << '\"';
        throw cvp_Exception(oss.str());
    }

    // read palette data from the binary stream
    const unsigned bins_num_u = static_cast<unsigned>(bins_num);
    const unsigned dataSize = bins_num_u * bins_num_u * bins_num_u;
    vector<double_64> data(dataSize, 0);
    const unsigned num_chars_to_read = sizeof(double_64) * dataSize;
    instream.read((char*)(&data[0]), num_chars_to_read);
    const unsigned num_chars_read = instream.gcount();
    if (num_chars_read != num_chars_to_read) {
        ostringstream oss;
        oss << "Invalid size when reading discrete palette data, expected "
            << num_chars_to_read << ", actually read " << num_chars_read;
        throw cvp_Exception(oss.str());
    }

    // create palette, set the data and precompute constants
    cvp_Palette * palette = new cvp_Palette(FCM_DISCRETE, dataSize);
    cvp_Palette::Data& palette_data = palette->data();
    copy(&data[0], &data[dataSize], &palette_data[0]);

    palette->precompute_discrete_color();

    return palette;
} // load_discrete_palette

/* static */ cvp_Palette*
cvp_Palette::load_continuous_palette(std::ifstream& instream,
        const std::string& stream_descriptor) {
    // read palette data from the binary stream
    const unsigned dataSize = 12;
    vector<double_64> data(dataSize, 0);
    const unsigned num_chars_to_read = sizeof(double_64) * dataSize;
    instream.read((char*)(&data[0]), num_chars_to_read);
    const unsigned num_chars_read = instream.gcount();
    if (num_chars_read != num_chars_to_read) {
        ostringstream oss;
        oss << "Invalid size when reading continuous palette data, expected "
            << num_chars_to_read << ", actually read " << num_chars_read;
        throw cvp_Exception(oss.str());
    }

    cvp_Palette * palette = new cvp_Palette(FCM_CONTINUOUS, dataSize);
    cvp_Palette::Data& palette_data = palette->data();
    copy(&data[0], &data[dataSize], &palette_data[0]);

    // precompute constants from the palette data
    palette->precompute_continuous_color();
    return palette;
} // load_continuous_palette

void cvp_Palette::save_discrete_palette(std::ofstream& outstream,
        const std::string& stream_descriptor) const {
    char buffer_int[NUM_BYTES_INT];

    DiscreteColorPrecompute * precompute =
            static_cast<DiscreteColorPrecompute*>(m_PrecomputeData);

    // write number of bins to the binary stream
    *(int_32*)buffer_int = precompute->binsNumPerDimension;
    outstream.write(buffer_int, NUM_BYTES_INT);

    // write palette data to the binary stream
    const unsigned dataSize = m_PaletteData.size();
    vector<double_64> data(dataSize, 0);
    copy(&m_PaletteData[0], &m_PaletteData[dataSize], &data[0]);
    const unsigned num_chars_to_write = sizeof(double_64) * dataSize;
    outstream.write((char*)(&data[0]), num_chars_to_write);
} // save_discrete_palette

void cvp_Palette::save_continuous_palette(std::ofstream& outstream,
        const std::string& stream_descriptor) const {
    // write palette data to the binary stream
    const unsigned dataSize = m_PaletteData.size();
    vector<double_64> data(dataSize, 0);
    copy(&m_PaletteData[0], &m_PaletteData[dataSize], &data[0]);
    const unsigned num_chars_to_write = sizeof(double_64) * dataSize;
    outstream.write((char*)(&data[0]), num_chars_to_write);
} // save_continuous_palette

void cvp_Palette::precompute() {
    switch (m_PaletteType) {
    case FCM_DISCRETE:
        precompute_discrete_color();
        break;
    case FCM_CONTINUOUS:
        precompute_continuous_color();
        break;
    }
} // precompute

cvp_Palette& cvp_Palette::operator=(const cvp_Palette& rhs) {

    if (this != &rhs) {
        // if current precompute data type is different from the assigned one,
        // delete it
        if (m_PaletteType != rhs.m_PaletteType) {
            if (m_PaletteType == FCM_DISCRETE) {
                DiscreteColorPrecompute * my_precompute_data =
                    static_cast<DiscreteColorPrecompute*>(m_PrecomputeData);
                delete my_precompute_data;
                m_PrecomputeData = 0;
            } else if (m_PaletteType == FCM_CONTINUOUS) {
                ContinuousColorPrecompute * my_precompute_data =
                    static_cast<ContinuousColorPrecompute*>(m_PrecomputeData);
                delete my_precompute_data;
                m_PrecomputeData = 0;
            } else {
                ostringstream oss;
                oss << "Invalid palette type " << m_PaletteType << " for LHS ";
                oss << " encountered when assigning palette!";
                throw cvp_Exception(oss.str());
            }
        }

        this->m_PaletteType = rhs.m_PaletteType;
        const Data& rhs_data = rhs.data();
        this->m_PaletteData.assign(rhs_data.begin(), rhs_data.end());

        // it the precompute data is not constructed, create it
        if (rhs.type() == FCM_DISCRETE) {
            if (!m_PrecomputeData) {
                m_PrecomputeData = new DiscreteColorPrecompute(
                    *(static_cast<DiscreteColorPrecompute*>(
                        rhs.precomputed_data())));
            } else {
                *(static_cast<DiscreteColorPrecompute*>(m_PrecomputeData)) =
                    *(static_cast<DiscreteColorPrecompute*>(
                        rhs.precomputed_data()));
            }
        } else if (rhs.type() == FCM_CONTINUOUS) {
            if (!m_PrecomputeData) {
                m_PrecomputeData = new ContinuousColorPrecompute(
                        *(static_cast<ContinuousColorPrecompute*>(
                                rhs.precomputed_data())));
            } else {
                *(static_cast<ContinuousColorPrecompute*>(m_PrecomputeData)) =
                    *(static_cast<ContinuousColorPrecompute*>(
                            rhs.precomputed_data()));
            }
        } else {
            ostringstream oss;
            oss << "Invalid palette type " << m_PaletteType << " for RHS ";
            oss << " encountered when assigning palette!";
            throw cvp_Exception(oss.str());
        }
    }
    return *this;
} // operator=


/////////////////////////////// PRIVATE //////////////////////////////////////

cvp_Palette::cvp_Palette(cvp_Palette::Type palette_type, unsigned palette_size) :
    m_PaletteType(palette_type), m_PaletteData(palette_size) {
    ostringstream oss;
    switch (m_PaletteType) {
    case FCM_DISCRETE:
        m_PrecomputeData = new DiscreteColorPrecompute(palette_size);
        break;
    case FCM_CONTINUOUS:
        m_PrecomputeData = new ContinuousColorPrecompute();
        break;
    default:
        oss << "Invalid palette type " << m_PaletteType;
        oss << " encountered when constructing palette!";
        throw cvp_Exception(oss.str());
    }
} // cvp_Palette

void cvp_Palette::precompute_discrete_color() {

    DiscreteColorPrecompute* precompute =
            (DiscreteColorPrecompute*)m_PrecomputeData;

    const DataType sum_intensities = accumulate(m_PaletteData.begin(),
        m_PaletteData.end(), 0.0);
    DataType digamma_value = boost::math::digamma(sum_intensities);
//    DataType digamma_value = fast_digamma(sum_intensities);
    precompute->logConstantUpdate = digamma_value + precompute->logBinVolume;
    precompute->logConstantMarginalLikelihood =
        // -precompute->logBinVolume - FastLog::instance()->log(sum_intensities);
        -precompute->logBinVolume - std::log(sum_intensities);

    const unsigned histogram_size = m_PaletteData.size();
    DataType * palette_data_ptr = &m_PaletteData[0];
    DataType * digamma_distr_ptr = &precompute->digammaDistribution[0];
    for(unsigned i = 0; i < histogram_size; i++) {
//        digamma_value = boost::math::digamma(*palette_data_ptr++, digamma_policy);
        digamma_value = fast_digamma(*palette_data_ptr++);
        *digamma_distr_ptr++ = ((boost::math::isnan)(digamma_value)) ?
                -std::numeric_limits<DataType>::max() : digamma_value;
//        *digamma_distr_ptr++ = digamma_value;

//        cout <<  "Precomputing digamma distribution " <<
//                     opPrecompute->digammaDistribution[i] << ", " <<
//                     ipDistribution[i] << endl << flush;
    }


//    if (!boost::math::isfinite(exp(opPrecompute->logConstant))) {
//        cout << "Log-constant too large: " << opPrecompute->logConstant
//             << ", constant=" << constant
//             << ", digamma(constant)=" << digamma(constant)
//             << ", logbinvolume=" << mLogBinVolume
//             << endl << flush;
//        assert(boost::math::isfinite(exp(opPrecompute->logConstant)));
//    }
} // precompute_discrete_color

void cvp_Palette::precompute_continuous_color() {

    // FastLog * fast_log = FastLog::instance();

    ContinuousColorPrecompute* precompute =
        (ContinuousColorPrecompute*)m_PrecomputeData;
    using namespace boost::math;

    DataType sumDigammaAlpha = 0;
    DataType sumLogBeta = 0;
    DataType sumInvTau = 0;
    DataType sumLogAlphaOverBeta = 0;

    precompute->logConstantMarginalLikelihood = 0;

    for(int i = 0; i < 3; i++) {
        DataType tau   = m_PaletteData[1+4*i];
        DataType alpha = m_PaletteData[2+4*i];
        DataType beta  = m_PaletteData[3+4*i];

        sumDigammaAlpha += boost::math::digamma(alpha);
        // sumLogBeta += fast_log->log(beta);
        sumLogBeta += std::log(beta);
        sumInvTau += 1.0 / tau;
        // sumLogAlphaOverBeta += fast_log->log(alpha/beta);
        sumLogAlphaOverBeta += std::log(alpha/beta);

        precompute->mu[i] = m_PaletteData[0+4*i];
        precompute->alphaOver2Beta[i] = alpha/beta/2;
//        precompute->studentScale[i] = tau / (2 * beta * (tau + 1));
        // sigma^2
        precompute->studentScale[i] = (tau + 1) * beta / (tau * alpha);
        precompute->studentPower[i] = -alpha - 0.5;

        students_t stud_dist(2 * alpha);
        precompute->logConstantMarginalLikelihood +=
            // fast_log->log(pdf(stud_dist, 0) / sqrt(precompute->studentScale[i]))
            std::log(pdf(stud_dist, 0) / sqrt(precompute->studentScale[i]))
            // - 0.5 * fast_log->log((tau + 1) * beta / (tau * alpha));
//            - fast_log->log(
//                cdf(stud_dist, (1 - precompute->mu[i]) / sqrt(precompute->studentScale[i])) -
//                cdf(stud_dist, -precompute->mu[i] / sqrt(precompute->studentScale[i])))
            // + 0.06
            ;

//      cout << "mu_" << i << " = " << opPrecompute->mu[i] << endl;
//      cout << "c_" << i << " = " << opPrecompute->alphaOver2Beta[i] << endl;

    }

//    precompute->logConstantMarginalLikelihood += LOG_DETERMINANT_BGR2YBR;

    // normalize

    precompute->logConstantUpdate =
          LOG_DETERMINANT_BGR2YBR // + log |A|
        + CONTINUOUS_PALETTE_PRECOMPUTE_CONSTANT // - 1.5*log(2*pi)
        + sumDigammaAlpha / 2
        - sumLogBeta / 2
        - sumInvTau / 2;

//    cout << "c" << " = " << opPrecompute->logConstant << endl;

//    if (!boost::math::isfinite(exp(opPrecompute->logConstant))) {
//        cout << "Log-constant too large: " << opPrecompute->logConstant
//             << ", distribution1 ("
//             << "m=" << ipDistribution[0] << ", "
//             << "t=" << ipDistribution[1] << ", "
//             << "a=" << ipDistribution[2] << ", "
//             << "b=" << ipDistribution[3] << "), "
//             << "distribution2 ("
//             << "m=" << ipDistribution[4] << ", "
//             << "t=" << ipDistribution[5] << ", "
//             << "a=" << ipDistribution[6] << ", "
//             << "b=" << ipDistribution[7] << "), "
//             << "distribution3 ("
//             << "m=" << ipDistribution[8] << ", "
//             << "t=" << ipDistribution[9] << ", "
//             << "a=" << ipDistribution[10] << ", "
//             << "b=" << ipDistribution[11] << "), "
//             << "sum_digamma_alpha=" << sumDigammaAlpha << ", "
//             << "sum_log_beta=" << sumLogBeta << ", "
//             << "sum_inv_tau=" << sumInvTau << ", "
//             << "log_det_transform=" << mLogDetTransformYpbpr
//             << endl << flush;
//        assert(boost::math::isfinite(exp(opPrecompute->logConstant)));
//    }
} // precompute_continuous_color

/////////////////////////////// GLOBAL ///////////////////////////////////////

bool palettes_header_equal(const cvp_Palette& palette1,
        const cvp_Palette& palette2) {
    if (palette1.type() != palette2.type()) {
        return false;
    }

    return (palette1.data() == palette2.data());

} // palettes_header_equal

bool palettes_equal(const cvp_Palette& palette1, const cvp_Palette& palette2) {
    if (!palettes_header_equal(palette1, palette2)) {
        return false;
    }
    switch (palette1.type()) {
    case cvp_Palette::FCM_CONTINUOUS:
        return *(static_cast<ContinuousColorPrecompute*>(
            palette1.precomputed_data())) ==
            *(static_cast<ContinuousColorPrecompute*>(
            palette2.precomputed_data()));
    case cvp_Palette::FCM_DISCRETE:
        return *(static_cast<DiscreteColorPrecompute*>(
            palette1.precomputed_data())) ==
            *(static_cast<DiscreteColorPrecompute*>(
            palette2.precomputed_data()));
    default:
        throw cvp_Exception("Invalid palette type encountered!");
    }
} // palettes_equal

bool operator==(const DiscreteColorPrecompute& dcp1,
        const DiscreteColorPrecompute& dcp2) {
    return (dcp1.binsNumPerDimension == dcp2.binsNumPerDimension) &&
           (dcp1.logBinVolume == dcp2.logBinVolume) &&
           (dcp1.logConstantMarginalLikelihood ==
                   dcp2.logConstantMarginalLikelihood) &&
           (dcp1.logConstantUpdate == dcp2.logConstantUpdate) &&
           (dcp1.digammaDistribution == dcp2.digammaDistribution);
} // operator==

bool operator==(const ContinuousColorPrecompute& dcp1,
        const ContinuousColorPrecompute& dcp2) {
    for (unsigned i = 0; i < 3; ++i) {
        if (dcp1.alphaOver2Beta[i] != dcp2.alphaOver2Beta[i]) {
            return false;
        }
        if (dcp1.mu[i] != dcp2.mu[i]) {
            return false;
        }
        if (dcp1.studentScale[i] != dcp2.studentScale[i]) {
            return false;
        }
        if (dcp1.studentPower[i] != dcp2.studentPower[i]) {
            return false;
        }
    }
    if (dcp1.logConstantUpdate != dcp1.logConstantUpdate) {
        return false;
    }
    if (dcp1.logConstantMarginalLikelihood !=
            dcp1.logConstantMarginalLikelihood) {
        return false;
    }
    return true;
} // operator==

} // namespace OpenCvPlus
