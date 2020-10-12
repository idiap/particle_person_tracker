/**
 * @file cxx/opencvplus/opencvplus/cvp_Palette.h
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

#ifndef __CVP_PALETTE_H__
#define __CVP_PALETTE_H__

// SYSTEM INCLUDES
#include <vector>
#include <string>
#include <cmath>

namespace OpenCvPlus {

/// @brief Palette class for different colour models.
///
/// This class is used to represent a palette of a colour model.
/// Depending on the model type (discrete or continuous) it contains either
/// a discrete distribution in a colour space (RGB, YPbPr or others),
/// or a continuous distribution represented by a number of parameters
/// (a quad is used currently that corresponds to parameters of Normal-Gamma
/// distribution).
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    13.09.2012

class cvp_Palette {

    public:

    /// Palette type (discrete / continuous)
    enum Type {
        FCM_DISCRETE = 0,
        FCM_CONTINUOUS = 1
    };

    /// Palette info is a pair consisting of palette type and file name
    typedef std::pair<Type, std::string> Info;
    /// Type of each element in the palette data
    typedef double DataType;
    /// Palette data
    typedef std::vector<DataType> Data;

    // LIFECYCLE

    /// Copy constructor to create a palette based on the existing one.
    /// @param other Palette to construct a copy from
    cvp_Palette(const cvp_Palette& other);

    /// Destructor, releases the allocated palette
    ~cvp_Palette();

    // OPERATIONS

    /// Factory method to load palette from a file.
    /// @param filename Palette model type and file name that contains
    /// @return Constructed palette
    static cvp_Palette* load_from_file(const std::string& filename);

    /// Save palette to a file.
    /// @param filename Name of a file to save palette to
    void save_to_file(const std::string& filename) const;

    /// Returns palette data (read-only access).
    /// @return Palette data
    const Data& data() const {
        return m_PaletteData;
    }

    /// Returns palette data (read-write access).
    /// @return Palette data
    Data& data() {
        return m_PaletteData;
    }

    /// Returns palette type. For discrete palettes returns FCM_DISCRETE,
    /// for continuous returns FCM_CONTINUOUS.
    /// @return Palette type
    Type type() const {
        return m_PaletteType;
    }

    /// Precomputes various distribution constants
    void precompute();

    /// Returns pointer to precomputed data
    /// @return Pointer to precomputed data
    void* precomputed_data() const {
        return m_PrecomputeData;
    }

    // OPERATORS

    /// Assignment operator.
    ///
    /// @param rhs Palette to assign
    /// @return Reference to self with modified contents.
    cvp_Palette& operator=(const cvp_Palette& rhs);

    private:

    /// Constructor to create a palette given its size.
    /// @param palette_type Palette type (FCM_DISCRETE, FCM_CONTINUOUS, etc.)
    /// @param palette_size Palette size
    cvp_Palette(Type palette_type, unsigned palette_size);

    /// Pre-computation for using a discrete color model to compute
    /// channel likelihoods.
    void precompute_discrete_color();

    /// Pre-computation for using a continuous color model to compute
    /// channel likelihoods.
    void precompute_continuous_color();

    /// Loads discrete palette from a stream
    /// @param instream Input stream
    /// @param stream_descriptor Stream descriptor
    /// @return Created discrete palette
    static cvp_Palette* load_discrete_palette(std::ifstream& instream,
            const std::string& stream_descriptor);

    /// Loads continuous palette from a stream
    /// @param instream Input stream
    /// @param stream_descriptor Stream descriptor
    /// @return Created continuous palette
    static cvp_Palette* load_continuous_palette(std::ifstream& instream,
            const std::string& stream_descriptor);

    /// Saves discrete palette into a binary stream
    /// @param outstream Output binary stream
    /// @param stream_descriptor Stream descriptor
    void save_discrete_palette(std::ofstream& outstream,
            const std::string& stream_descriptor) const;

    /// Saves continuous palette into a binary stream
    /// @param outstream Output binary stream
    /// @param stream_descriptor Stream descriptor
    void save_continuous_palette(std::ofstream& outstream,
            const std::string& stream_descriptor) const;

    // palette type (discrete, continuous, histogram)
    Type m_PaletteType;
    // palette data (exact contents depends on palette type)
    Data m_PaletteData;
    // palette precomputation data
    void* m_PrecomputeData;

}; // class cvp_Palette


/// Checks if palette headers (type, data size and data) are equal. Precomputed
/// caches are not verified for equality.
/// @param palette1 First palette to compare
/// @param palette2 Second palette to compare
/// @return True if palettes' headers are equal
bool palettes_header_equal(const cvp_Palette& palette1,
        const cvp_Palette& palette2);

/// Checks if palettes are equal. Both headers (type, data size and data)
/// and precomputed caches are verified for equality.
/// @param palette1 First palette to compare
/// @param palette2 Second palette to compare
/// @return True if palettes are completely identical
bool palettes_equal(const cvp_Palette& palette1, const cvp_Palette& palette2);

/// Output operator for a palette
/// @param out Output stream to write to
/// @param palette Palette to write
/// @return Output stream
std::ostream& operator<<(std::ostream& out, const cvp_Palette& palette);

/// Class that caches precomputed data for discrete palettes. This data is used
/// when evaluating likelihoods and performing colour model adaptation.
class DiscreteColorPrecompute {
public:

    DiscreteColorPrecompute(unsigned histogram_size) :
        logBinVolume (-log(histogram_size)),
        digammaDistribution(histogram_size) {
        binsNumPerDimension = static_cast<unsigned>(round(pow(
                static_cast<double>(histogram_size), 1.0 / 3.0)));
    } // DiscreteColorPrecompute

    ~DiscreteColorPrecompute() {
    } // ~DiscreteColorPrecompute

    unsigned binsNumPerDimension;
    cvp_Palette::DataType logBinVolume;
    std::vector<cvp_Palette::DataType> digammaDistribution;
    cvp_Palette::DataType logConstantUpdate;
    cvp_Palette::DataType logConstantMarginalLikelihood;
};

/// Compares two cached precomputed data for discrete distributions
/// @param dcp1 First precomputed data to compare
/// @param dcp2 Second precomputed data to compare
/// @return True if data are equal
bool operator==(const DiscreteColorPrecompute& dcp1,
        const DiscreteColorPrecompute& dcp2);

/// Class that caches precomputed data for continuous palettes. This data
/// is used when evaluating likelihoods and performing colour model adaptation.
class ContinuousColorPrecompute {
public:
    cvp_Palette::DataType alphaOver2Beta [3];
    cvp_Palette::DataType mu [3];
    cvp_Palette::DataType studentScale [3];
    cvp_Palette::DataType studentPower [3];
    cvp_Palette::DataType logConstantUpdate;
    cvp_Palette::DataType logConstantMarginalLikelihood;
};

/// Compares two cached precomputed data for continuous distributions
/// @param dcp1 First precomputed data to compare
/// @param dcp2 Second precomputed data to compare
/// @return True if data are equal
bool operator==(const ContinuousColorPrecompute& dcp1,
        const ContinuousColorPrecompute& dcp2);

} // namespace OpenCvPlus

#endif // __CVP_PALETTE_H__
