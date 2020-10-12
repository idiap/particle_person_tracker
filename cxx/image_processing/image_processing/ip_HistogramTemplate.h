// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_HistogramTemplate - A storage class for a histogram template
//                        (an array of histograms).
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//          Carl Scheffler (Carl.Scheffler@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __IP_HISTOGRAMTEMPLATE_H__
#define __IP_HISTOGRAMTEMPLATE_H__

// SYSTEM INCLUDES
#include <vector>                   // STL vector

// PROJECT INCLUDES
#include <opencvplus/cvplus.h>      // for real typedef

// forward declaration of the class
namespace ImageProcessing {
    class ip_HistogramTemplate;
}

// GLOBAL OPERATORS

/// Output operator.
///
/// @param os Output stream to put the histogram template
/// @param hp Histogram template to stream
/// @return A reference to the output stream.
std::ostream& operator<<(std::ostream& os,
        const ImageProcessing::ip_HistogramTemplate& hist_template);

/// Input operator.
///
/// @param is Input stream to put the histogram template
/// @param hp Histogram template to stream
/// @return A reference to the input stream.
std::istream& operator>>(std::istream& os,
        ImageProcessing::ip_HistogramTemplate& hist_template);


/// Substraction operator.
///
/// @param hp1 Minuend histogram template
/// @param hp2 Subtrahend histogram template
/// @return The elementwise difference of the two histogram templates.
ImageProcessing::ip_HistogramTemplate operator-(
        const ImageProcessing::ip_HistogramTemplate& hp1,
        const ImageProcessing::ip_HistogramTemplate& hp2);

/// Elementwise substraction of constant operator.
///
/// @param ht Minuend histogram template
/// @param v  Subtrahent constant
/// @return The elementwise difference of the histogram template with a constant.
ImageProcessing::ip_HistogramTemplate operator-(
        const ImageProcessing::ip_HistogramTemplate& ht,
        const OpenCvPlus::real& v);

/// Elementwise substraction of constant operator.
///
/// @param v  Minuend constant
/// @param ht Subtrahent histogram template
/// @return The elementwise difference of the constant with the histogram template.
ImageProcessing::ip_HistogramTemplate operator-(
        const OpenCvPlus::real& v,
        const ImageProcessing::ip_HistogramTemplate& ht);

/// Addition operator.
///
/// @param hp1 Augend histogram template
/// @param hp2 Addend histogram template
/// @return The elementwise sum of the two histogram templates.
ImageProcessing::ip_HistogramTemplate operator+(
        const ImageProcessing::ip_HistogramTemplate& hp1,
        const ImageProcessing::ip_HistogramTemplate& hp2);

/// Elementwise addition of constant operator.
///
/// @param ht Augend histogram template
/// @param v  Addend constant
/// @return The elementwise sum of the histogram template with a constant.
ImageProcessing::ip_HistogramTemplate operator+(
        const ImageProcessing::ip_HistogramTemplate& ht,
        const OpenCvPlus::real& v);

/// Elementwise addition of constant operator.
///
/// @param v  Addend constant
/// @param ht Augend histogram template
/// @return The elementwise sum of the histogram template with a constant.
ImageProcessing::ip_HistogramTemplate operator+(
        const OpenCvPlus::real& v,
        const ImageProcessing::ip_HistogramTemplate& ht);

/// Elementwise multiplication operator.
///
/// @param hp1 Multiplicand histogram template
/// @param hp2 Multiplier histogram template
/// @return The elementwise product of the two histogram templates.
ImageProcessing::ip_HistogramTemplate operator*(
        const ImageProcessing::ip_HistogramTemplate& hp1,
        const ImageProcessing::ip_HistogramTemplate& hp2);

/// Elementwise multiplication by a constant operator.
///
/// @param ht Multiplicand histogram template
/// @param v  Multiplier constant
/// @return The elementwise product of the histogram template by a constant.
ImageProcessing::ip_HistogramTemplate operator*(
        const ImageProcessing::ip_HistogramTemplate& ht,
        const OpenCvPlus::real& v);

/// Elementwise multiplication by a constant operator.
///
/// @param v  Multiplier constant
/// @param ht Multiplicand histogram template
/// @return The elementwise product of the histogram template by a constant.
ImageProcessing::ip_HistogramTemplate operator*(
        const OpenCvPlus::real& v,
        const ImageProcessing::ip_HistogramTemplate& ht);

/// Elementwise division operator.
///
/// @param hp1 Dividend histogram template
/// @param hp2 Divisor histogram template
/// @return The elementwise quotient of the two histogram templates.
ImageProcessing::ip_HistogramTemplate operator/(
        const ImageProcessing::ip_HistogramTemplate& hp1,
        const ImageProcessing::ip_HistogramTemplate& hp2);

/// Elementwise division by a constant operator.
///
/// @param ht Dividend histogram template
/// @param v  Divisor constant
/// @return The elementwise quotient of the histogram template by a constant.
ImageProcessing::ip_HistogramTemplate operator/(
        const ImageProcessing::ip_HistogramTemplate& ht,
        const OpenCvPlus::real& v);

/// Elementwise division operator.
///
/// @param v  Dividend constant
/// @param ht Divisor histogram template
/// @return The elementwise quotient of a constant by the histogram template.
ImageProcessing::ip_HistogramTemplate operator/(
        const OpenCvPlus::real& v,
        const ImageProcessing::ip_HistogramTemplate& ht);

/// Elementwise equality operator.
///
/// @param hp1 Left-hands histogram template to compare
/// @param hp2 Right-hands histogram template to compare
/// @return The elementwise equality of the two histogram templates.
bool operator==(const ImageProcessing::ip_HistogramTemplate& hp1,
        const ImageProcessing::ip_HistogramTemplate& hp2);

namespace ImageProcessing {

/// @brief A storage class for a histogram template (an array of histograms)
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @author  Carl Scheffler (Carl.Scheffler@idiap.ch)
/// @version 2.0
/// @date    23.02.2011

class ip_HistogramTemplate {

    public:

    typedef std::vector<OpenCvPlus::real> Histogram;


    /// Constructor
    /// @param num_histograms Number of histograms in the template
    /// @param num_bins Number of bins in each histogram
    ip_HistogramTemplate(unsigned num_histograms, unsigned num_bins);

//    /// Copy constructor
//    ip_HistogramTemplate(const ip_HistogramTemplate& that) :
//        m_HistogramCollection(that.m_HistogramCollection) {
//        std::vector<ImageProcessing::ip_HistogramTemplate::Histogram>
//            ::iterator ii;
//        std::vector<ImageProcessing::ip_HistogramTemplate::Histogram>
//            ::const_iterator jj;
//
//        for (ii = m_HistogramCollection.begin(),
//            jj = that.m_HistogramCollection.begin();
//            (ii != m_HistogramCollection.end()) &&
//            (jj != that.m_HistogramCollection.end()); ++ii, ++jj) {
//                ii->assign(jj->begin(), jj->end());
//        }
//    }

    /// Destructor
    ~ip_HistogramTemplate();

    /// Obtain a histogram by its index for a read-only access
    /// @param index Index of the histogram
    /// @return Histogram at the given index
    const Histogram& get_histogram(unsigned index) const {
        return m_HistogramCollection[index];
    }

    /// Obtain a histogram by its index for a read/write access
    /// @param index Index of the histogram
    /// @return Histogram at the given index
    Histogram& get_histogram(unsigned index) {
        return m_HistogramCollection[index];
    }

    /// Obtain the number of histograms
    /// @return Number of histograms in the template
    unsigned size() const {
        return m_HistogramCollection.size();
    }

    /// Reset all values in all histograms to 0
    void clear();

    // OPERATORS

    /// Assignment operator.
    ///
    /// @param rhs Histogram template to assign
    /// @return Reference to self with modified contents.
    ip_HistogramTemplate& operator=(const ip_HistogramTemplate& rhs);

    /// Addition assignment operator.
    ///
    /// @param rhs Histogram template to add
    /// @return Reference to self with modified contents.
    ip_HistogramTemplate& operator+=(const ip_HistogramTemplate& rhs);

    /// Constant addition assignment operator.
    ///
    /// @param rhs Constant to add
    /// @return Reference to self with modified contents.
    ip_HistogramTemplate& operator+=(const OpenCvPlus::real& rhs);

    /// Substraction assignment operator.
    ///
    /// @param rhs Histogram template to substract
    /// @return Reference to self with modified contents.
    ip_HistogramTemplate& operator-=(const ip_HistogramTemplate& rhs);

    /// Constant substraction assignment operator.
    ///
    /// @param rhs Constant to substract
    /// @return Reference to self with modified contents.
    ip_HistogramTemplate& operator-=(const OpenCvPlus::real& rhs);

    /// Multiplication assignment operator.
    ///
    /// @param rhs Histogram template to multiply with
    /// @return Reference to self with modified contents.
    ip_HistogramTemplate& operator*=(const ip_HistogramTemplate& rhs);

    /// Constant multiplication assignment operator.
    ///
    /// @param rhs Constant to multiply with
    /// @return Reference to self with modified contents.
    ip_HistogramTemplate& operator*=(const OpenCvPlus::real& rhs);

    /// Division assignment operator.
    ///
    /// @param rhs Head pose structure to divide with
    /// @return Reference to self with modified contents.
    ip_HistogramTemplate& operator/=(const ip_HistogramTemplate& rhs);

    /// Constant division assignment operator.
    ///
    /// @param rhs Constant to divide with
    /// @return Reference to self with modified contents.
    ip_HistogramTemplate& operator/=(const OpenCvPlus::real& rhs);

    private:

    // vector of histograms, all elements (vectors) are of the same size
    // that corresponds to the number of bins
    std::vector<Histogram> m_HistogramCollection;

    // FRIEND DECLARATIONS

    friend std::ostream& (::operator<<)(std::ostream& os,
            const ip_HistogramTemplate& hist_template);
    friend std::istream& (::operator>>)(std::istream& os,
            ip_HistogramTemplate& hist_template);
    friend ip_HistogramTemplate (::operator-)(const ip_HistogramTemplate& ht1,
            const ip_HistogramTemplate& ht2);
    friend ip_HistogramTemplate (::operator-)(const ip_HistogramTemplate& ht,
            const OpenCvPlus::real& v);
    friend ip_HistogramTemplate (::operator-)(const OpenCvPlus::real& v,
            const ip_HistogramTemplate& ht);
    friend ip_HistogramTemplate (::operator+)(const ip_HistogramTemplate& ht1,
            const ip_HistogramTemplate& ht2);
    friend ip_HistogramTemplate (::operator+)(const ip_HistogramTemplate& ht,
            const OpenCvPlus::real& v);
    friend ip_HistogramTemplate (::operator+)(const OpenCvPlus::real& v,
            const ip_HistogramTemplate& ht);
    friend ip_HistogramTemplate (::operator*)(const ip_HistogramTemplate& ht1,
            const ip_HistogramTemplate& ht2);
    friend ip_HistogramTemplate (::operator*)(const ip_HistogramTemplate& ht,
            const OpenCvPlus::real& v);
    friend ip_HistogramTemplate (::operator*)(const OpenCvPlus::real& v,
            const ip_HistogramTemplate& ht);
    friend ip_HistogramTemplate (::operator/)(const ip_HistogramTemplate& ht1,
            const ip_HistogramTemplate& ht2);
    friend ip_HistogramTemplate (::operator/)(const ip_HistogramTemplate& ht,
            const OpenCvPlus::real& v);
    friend ip_HistogramTemplate (::operator/)(const OpenCvPlus::real& v,
            const ip_HistogramTemplate& ht);

    friend bool (::operator==)(const ip_HistogramTemplate& ht1,
            const ip_HistogramTemplate& ht2);

    friend OpenCvPlus::real distance_L1(const ip_HistogramTemplate& hog1,
            const ip_HistogramTemplate& hog2, OpenCvPlus::real threshold);
    friend OpenCvPlus::real distance_L2_squared(
            const ip_HistogramTemplate& hog1,
            const ip_HistogramTemplate& hog2, OpenCvPlus::real threshold);
    friend OpenCvPlus::real distance_Mahalanobis_squared(
            const ip_HistogramTemplate& hog1, const ip_HistogramTemplate& hog2,
            const ip_HistogramTemplate& stddev, OpenCvPlus::real threshold);
    friend OpenCvPlus::real distance_Chi2(const ip_HistogramTemplate& hog1,
            const ip_HistogramTemplate& hog2, OpenCvPlus::real threshold);
};

/// L1 distance between two HOG features.
/// Equals to sum of absolute values of elementwise differences of the two
/// features.
/// @param hog1 First HOG feature
/// @param hog2 Second HOG feature
/// @param threshold Theshold for individual histogram L1 distances.
/// If the threshold value is negative (by default), it is not applied;
/// if it is positive, it is applied to each computed distance between
/// histograms
/// @return Measured L1 distance between the two features
OpenCvPlus::real distance_L1(const ip_HistogramTemplate& hog1,
        const ip_HistogramTemplate& hog2,
        OpenCvPlus::real threshold = -1);

/// Squared L2 distance between two HOG features.
/// Each HOG feature is considered as a concatenation of histograms.
/// The distance is computed on a feature as a whole, i.e.
/// distance is a sum over all feature cells AND all histograms of
/// squared differences of the corresponding histogram values.
/// @param hog1 First HOG feature
/// @param hog2 Second HOG feature
/// @param threshold Theshold for individual histogram L2 distances.
/// If the threshold value is negative (by default), it is not applied;
/// if it is positive, it is applied to each computed squared distance between
/// histograms
/// @return Measured squared L2 distance between the two features
OpenCvPlus::real distance_L2_squared(const ip_HistogramTemplate& hog1,
        const ip_HistogramTemplate& hog2,
        OpenCvPlus::real threshold = -1);

/// Squared Mahalanobis distance between two HOG features given the
/// corresponding standard deviations.
/// Equals to sum of squared elementwise differences of the two features,
/// divided by the corresponding variance (square of standard deviation) values.
/// @param hog1 First HOG feature
/// @param hog2 Second HOG feature
/// @param stddevs Standard deviation values
/// @param threshold Theshold for individual histogram Mahalanobis distances.
/// If the threshold value is negative (by default), it is not applied;
/// if it is positive, it is applied to each computed squared distance between
/// histograms
/// @return Measured squared Mahalanobis distance between the two features
OpenCvPlus::real distance_Mahalanobis_squared(const ip_HistogramTemplate& hog1,
        const ip_HistogramTemplate& hog2, const ip_HistogramTemplate& stddev,
        OpenCvPlus::real threshold = -1);

/// Chi-square distance between two HOG features.
/// Equals to sum of squared elementwise differences of the two features,
/// divided by their sums.
/// @param hog1 First HOG feature
/// @param hog2 Second HOG feature
/// @param threshold Theshold for individual histogram Chi-square distances.
/// If the threshold value is negative (by default), it is not applied;
/// if it is positive, it is applied to each computed distance between
/// histograms
/// @return Measured Chi-square distance between the two features
OpenCvPlus::real distance_Chi2(const ip_HistogramTemplate& hog1,
        const ip_HistogramTemplate& hog2,
        OpenCvPlus::real threshold = -1);

} // namespace ImageProcessing

#endif // __IP_HISTOGRAMTEMPLATE_H__
