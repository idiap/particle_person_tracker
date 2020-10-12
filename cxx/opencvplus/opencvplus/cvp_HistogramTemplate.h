/**
 * cvp_HistogramTemplate class: A storage class for a histogram
 * template (an array of histograms).
 *
 * A histogram template is simply a 1d array of histograms. The
 * meaning of the index in the array depends on the usage. Each
 * histogram in the array can be multi-dimensional. This is a storage
 * class in that the computation of the histograms is left up to the
 * user.
 *
 * @author Jean-Marc Odobez (Jean-Marc.Odobez@idiap.ch)
 * @author Daniel Gatica-Perez (gatica@idiap.ch)
 * @author Carl Scheffler (Carl.Scheffler@idiap.ch)
 *
 * See COPYING file for the complete license text.
 */

#ifndef __CVP_HISTOGRAMTEMPLATE_H__
#define __CVP_HISTOGRAMTEMPLATE_H__

#include "cvplus.h"

#include <string>
#include <iostream>

namespace OpenCvPlus {

  class cvp_HistogramTemplate {

  // Attributes
  private:

    /// Array of histograms. Implemented using the OpenCV Histogram type.
    CvHistogram** mpHistograms;

    /// Number of histograms in the template.
    int mNumberOfHistograms;

    /// Number of dimensions in each histogram.
    int mNumberOfDimensions;


  // Lifecycle methods
  public:

    /// Default constructor.
    cvp_HistogramTemplate();

    /// Constructor, creating histograms with arbitrary bin counts and
    /// bin spacing along each dimension.
    /// @param iNumberOfHistograms Number of histograms in the template.
    /// @param iNumberOfDimensions Number of dimensions in each histogram.
    /// @param ipBinsPerDimension Number of bins along each dimension.
    /// @param ipRanges Array of ranges for histogram bins. The exact
    ///        meaning depends on iIsUniform. For details, see the
    ///        OpenCV documentation for cvCreateHist().
    /// @param iIsUniform Flag indicating whether bins are spaced
    ///        uniformly or not.
    cvp_HistogramTemplate(int iNumberOfHistograms, int iNumberOfDimensions,
			  int* ipBinsPerDimension, real** ipRanges, bool iIsUniform = false);

    /// Constructor, creating histograms with equal number of bins
    /// along each dimension and where bins are uniformly spaced.
    /// @param iNumberOfHistograms Number of histograms in the template.
    /// @param iNumberOfDimensions Number of dimensions in each histogram.
    /// @param iNumberOfBins Number of bins along each dimension.
    /// @param iMinRange The left edge of the first bin.
    /// @param iMaxRange The right edge of the last bin.
    cvp_HistogramTemplate(int iNumberOfHistograms, int iNumberOfDimensions,
			  int iNumberOfBins, real iMinRange, real iMaxRange);

    /// Destructor.
    virtual ~cvp_HistogramTemplate();


  // Lifecycle support methods
  private:

    /// Allocate memory for histogram template.
    /// @param iNumberOfHistograms Number of histograms in the template.
    /// @param iNumberOfDimensions Number of dimensions in each histogram.
    /// @param ipBinsPerDimension Number of bins along each dimension.
    /// @param ipRanges Array of ranges for histogram bins. The exact
    ///        meaning depends on iIsUniform. For details, see the
    ///        OpenCV documentation for cvCreateHist().
    /// @param iIsUniform Flag indicating whether bins are spaced
    ///        uniformly or not.
    virtual void create_histogram(int iNumberOfHistograms, int iNumberOfDimensions,
				  int* ipBinsPerDimension, real** ipRanges,
				  bool iIsUniform = false);

    /// Allocate memory for histogram template, assuming uniform
    /// spacing of bins.
    /// @param iNumberOfHistograms Number of histograms in the template.
    /// @param iNumberOfDimensions Number of dimensions in each histogram.
    /// @param iNumberOfBins Number of bins along each dimension.
    /// @param iMinRange The left edge of the first bin.
    /// @param iMaxRange The right edge of the last bin.
    virtual void create_histogram_uniform(int iNumberOfHistograms, int iNumberOfDimensions,
					  int iNumberOfBins, real iMinRange, real iMaxRange);

    /// Free histogram template memory.
    virtual void delete_histogram();


  // Getter and setter methods
  public:

    /// Return number of dimensions in histograms.
    int get_number_of_dimensions() const
      { return mNumberOfDimensions; }

    /// Return a pointer to a particular histogram in the template.
    /// @param iIndex The histogram to return.
    CvHistogram* get_histogram_pointer(int iIndex) const;

    /// Return a pointer to the array of histograms in the template.
    CvHistogram** get_histogram_pointer_array() const
      { return mpHistograms; }

    /// Return number of histograms in the template.
    int get_number_of_histograms() const
      { return mNumberOfHistograms; }

    /// Return whether histograms in the template have uniformly
    /// distributed bins.
    bool is_uniform() const
      { return CV_IS_UNIFORM_HIST(mpHistograms[0]); }

    /// Return the number of histogram bins along a particular
    /// dimension. If the dimension is not specified the total number
    /// of bins in the entire histogram is returned.
    /// @param iDimension The dimension along which to count the bins.
    ///        Use -1 (default) to return the total number of bins
    ///        across all dimensions.
    int get_number_of_bins(int iDimension = -1) const;


  // General methods
  public:

    /// Create and return a copy of the histogram template.
    virtual cvp_HistogramTemplate* create_copy() const;

    /// Test whether this histogram template has the same internal
    /// structure (number of histograms, dimensions and bins) as
    /// another histogram template.
    /// @param ipHistogramTemplate A pointer to the histogram template to
    ///        compare against.
    virtual bool same_template(const cvp_HistogramTemplate *ipHistogramTemplate) const;

    /// Linearly interpolate a histogram in this template with the
    /// corresponding histogram in another template. The update equation is
    ///     x := (1-iFactor)*x + iFactor*y
    /// where x is this histogram and y is the other.
    /// @param ipHistogramTemplate Pointer to the histogram template with which
    ///        to interpolate.
    /// @param iFactor The interpolation weight.
    /// @param iHistogramIndex Index of the histogram to update.
    void update(const cvp_HistogramTemplate* ipHistogramTemplate, real iFactor, int iHistogramIndex);


  // Input/output methods
  public:
    /// Display debug information about the histogram template.
    /// @param iOutputStream The output stream to which to write.
    ///        Default: stdout.
    /// @param iComments Optional additional commpents to print to the
    ///        output stream. Default: empty string.
    void debug_information(std::ostream iOutputStream,
			   const std::string iComments = "") const;
    // void debug_information(std::ostream iOutputStream = std::cout,
    //     		   const std::string iComments = "") const;

    /// Save the histogram template to a stream. Not implemented.
    virtual void save(std::ostream iOutputStream) const
      { throw cvp_ExceptionNotImplemented(); }

    /// Load the histogram template from a stream. Not implemented.
    virtual void load(std::istream iInputStream)
      { throw cvp_ExceptionNotImplemented(); }

  };

} // namespace OpenCvPlus

#endif
