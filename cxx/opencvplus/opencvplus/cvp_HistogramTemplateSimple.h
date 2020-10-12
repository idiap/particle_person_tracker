/**
 * cvp_HistogramTemplateSimple class: A storage class for a histogram
 * template (an array of histograms).
 *
 * This is a much simplified version of the cvp_HistogramTemplate
 * class. It also uses different distance metrics for comparing
 * histograms. Simplifications include the following.
 *  - There is always exactly 1 dimension.
 *  - Histograms are stored as float arrays.
 *  - Bins are always uniformly distributed.
 *  - Bin ranges are not specified, only bin counts.
 *
 * A histogram template is simply a 1d array of histograms. The
 * meaning of the index in the array depends on the usage. Each
 * histogram in the array can be multi-dimensional. This is a storage
 * class in that the computation of the histograms is left up to the
 * user. The distance between two histogram templates can be computed
 * with the distance() method. There are various distance metrics
 * available, see the cvp_HistogramDistance enum for details.
 *
 * @author Elisa Ricci (Elisa.Ricci@idiap.ch)
 * @author Carl Scheffler (Carl.Scheffler@idiap.ch)
 *
 * See COPYING file for the complete license text.
 */

#ifndef __CVP_HISTOGRAMTEMPLATESIMPLE_H__
#define __CVP_HISTOGRAMTEMPLATESIMPLE_H__

#include "cvplus.h"

#include <string>
#include <iostream>

namespace OpenCvPlus {

  class cvp_HistogramTemplateSimple {

  // Attributes
  private:

    /// Array of histograms.
    real** mpHistograms;

    /// Number of histograms in the template
    int mNumberOfHistograms;

    /// Number of bins in each histogram
    int mNumberOfBins;


  // Lifecycle methods
  public:

    /// Default constructor.
    cvp_HistogramTemplateSimple();

    /// Constructor, creating the histogram template. Histogram
    /// entries are *not* zeroed.
    /// @param iNumberOfHistograms The number of histograms in the template.
    /// @param iNumberOfBins The number of bins in each histogram.
    cvp_HistogramTemplateSimple(int iNumberOfHistograms, int iNumberOfBins);

    /// Destructor.
    virtual ~cvp_HistogramTemplateSimple();


  // Lifecycle support methods
  private:

    /// Free histogram template memory.
    virtual void delete_histogram();

    /// Allocate memory for histogram template.
    virtual void create_histogram(int iNumberOfHistograms, int iNumberOfBins);


  // Getters and setters
  public:

    /// Return a pointer to a particular histogram in the template.
    /// @param iIndex The histogram to return.
    real* get_histogram_pointer(int iIndex) const;

    /// Return a pointer to the array of histograms in the template.
    real** get_histogram_pointer_array() const
      { return mpHistograms; }

    /// Return number of bins per histogram.
    int get_number_of_bins() const
      { return mNumberOfBins; }

    /// Return number of histograms in the template.
    int get_number_of_histograms() const
      { return mNumberOfHistograms; }


  // General methods
  public:

    /// Create and return a copy of the histogram template.
    virtual cvp_HistogramTemplateSimple *create_copy() const;

    /// Test whether this histogram template has the same internal
    /// structure (number of histograms and bins) as another histogram
    /// template.
    /// @param ipHistogramTemplate A pointer to the histogram template to
    ///        compare against.
    virtual bool same_template(const cvp_HistogramTemplateSimple* ipHistogramTemplate) const;


  // Input/output methods
  public:
    /// Display debug information about the histogram template.
    /// @param iOutputStream The output stream to which to write.
    ///        Default: stdout.
    /// @param iComments Optional additional commpents to print to the
    ///        output stream. Default: empty string.
    void debug_information(std::ostream& iOutputStream = std::cout,
			   const std::string iComments = "") const;

    /// Save the histogram template to a stream. Not implemented.
    virtual void save(std::ostream& iOutputStream) const
      { throw cvp_ExceptionNotImplemented(); }

    /// Load the histogram template from a stream. Not implemented.
    virtual void load(std::istream& iInputStream)
      { throw cvp_ExceptionNotImplemented(); }

  };

} // namespace OpenCvPlus

#endif
