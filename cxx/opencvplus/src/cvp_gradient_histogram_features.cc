/**
 * Copyright (c) 2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#include "opencvplus/cvp_gradient_histogram_features.h"

#include <stdio.h>
#include <sstream>
#include <fstream>
#include <iterator>
#include <iomanip>

using namespace std;

namespace OpenCvPlus {

  // --------------------------------------------------

  // SPURIOUS VARIABLES THAT SHOULD REALLY BE FACTORED OUT OR DELETED.
  // ALL OF THESE ARE USED IN cvp_CompareSimpleHistogramTemplates() ONLY.

  // Former global variables
  // Taken from the hash define of THRESHOLD_DISTANCE in bayes_filter/bf_Parameters.h
  const double CVP_HISTOGRAM_TEMPLATE_SIMPLE_DISTANCE_THRESHOLD = 1.0;
  // Taken from the hash define of LAMBDA_POSE in bayes_filter/bf_Parameters.h
  const double CVP_HISTOGRAM_TEMPLATE_SIMPLE_LAMBDA_POSE = 1.5;

  /*
  // Look-up table for computing the so-called L1 distance
  // REFACTOR NOTE: Look-up table replaced with conditional expression
  real* CVP_HISTOGRAM_TEMPLATE_SIMPLE_LUT_L1;
  // Create look up table for L1 distance
  CVP_HISTOGRAM_TEMPLATE_SIMPLE_LUT_L1 = new real [mNumberOfHistograms * mNumberOfBins];
  real tmpReal = 0.5 * mNumberOfHistograms * mNumberOfBins;
  for(int i = 0; i < mNumberOfHistograms * mNumberOfBins; i++) {
    if(i <= tmpReal)
      CVP_HISTOGRAM_TEMPLATE_SIMPLE_LUT_L1[i] = 1.0;
    else
      CVP_HISTOGRAM_TEMPLATE_SIMPLE_LUT_L1[i] = 0.0001; //0.001; //0.1; // 0.00000001;
  }
  */

  // --------------------------------------------------

  real cvp_CompareHistogramTemplates(const cvp_HistogramTemplate* ipH1,
				     const cvp_HistogramTemplate* ipH2,
				     cvp_HistogramDistance iMethod) {
    if(!ipH1->same_template(ipH2))
      throw cvp_ExceptionValueError("Histogram templates do not match.");

    int numberOfHistograms = ipH1->get_number_of_histograms();
    double result = 0;

    switch(iMethod) {

    // OpenCV distance metrics
    case CVP_HIST_DISTANCE_CORREL: // TODO: The correlation method
				   // should also be inverted, like
				   // the intersection method. In
				   // general, what guarantees to we
				   // want to make about the min/max
				   // distances?
    case CVP_HIST_DISTANCE_CHISQR:
    case CVP_HIST_DISTANCE_INTERSECT:
      for(int i = 0; i < numberOfHistograms; i++)
	result += cvCompareHist(ipH1->get_histogram_pointer(i),
				ipH2->get_histogram_pointer(i),
				(int)iMethod);
      if(iMethod == CVP_HIST_DISTANCE_INTERSECT)
	result = 1 - result;
      break;

    // Bhatacharyya distance
    case CVP_HIST_DISTANCE_BHATACHARYYA: // TODO: It looks like this
					 // is also implemented in
					 // OpenCV. Are the
					 // computations equaivalent?
					 // Then replace this code
					 // with a call to OpenCV.
      for(int pos = 0; pos < numberOfHistograms; pos++) {
	if(CV_IS_SPARSE_HIST(ipH1->get_histogram_pointer(pos)))
	  throw cvp_ExceptionValueError("Bhatacharrya coefficient computation not implemented with trees.");

	real *pReal1 = 0, *pReal2 = 0;
	int totalBins = ipH1->get_number_of_bins();
	cvGetRawData(ipH1->get_histogram_pointer(pos)->bins, (uchar**)&pReal1);
	cvGetRawData(ipH2->get_histogram_pointer(pos)->bins, (uchar**)&pReal2);
	for (int i = 0; i < totalBins; i++ )
	  // Remark: cvSqrt() is not as precise as sqrt()
	  result += cvSqrt(pReal1[i] * pReal2[i]);
      }
      result = cvSqrt(1 - result);
      break;

    // Haussdorf distance
    case CVP_HIST_DISTANCE_HAUSSDORF: { // Reduce scope of local variables in case statement
      if(ipH1->get_number_of_dimensions() != 3)
	throw cvp_ExceptionValueError("Haussdorf distance not implemented with number of dimensions != 3.");

      int numberOfDimensions = 3;
      int numberOfBins = ipH1->get_number_of_bins(0);

      for(int i = 1; i < numberOfDimensions; i++)
	if(ipH1->get_number_of_bins(i) != numberOfBins)
	  throw cvp_ExceptionValueError("Number of bins should be the same along all dimensions.");

      if(CV_IS_SPARSE_HIST(ipH1->get_histogram_pointer(0)))
	throw cvp_ExceptionValueError("Haussdorf distance not implemented with trees.");

      real* pReal1;
      real* pReal2;
      real v, vDistance, newVDistance;

#define VAL3(ad, i1, i2, i3, n)  *(ad+(n*i1+i2)*n+i3)
//#define MIN(x, y) ((x)<(y))?(x):(y)
//#define MAX(x, y) ((x)>(y))?(x):(y)

      for(int histogramIndex = 0; histogramIndex < numberOfHistograms; histogramIndex++) {
	pReal1 = (real*)ipH1->get_histogram_pointer(histogramIndex)->mat.data.fl;
	pReal2 = (real*)ipH2->get_histogram_pointer(histogramIndex)->mat.data.fl;
	for(int i = 0; i < numberOfBins; i++) {
	  for(int j = 0; j < numberOfBins; j++) {
	    for(int k = 0; k < numberOfBins; k++) {
	      v = VAL3(pReal1, i, j, k, numberOfBins);
	      // Find minimum distance between v and all other possible values
	      vDistance = 1e18; // Initializing vDistance = 1 should
				// suffice since histograms are
				// supposed to be normalized
	      for(int ii = MAX(0, i-1); ii <= MIN(numberOfBins-1, i+1); ii++)
		for(int jj = MAX(0, j-1); jj <= MIN(numberOfBins-1, j+1); jj++)
		  for(int kk = MAX(0, k-1); kk <= MIN(numberOfBins-1, k+1); kk++) {
		    newVDistance = fabs(v - VAL3(pReal2, ii, jj, kk, numberOfBins));
		    if(newVDistance < vDistance)
		      vDistance = newVDistance;
		  }
	      result += vDistance;
	    }
	  }
	}
      } // Sum has been performed for all bins and for all histograms in the template

#undef VAL3
#undef MIN
#undef MAX

      break;
    } // Scope

    case CVP_HIST_DISTANCE_FGBG:
      throw cvp_ExceptionNotImplemented();
      /*
      for(int pos = 0; pos < numberOfHistograms; pos++) {
	if(CV_IS_SPARSE_HIST(ipH1->get_histogram_pointer(pos)))
	  throw cvp_ExceptionValueError("FGBG distance computation not implemented with trees.");

	real *pReal1 = 0;
	real *pReal2 = 0;
	int totalBins = ipH1->get_number_of_bins();
	cvGetRawData(ipH1->get_histogram_pointer(pos)->bins, (uchar**)&pReal1);
	cvGetRawData(ipH2->get_histogram_pointer(pos)->bins, (uchar**)&pReal2);
	for(i = 0; i < totalBins; i++)
	  // Remark: cvSqrt() is not as precise as sqrt()
	  result += cvSqrt(pReal1[i] * pReal2[i]);
      }
      result = cvSqrt(1 - result);
      */
      break;

    default:
      throw cvp_ExceptionValueError("No such histogram distance defined.");
    }
    return (real)result;
  }

  // --------------------------------------------------

  real cvp_CompareSimpleHistogramTemplates(const cvp_HistogramTemplateSimple* ipH1,
					    const cvp_HistogramTemplateSimple* ipH2,
					    cvp_HistogramDistanceSimple iMethod) {
    if(!ipH1->same_template(ipH2))
      throw cvp_ExceptionValueError("Cannot compute distance between histogram templates with different characteristics.");

    int numberOfHistograms = ipH1->get_number_of_histograms();
    int numberOfBins = ipH1->get_number_of_bins();

    real result = 0;
    real sumReal, tmpReal;

    switch(iMethod) {
    case CVP_HIST_DISTANCE_EUCLIDEAN:
      for(int i = 0; i < numberOfHistograms; i++) {
	sumReal = 0.0;
	for(int j = 0; j < numberOfBins; j++) {
	  tmpReal = ipH1->get_histogram_pointer(i)[j] - ipH2->get_histogram_pointer(i)[j];
	  sumReal += tmpReal * tmpReal;
	}
	tmpReal = sqrt(sumReal);
	if(tmpReal > CVP_HISTOGRAM_TEMPLATE_SIMPLE_DISTANCE_THRESHOLD)
	  tmpReal = CVP_HISTOGRAM_TEMPLATE_SIMPLE_DISTANCE_THRESHOLD;
	result += tmpReal;
      }
      result *= CVP_HISTOGRAM_TEMPLATE_SIMPLE_LAMBDA_POSE;
      break;

    case CVP_HIST_DISTANCE_MAHALANOBIS:
      throw cvp_ExceptionNotImplemented("Mahalanobis distance for histograms not implemented.");
      break;

    case CVP_HIST_DISTANCE_L1:
      // TODO: BUG: This is not the L1 distance. I don't know wtf it's
      // supposed to be. The look-up table mLutL1 is the step function
      // ((x<=numberOfHistograms*numberOfBins)?1:0.0001), so why
      // bother using a look-up table in the first place? The
      // calculation below computes the squared L2 distance, which is
      // then passed through the step function. Very bizarre... and
      // this is being used in the current incarnation of the head
      // pose tracker.
      for(int i = 0; i < numberOfHistograms; i++) {
	sumReal = 0.0;
	for(int j = 0; j < numberOfBins; j++) {
	  tmpReal = ipH1->get_histogram_pointer(i)[j] - ipH2->get_histogram_pointer(i)[j];
	  sumReal += tmpReal * tmpReal;
	}
	result += sumReal;
      }
      // REFACTOR NOTE: This used to be implemented with a look-up
      // table, but has been replaced with a conditional expression
      //result = CVP_HISTOGRAM_TEMPLATE_SIMPLE_LUT_L1[(int)result];
      result = ((int)result <= (0.5*numberOfHistograms*numberOfBins)) ? 1.0 : 0.0001;
      break;

    default:
      throw cvp_ExceptionValueError("Histogram distance metric not defined.");
    }

    return result;
  }

  // --------------------------------------------------

  // void cvp_ComputeIntegralGradientHistogram

  // --------------------------------------------------

  void cvp_ComputeGradients(const IplImage* ipFloatImage, IplImage* opGradientX, IplImage* opGradientY,
			    IplImage* opGradientMagnitude, IplImage* opGradientAngle,
			    real iMagnitudeCap) {

      static const real HALF_PI_MUL_100 = 157;
      cvSetZero(opGradientX);
      cvSetZero(opGradientY);
      cvSetZero(opGradientMagnitude);
      cvSetZero(opGradientAngle);

    // Use OpenCV to compute X and Y gradient components

    CvRect cvroi = cvGetImageROI(ipFloatImage);
    cvSetImageROI(opGradientX, cvroi);
    cvSetImageROI(opGradientY, cvroi);
    cvSobel(ipFloatImage, opGradientX, 1, 0, 3);
    cvSobel(ipFloatImage, opGradientY, 0, 1, 3);
    cvResetImageROI(opGradientX);
    cvResetImageROI(opGradientY);

    real dx, dy, mag;
    for(int r = cvroi.y; r < cvroi.y + cvroi.height; r++) {
        for(int c = cvroi.x; c < cvroi.x + cvroi.width; c++) {
            dx = CV_IMAGE_ELEM(opGradientX, float, r, c);
            dy = CV_IMAGE_ELEM(opGradientY, float, r, c);

            // Compute angle
//            angle = dy / (dx + FLT_EPSILON);
            //th = cvRound(atan(angle)*100)+157;
//            angle = cvRound(cvp_arctan(dy / (dx + FLT_EPSILON))) + 157;

            // Compute magnitude
            mag = dx * dx + dy * dy;
            CV_IMAGE_ELEM(opGradientMagnitude, float, r, c) =
                cvSqrt((mag > iMagnitudeCap - 1) ? iMagnitudeCap - 1 : mag);
            // NOTE: this (arctan) is function defined in utils.cc; should be faster than math.h atan function
            // returns atan value scaled by 100
            // offset is added for the value to be in the range [0, 100 * PI]
            CV_IMAGE_ELEM(opGradientAngle, float, r, c) =
                cvRound(cvp_arctan(dy / (dx + FLT_EPSILON))) + HALF_PI_MUL_100;
        }
    }
  }

  // --------------------------------------------------

  /// Normalize each block in a block histogram.
  /// @param iopBlockHistogram The block histogram to normalize in place.
  /// @param iNumberOfEntries The total number of entries in the histogram.
  /// @param iNumberOfBlocks The number of entries per block. This must divide
  ///        into iNumberOfEntries without remainder.
  void _normalize_histogram_blocks(real* iopBlockHistogram, int iNumberOfEntries, int iNumberOfBlocks) {
    int entriesPerBlock = iNumberOfEntries / iNumberOfBlocks;
    real* pHistogram = new real[entriesPerBlock];
    for(int block = 0; block < iNumberOfBlocks; block++) {
      for(int i = 0; i < entriesPerBlock; i++)
	pHistogram[i] = iopBlockHistogram[block*entriesPerBlock+i];
      cvp_NormalizeRealVector(pHistogram, entriesPerBlock);
      for(int i = 0; i < entriesPerBlock; i++)
	iopBlockHistogram[block*entriesPerBlock+i] = pHistogram[i];
    }
    delete[] pHistogram;
  }

  // --------------------------------------------------

  void cvp_ComputeSumOfGradientHistograms(int iTopY, int iLeftX, int iBottomY, int iRightX,
					  CvMat const* const* ipIntegralGradientHistogram,
					  real* opHistogram, int iNumberOfBins) {

      cout << "CVP " << iLeftX << " " << iTopY << " " << iRightX << " " << iBottomY << " " << endl;
    for(int bin = 0; bin < iNumberOfBins; bin++) {
      int columns = ipIntegralGradientHistogram[bin]->cols;
      real const* data = ipIntegralGradientHistogram[bin]->data.fl;

      // Find and subtract values to top and left in the integral image
      real integrateUp(0), integrateLeft(0), integrateUpLeft(0);
      if(iLeftX == 0) { // No up
	integrateLeft = 0;
	integrateUpLeft = 0;
      } else
	integrateLeft = data[iBottomY * columns + (iLeftX-1)];
      if(iTopY == 0) { // No left
	integrateUp = 0;
	integrateUpLeft = 0;
      } else
	integrateUp = data[(iTopY-1) * columns + iRightX];
      if((iLeftX > 0) && (iTopY > 0))
	integrateUpLeft = data[(iTopY-1) * columns + (iLeftX-1)];

      opHistogram[bin] = data[iBottomY * columns + iRightX] +
                         integrateUpLeft - integrateLeft - integrateUp;
      cout << "    " << integrateLeft << " " << integrateUp << " " << integrateUpLeft << " " << opHistogram[bin] << endl;
    }
  }

  // --------------------------------------------------

  void cvp_ComputeIntegralGradientHistogram(
    IplImage const* ipGradientMagnitude, IplImage const* ipGradientAngle,
    CvMat** opIntegralGradientHistogram, int iNumberOfBins) {

    static real** histogramBinLookup = 0;
    static int cachedNumberOfBins = 0;

    static const int MIN_INT_ANGLE = 0;   // minimal integer gradient angle
    static const int MAX_INT_ANGLE = 314; // maximal integer gradient angle
    // The inverse length scale at which to smooth the bin-angle mapping
    static const real SMOOTHING_BW = 3;

    if(!histogramBinLookup || (cachedNumberOfBins != iNumberOfBins)) {
      // Build histogram bin look-up table. This is a mapping from
      // (bin,angle) pairs to the probability that angle belongs to
      // the given gradient histogram bin.

        // Release previously allocated memory, if necessary
        if (histogramBinLookup) {
            for(int bin = 0; bin < cachedNumberOfBins; ++bin) {
                delete[] histogramBinLookup[bin];
            }
            delete[] histogramBinLookup;
        }

        cachedNumberOfBins = iNumberOfBins;

        // Allocate memory
        histogramBinLookup = new real* [iNumberOfBins];
        for(int bin = 0; bin < iNumberOfBins; bin++) {
            histogramBinLookup[bin] = new real [MAX_INT_ANGLE + 1];
        }

      // Compute temporary look-up table for bin centers of angle histogram
      real stepAngle = static_cast<real>(MAX_INT_ANGLE - MIN_INT_ANGLE) /
              iNumberOfBins;
      vector<real> theta_centers(iNumberOfBins);
      for(int bin = 0; bin < iNumberOfBins; bin++) {
          theta_centers[bin] = round(MIN_INT_ANGLE + (bin + 0.5) * stepAngle);
      }

      real weight;
      real weight_sum;
      for(int angle = MIN_INT_ANGLE; angle <= MAX_INT_ANGLE; angle++) {
          weight_sum = 0;
          for(int bin = 0; bin < iNumberOfBins; ++bin) {
              weight = exp(-SMOOTHING_BW * fabs(theta_centers[bin] - angle));
              histogramBinLookup[bin][angle] = weight;
              weight_sum += weight;
          }
          // Normalize
          for(int bin = 0; bin < iNumberOfBins; ++bin) {
              histogramBinLookup[bin][angle] /= weight_sum;
          }
      }
      ofstream ofs("cvp_normalized_weights.txt");
      for(int bin = 0; bin < iNumberOfBins; bin++) {
          ostream_iterator<OpenCvPlus::real> out_it(ofs, " ");
          copy(&histogramBinLookup[bin][0],
               &histogramBinLookup[bin][MAX_INT_ANGLE + 1], out_it);
          ofs << endl;
      }
    }

    // Reset integral histogram matrices
    for(int i = 0; i < iNumberOfBins; i++) {
        cvSetZero(opIntegralGradientHistogram[i]);
    }

    // Prepare helper variables for stepping through gradient image data
    int stepMagnitude = ipGradientMagnitude->widthStep / sizeof(real) -
            ipGradientMagnitude->width; // because of alignment
    int stepAngle = ipGradientAngle->widthStep / sizeof(real) -
            ipGradientAngle->width; // because of alignment
    const real * pDataMagnitude = (real*)ipGradientMagnitude->imageData;
    const real * pDataAngle = (real*)ipGradientAngle->imageData;

    real* pHistogramData;   // pointer to the integral image data
    int imageColumns;       // number of columns in the integral image
    int prevRow;            // stores the offset for previous row
    int thisRow;            // stores the offset for current row
    real integrateUp;       // histogram value for upper neighbour
    real integrateLeft;     // histogram value for left neighbour
    real integrateUpLeft;   // histogram value for upper-left neighbour
    real currentValue;      // histogram value for current cell
    real* pHistogramLookupData; // pointer to the current histogram lookup data

    // Build integral image
    for(int k = 0; k < iNumberOfBins; k++) {
        std::ostringstream oss;
        oss << "cvp_histogram_data" << k << ".jpg";
        std::ostringstream osstxt;
        osstxt << "cvp_histogram_data" << k << ".txt";
        ofstream ofs(osstxt.str().c_str());
        //IplImage image;
        pDataMagnitude = (real*)ipGradientMagnitude->imageData;
        pDataAngle = (real*)ipGradientAngle->imageData;
        pHistogramData = opIntegralGradientHistogram[k]->data.fl;
        imageColumns = opIntegralGradientHistogram[k]->cols;
        pHistogramLookupData = histogramBinLookup[k];
        for(int row = 0; row < ipGradientAngle->height; row++) {
            integrateLeft = 0;
            for(int column = 0; column < ipGradientAngle->width; column++) {
                thisRow = row * imageColumns;
                prevRow = thisRow - imageColumns;
                integrateUp = (row ? pHistogramData[prevRow + column] : 0);
                integrateUpLeft = (((row) && (column)) ?
                        pHistogramData[prevRow + column - 1] : 0);
                currentValue =
                        (*pDataMagnitude) * pHistogramLookupData[
                                static_cast<int>(*pDataAngle)] +
                        integrateLeft + integrateUp - integrateUpLeft;
                pHistogramData[thisRow + column] = currentValue;
                ofs << currentValue << " ";
                integrateLeft = currentValue;
                pDataMagnitude++;
                pDataAngle++;
            } // Next column
            ofs << endl;
            pDataMagnitude += stepMagnitude; // Move to next row in input images
            pDataAngle += stepAngle;
        } // Next row
        //cvGetImage(pHistogramData)
        cvSaveImage(oss.str().c_str(), opIntegralGradientHistogram[k]);
    } // Next integral image
  }

  // --------------------------------------------------

  void cvp_ComputeBlockGradientHistogram(
    int iImageHeight, int iImageWidth, real* opHistogram,
    real* opTempHistogram, int iNumberOfBins,
    CvMat const* const* ipIntegralGradientHistogram,
    int iNumberOfColumnBlocks, int iNumberOfRowBlocks,
    int iColumnsPerBlock, int iRowsPerBlock) {

    // Allocate temporary storage, if necessary
    bool releaseTempHistogram = false;
    if(opTempHistogram == NULL) {
      opTempHistogram = new real[iNumberOfBins];
      releaseTempHistogram = true;
    }

    int numberOfBlocks = iNumberOfColumnBlocks * iNumberOfRowBlocks * iColumnsPerBlock * iRowsPerBlock;
    float stepRow = static_cast<float>(iImageHeight) / (iRowsPerBlock*iNumberOfRowBlocks);
    float stepColumn = static_cast<float>(iImageWidth) / (iColumnsPerBlock*iNumberOfColumnBlocks);

    // Compute the bounding box coordinates (top, left, bottom, right)
    // of each block
    int** pBlockCoordinates = new int* [numberOfBlocks];
    for(int i = 0; i < numberOfBlocks; i++)
      pBlockCoordinates[i] = new int [4];
    int blockIndex = 0;
    for(int blockRow = 0; blockRow < iNumberOfRowBlocks; blockRow++) {
      for(int blockColumn = 0; blockColumn < iNumberOfColumnBlocks; blockColumn++) {
	for(int rowInBlock = 0; rowInBlock < iRowsPerBlock; rowInBlock++) {
	  for(int columnInBlock = 0; columnInBlock < iColumnsPerBlock; columnInBlock++) {
	    int imageRow = blockRow * iRowsPerBlock + rowInBlock;
	    int imageColumn = blockColumn * iColumnsPerBlock + columnInBlock;
	    pBlockCoordinates[blockIndex][0] = std::max(static_cast<double>(round(imageRow*stepRow)),
                                                        static_cast<double>(0.0));
	    pBlockCoordinates[blockIndex][1] = std::max(static_cast<double>(round(imageColumn*stepColumn)),
                                                        static_cast<double>(0.0));
	    pBlockCoordinates[blockIndex][2] = std::min(static_cast<double>(round((imageRow+1)*stepRow)),
                                                        static_cast<double>(iImageHeight - 1.0));
	    pBlockCoordinates[blockIndex][3] = std::min(static_cast<double>(round((imageColumn+1)*stepColumn)),
                                                        static_cast<double>(iImageWidth - 1.0));
	    // To be sure that we fill the input Bounding Box
	    // Note that Bounding Box on the right side and bottom might
	    // be bigger than the other ones
//	    if(imageColumn == iColumnsPerBlock * iNumberOfColumnBlocks - 1)
//	      pBlockCoordinates[blockIndex][3] = iImageWidth-1;
//	    if(imageRow == iRowsPerBlock * iNumberOfRowBlocks - 1)
//	      pBlockCoordinates[blockIndex][2] = iImageHeight-1;
	    blockIndex++;
	  }
	}
      }
    }

    ostringstream oss;
    oss << "cvp_hog_feature_" << iNumberOfColumnBlocks << iNumberOfRowBlocks <<
        iColumnsPerBlock << iRowsPerBlock;
    ofstream ofs1((oss.str() + "_unnormalized.txt").c_str());
    ostringstream oss_blocks;
    oss_blocks << "cvp_feature_blocks_" << iNumberOfColumnBlocks << iNumberOfRowBlocks <<
        iColumnsPerBlock << iRowsPerBlock << ".txt";
    ofstream ofs_blocks(oss_blocks.str().c_str());
    // Compute the sum of histograms in each block, and normalize
    for(int blockIndex = 0; blockIndex < numberOfBlocks; blockIndex++) {
      cvp_ComputeSumOfGradientHistograms(pBlockCoordinates[blockIndex][0], pBlockCoordinates[blockIndex][1],
					 pBlockCoordinates[blockIndex][2], pBlockCoordinates[blockIndex][3],
					 ipIntegralGradientHistogram, opTempHistogram, iNumberOfBins);
      ofs_blocks << pBlockCoordinates[blockIndex][0] << " "
                 << pBlockCoordinates[blockIndex][1] << " "
                 << pBlockCoordinates[blockIndex][2] - pBlockCoordinates[blockIndex][0] + 1 << " "
                 << pBlockCoordinates[blockIndex][3] - pBlockCoordinates[blockIndex][1] + 1 << endl;
      for(int bin = 0; bin < iNumberOfBins; bin++) {
          opHistogram[blockIndex*iNumberOfBins+bin] = opTempHistogram[bin];
          ofs1 << setw(11) << setfill(' ') << opTempHistogram[bin] << " ";
      }
      ofs1 << endl;
    }

    ofstream ofs2((oss.str() + "_normalized1.txt").c_str());
    _normalize_histogram_blocks(opHistogram, iNumberOfBins*numberOfBlocks, iNumberOfColumnBlocks*iNumberOfRowBlocks);
    for(int blockIndex = 0; blockIndex < numberOfBlocks; blockIndex++) {
        for(int bin = 0; bin < iNumberOfBins; bin++) {
            ofs2 << opHistogram[blockIndex*iNumberOfBins+bin];
        }
        ofs2 << endl;
    }
    ofstream ofs3((oss.str() + "_normalized2.txt").c_str());
    cvp_NormalizeRealVector(opHistogram, iNumberOfBins*numberOfBlocks);
    for(int blockIndex = 0; blockIndex < numberOfBlocks; blockIndex++) {
        for(int bin = 0; bin < iNumberOfBins; bin++) {
            ofs3 << opHistogram[blockIndex*iNumberOfBins+bin];
        }
        ofs3 << endl;
    }

    // Release memory
    for(int i = 0; i < numberOfBlocks; i++)
      delete [] pBlockCoordinates[i];
    delete [] pBlockCoordinates;
    if(releaseTempHistogram)
      delete [] opTempHistogram;
  }

  // --------------------------------------------------

  void cvp_ComputeIntegralImage(IplImage const* ipSource, CvMat* opIntegralImage) {
    cvSetZero(opIntegralImage);

    float* pIntegralImageData = opIntegralImage->data.fl;
    int integralImageColumns = opIntegralImage->cols;

    // Fill first element, row = 0, column = 0
    pIntegralImageData[0] = cvp_PixVal8U(ipSource, 0, 0);

    // Fill first column, column = 0
    for(int row = 1; row < ipSource->height; row++)
      pIntegralImageData[row*integralImageColumns] = pIntegralImageData[(row-1)*integralImageColumns] + cvp_PixVal8U(ipSource, row, 0);

    // Fill first row, row = 0
    for(int column = 1; column < ipSource->width; column++)
      pIntegralImageData[column] = pIntegralImageData[column-1] + cvp_PixVal8U(ipSource, 0, column);

    for(int row = 1; row < ipSource->height; row++) {
      for(int column = 1; column < ipSource->width; column++) {
	int prevRowPointer = (row-1)*integralImageColumns;
	int rowPointer = row*integralImageColumns;
	pIntegralImageData[rowPointer+column] =
	  pIntegralImageData[rowPointer+column-1] +
	  pIntegralImageData[prevRowPointer+column] -
	  pIntegralImageData[prevRowPointer+column-1] +
	  cvp_PixVal8U(ipSource, row, column);
      }
    }
  }

  // --------------------------------------------------

} // namespace OpenCvPlus
