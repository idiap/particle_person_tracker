// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_HistogramTemplate - A storage class for a histogram template
//                        (an array of histograms).
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//          Carl Scheffler (Carl.Scheffler@idiap.ch)
//
// See COPYING file for the complete license text.

// SYSTEM INCLUDES
#include <boost/foreach.hpp>         // foreach loop
#include <boost/lambda/lambda.hpp>   // for transformation in operators
#include <iterator>                  // for vector output
#include <numeric>                   // for inner product

// LOCAL INCLUDES
#include <image_processing/ip_HistogramTemplate.h>  // declaration of this

namespace ImageProcessing {

/////////////////////////////// PUBLIC ///////////////////////////////////////

ip_HistogramTemplate::ip_HistogramTemplate(unsigned num_histograms,
        unsigned num_bins) {

    Histogram hist_prototype(num_bins, 0);
    m_HistogramCollection.assign(num_histograms, hist_prototype);

} // ip_HistogramTemplate

/* static */ ip_HistogramTemplate::~ip_HistogramTemplate() {
    m_HistogramCollection.clear();
} // ~ip_HistogramTemplate

void ip_HistogramTemplate::clear() {
    BOOST_FOREACH(Histogram& histogram, m_HistogramCollection) {
        fill(histogram.begin(), histogram.end(), 0.0);
    }
} // clear

/////////////////////////////// OPERATORS ////////////////////////////////////

using namespace boost::lambda;

ip_HistogramTemplate&
ip_HistogramTemplate::operator=(const ip_HistogramTemplate& rhs) {
    if (this != &rhs) {

        m_HistogramCollection = rhs.m_HistogramCollection;
//        std::vector<ImageProcessing::ip_HistogramTemplate::Histogram>
//            ::iterator ii;
//        std::vector<ImageProcessing::ip_HistogramTemplate::Histogram>
//            ::const_iterator jj;
//
//        for (ii = m_HistogramCollection.begin(),
//            jj = rhs.m_HistogramCollection.begin();
//            (ii != m_HistogramCollection.end()) &&
//            (jj != rhs.m_HistogramCollection.end()); ++ii, ++jj) {
//                ii->assign(jj->begin(), jj->end());
//        }
    }
    return *this;
} // =

ip_HistogramTemplate&
ip_HistogramTemplate::operator+=(const ip_HistogramTemplate& rhs) {
    std::vector<ImageProcessing::ip_HistogramTemplate::Histogram>
        ::iterator ii;
    std::vector<ImageProcessing::ip_HistogramTemplate::Histogram>
        ::const_iterator jj;

    for (ii = m_HistogramCollection.begin(),
        jj = rhs.m_HistogramCollection.begin();
        (ii != m_HistogramCollection.end()) &&
        (jj != rhs.m_HistogramCollection.end()); ++ii, ++jj) {
            transform(ii->begin(), ii->end(), jj->begin(), ii->begin(),
                    _1 + _2);
    }
    return *this;
} // +=

ip_HistogramTemplate&
ip_HistogramTemplate::operator+=(const OpenCvPlus::real& rhs) {
    std::vector<ip_HistogramTemplate::Histogram>::iterator ii;
    for (ii = m_HistogramCollection.begin();
         ii != m_HistogramCollection.end(); ++ii) {
            transform(ii->begin(), ii->end(), ii->begin(), _1 + rhs);
    }
    return *this;
} // +=

ip_HistogramTemplate&
ip_HistogramTemplate::operator-=(const ip_HistogramTemplate& rhs) {
    std::vector<ImageProcessing::ip_HistogramTemplate::Histogram>
        ::iterator ii;
    std::vector<ImageProcessing::ip_HistogramTemplate::Histogram>
        ::const_iterator jj;

    for (ii = m_HistogramCollection.begin(),
        jj = rhs.m_HistogramCollection.begin();
        (ii != m_HistogramCollection.end()) &&
        (jj != rhs.m_HistogramCollection.end()); ++ii, ++jj) {
            transform(ii->begin(), ii->end(), jj->begin(), ii->begin(),
                    _1 - _2);
    }
    return *this;
} // -=

ip_HistogramTemplate&
ip_HistogramTemplate::operator-=(const OpenCvPlus::real& rhs) {
    std::vector<ip_HistogramTemplate::Histogram>::iterator ii;
    for (ii = m_HistogramCollection.begin();
         ii != m_HistogramCollection.end(); ++ii) {
            transform(ii->begin(), ii->end(), ii->begin(), _1 - rhs);
    }
    return *this;
} // -=

ip_HistogramTemplate&
ip_HistogramTemplate::operator*=(const ip_HistogramTemplate& rhs) {
    std::vector<ImageProcessing::ip_HistogramTemplate::Histogram>
        ::iterator ii;
    std::vector<ImageProcessing::ip_HistogramTemplate::Histogram>
        ::const_iterator jj;

    for (ii = m_HistogramCollection.begin(),
        jj = rhs.m_HistogramCollection.begin();
        (ii != m_HistogramCollection.end()) &&
        (jj != rhs.m_HistogramCollection.end()); ++ii, ++jj) {
            transform(ii->begin(), ii->end(), jj->begin(), ii->begin(),
                    _1 * _2);
    }
    return *this;
} // *=

ip_HistogramTemplate&
ip_HistogramTemplate::operator*=(const OpenCvPlus::real& rhs) {
    std::vector<ip_HistogramTemplate::Histogram>::iterator ii;
    for (ii = m_HistogramCollection.begin();
         ii != m_HistogramCollection.end(); ++ii) {
            transform(ii->begin(), ii->end(), ii->begin(), _1 * rhs);
    }
    return *this;
} // *=

ip_HistogramTemplate&
ip_HistogramTemplate::operator/=(const ip_HistogramTemplate& rhs) {
    std::vector<ImageProcessing::ip_HistogramTemplate::Histogram>
        ::iterator ii;
    std::vector<ImageProcessing::ip_HistogramTemplate::Histogram>
        ::const_iterator jj;

    for (ii = m_HistogramCollection.begin(),
        jj = rhs.m_HistogramCollection.begin();
        (ii != m_HistogramCollection.end()) &&
        (jj != rhs.m_HistogramCollection.end()); ++ii, ++jj) {
            transform(ii->begin(), ii->end(), jj->begin(), ii->begin(),
                    _1 / _2);
    }
    return *this;
} // /=

ip_HistogramTemplate&
ip_HistogramTemplate::operator/=(const OpenCvPlus::real& rhs) {
    std::vector<ip_HistogramTemplate::Histogram>::iterator ii;
    for (ii = m_HistogramCollection.begin();
         ii != m_HistogramCollection.end(); ++ii) {
            transform(ii->begin(), ii->end(), ii->begin(), _1 / rhs);
    }
    return *this;
} // /=


OpenCvPlus::real distance_L1(const ip_HistogramTemplate& hog1,
        const ip_HistogramTemplate& hog2,
        OpenCvPlus::real threshold) {
    typedef std::vector<ip_HistogramTemplate::Histogram> HistogramVector;

    const HistogramVector::const_iterator iihc_begin =
            hog1.m_HistogramCollection.begin();
    const HistogramVector::const_iterator iihc_end =
            hog1.m_HistogramCollection.end();
    HistogramVector::const_iterator iihc = iihc_begin;

    const HistogramVector::const_iterator jjhc_begin =
            hog2.m_HistogramCollection.begin();
    const HistogramVector::const_iterator jjhc_end =
            hog2.m_HistogramCollection.end();
    HistogramVector::const_iterator jjhc = jjhc_begin;

    OpenCvPlus::real sum_abs;
    OpenCvPlus::real result = 0.0f;
    for (; (iihc != iihc_end) && (jjhc != jjhc_end); ++iihc, ++jjhc) {
        const ip_HistogramTemplate::Histogram::const_iterator ii_begin =
                iihc->begin();
        const ip_HistogramTemplate::Histogram::const_iterator ii_end =
                iihc->end();
        ip_HistogramTemplate::Histogram::const_iterator ii = ii_begin;
        const ip_HistogramTemplate::Histogram::const_iterator jj_begin =
                jjhc->begin();
        const ip_HistogramTemplate::Histogram::const_iterator jj_end =
                jjhc->end();
        ip_HistogramTemplate::Histogram::const_iterator jj = jj_begin;
        sum_abs = 0.0f;
        for (; (ii != ii_end) && (jj != jj_end); ++ii, ++jj) {
            sum_abs += fabs(*ii - *jj);
        }
        result +=
            ((threshold >= 0) ?
                    ((sum_abs > threshold) ? threshold : sum_abs) :
                    sum_abs);
    }
    return result;
} // distance_L1

OpenCvPlus::real
distance_L2_squared(const ip_HistogramTemplate& hog1,
        const ip_HistogramTemplate& hog2,
        OpenCvPlus::real threshold) {

    typedef std::vector<ip_HistogramTemplate::Histogram> HistogramVector;
    typedef ip_HistogramTemplate::Histogram Hist;
    typedef ip_HistogramTemplate::Histogram::value_type HistVal;

    OpenCvPlus::real result = 0.0f;

    const Hist * pColl1 = &hog1.m_HistogramCollection[0];
    const Hist * pColl1End = pColl1 + hog1.m_HistogramCollection.size();
    const Hist * pColl2 = &hog2.m_HistogramCollection[0];
//    const Hist * pColl2End = pColl2 + hog2.m_HistogramCollection.size();

    const HistVal * pHist1;
    const HistVal * pHist1End;
    const HistVal * pHist2;
    const HistVal * pHist2End;

    OpenCvPlus::real sum_sqr, sqr_val;
    for (; pColl1 != pColl1End; ++pColl1, ++pColl2) {
        pHist1 = &(*pColl1)[0];
        pHist1End = pHist1 + pColl1->size();
        pHist2 = &(*pColl2)[0];
        pHist2End = pHist2 + pColl2->size();
        sum_sqr = 0.0f;
        for (; pHist1 != pHist1End; ++pHist1, ++pHist2) {
            sqr_val = *pHist1 - *pHist2;
            sum_sqr += sqr_val * sqr_val;
        }
        result +=
            ((threshold >= 0) ?
                    ((sum_sqr > threshold) ? threshold : sum_sqr) :
                    sum_sqr);
    }
    return result;
} // distance_L2_squared

OpenCvPlus::real distance_Mahalanobis_squared(const ip_HistogramTemplate& hog1,
        const ip_HistogramTemplate& hog2, const ip_HistogramTemplate& stddev,
        OpenCvPlus::real threshold) {

    OpenCvPlus::real result = 0.0f;

    std::vector<ip_HistogramTemplate::Histogram>::const_iterator ii;
    std::vector<ip_HistogramTemplate::Histogram>::const_iterator jj;
    std::vector<ip_HistogramTemplate::Histogram>::const_iterator kk;

    ip_HistogramTemplate::Histogram temp_hist(hog1.get_histogram(0));
    OpenCvPlus::real sum_sqr;

    for (ii = hog1.m_HistogramCollection.begin(),
         jj = hog2.m_HistogramCollection.begin(),
         kk = stddev.m_HistogramCollection.begin();
        (ii != hog1.m_HistogramCollection.end()) &&
        (jj != hog2.m_HistogramCollection.end()) &&
        (kk != stddev.m_HistogramCollection.end()); ++ii, ++jj, ++kk) {
        transform(ii->begin(), ii->end(), jj->begin(), temp_hist.begin(),
                _1 - _2);
        transform(temp_hist.begin(), temp_hist.end(), kk->begin(),
                temp_hist.begin(), _1 / _2);
        sum_sqr = inner_product(temp_hist.begin(), temp_hist.end(),
                temp_hist.begin(), 0.0f);
        result +=
            ((threshold >= 0) ?
                    ((sum_sqr > threshold) ? threshold : sum_sqr) :
                    sum_sqr);
    }
    return result;
} // distance_Mahalanobis_squared

OpenCvPlus::real distance_Chi2(const ip_HistogramTemplate& hog1,
        const ip_HistogramTemplate& hog2,
        OpenCvPlus::real threshold /* = -1 */) {

    OpenCvPlus::real result = 0.0f;

    typedef std::vector<ip_HistogramTemplate::Histogram> HistogramVector;
    typedef ip_HistogramTemplate::Histogram Hist;
    typedef ip_HistogramTemplate::Histogram::value_type HistVal;

    const Hist * pColl1 = &hog1.m_HistogramCollection[0];
    const Hist * pColl1End = pColl1 + hog1.m_HistogramCollection.size();
    const Hist * pColl2 = &hog2.m_HistogramCollection[0];

    const HistVal * pHist1;
    const HistVal * pHist1End;
    const HistVal * pHist2;
    const HistVal * pHist2End;

    std::vector<ip_HistogramTemplate::Histogram>::const_iterator ii;
    std::vector<ip_HistogramTemplate::Histogram>::const_iterator jj;

    ip_HistogramTemplate::Histogram temp_hist_1(hog1.get_histogram(0));
    ip_HistogramTemplate::Histogram temp_hist_2(temp_hist_1);
    OpenCvPlus::real sum_sqr;

    for (ii = hog1.m_HistogramCollection.begin(),
         jj = hog2.m_HistogramCollection.begin();
        (ii != hog1.m_HistogramCollection.end()) &&
        (jj != hog2.m_HistogramCollection.end()); ++ii, ++jj) {
        transform(ii->begin(), ii->end(),
                  jj->begin(), temp_hist_1.begin(),
                _1 - _2);
        transform(ii->begin(), ii->end(),
                  jj->begin(), temp_hist_2.begin(),
                _1 + _2);
        transform(temp_hist_1.begin(), temp_hist_1.end(),
                temp_hist_1.begin(), temp_hist_1.begin(),
                _1 * _2);
        transform(temp_hist_1.begin(), temp_hist_1.end(),
                temp_hist_2.begin(), temp_hist_1.begin(),
                _1 / _2);
        sum_sqr = accumulate(temp_hist_1.begin(), temp_hist_1.end(), 0.0f);
        result +=
            ((threshold >= 0) ?
                    ((sum_sqr > threshold) ? threshold : sum_sqr) :
                    sum_sqr);
    }
    return result;
} // distance_Chi2


} // namespace ImageProcessing

/////////////////////////////// GLOBAL ///////////////////////////////////////

using namespace std;
using namespace boost::lambda;

std::ostream& operator<<(std::ostream& os,
        const ImageProcessing::ip_HistogramTemplate& hist_template) {

    ostream_iterator<OpenCvPlus::real> out_it (os, ", ");

    for (unsigned i = 0; i < hist_template.size(); ++i) {
        const ImageProcessing::ip_HistogramTemplate::Histogram& histogram =
                hist_template.get_histogram(i);
        os << i << ": (";
        copy (histogram.begin(), histogram.end(), out_it );
        os << ")" << endl;
    }
    return os;
}

ImageProcessing::ip_HistogramTemplate operator-(
        const ImageProcessing::ip_HistogramTemplate& hp1,
        const ImageProcessing::ip_HistogramTemplate& hp2) {

    ImageProcessing::ip_HistogramTemplate result(hp1);

    std::vector<ImageProcessing::ip_HistogramTemplate::Histogram>::iterator ii;
    std::vector<ImageProcessing::ip_HistogramTemplate::Histogram>
        ::const_iterator jj;

   for (ii = result.m_HistogramCollection.begin(),
        jj = hp2.m_HistogramCollection.begin();
        (ii != result.m_HistogramCollection.end()) &&
        (jj != hp2.m_HistogramCollection.end()); ++ii, ++jj) {
       transform(ii->begin(), ii->end(), jj->begin(), ii->begin(), _1 - _2);
   }

   return result;
} // -

ImageProcessing::ip_HistogramTemplate operator-(
        const ImageProcessing::ip_HistogramTemplate& ht,
        const OpenCvPlus::real& v) {
    ImageProcessing::ip_HistogramTemplate result(ht);
    std::vector<ImageProcessing::ip_HistogramTemplate::Histogram>::iterator ii;
    for (ii = result.m_HistogramCollection.begin();
         ii != result.m_HistogramCollection.end(); ++ii) {
        transform(ii->begin(), ii->end(), ii->begin(), _1 - v);
    }
    return result;
} // +

ImageProcessing::ip_HistogramTemplate operator-(
        const OpenCvPlus::real& v,
        const ImageProcessing::ip_HistogramTemplate& ht) {
    ImageProcessing::ip_HistogramTemplate result(ht);
    std::vector<ImageProcessing::ip_HistogramTemplate::Histogram>::iterator ii;
    for (ii = result.m_HistogramCollection.begin();
         ii != result.m_HistogramCollection.end(); ++ii) {
        transform(ii->begin(), ii->end(), ii->begin(), v - _1);
    }
    return result;
} // +

ImageProcessing::ip_HistogramTemplate operator+(
        const ImageProcessing::ip_HistogramTemplate& hp1,
        const ImageProcessing::ip_HistogramTemplate& hp2) {

    ImageProcessing::ip_HistogramTemplate result(hp1);

    std::vector<ImageProcessing::ip_HistogramTemplate::Histogram>::iterator ii;
    std::vector<ImageProcessing::ip_HistogramTemplate::Histogram>
        ::const_iterator jj;

   for (ii = result.m_HistogramCollection.begin(),
        jj = hp2.m_HistogramCollection.begin();
        (ii != result.m_HistogramCollection.end()) &&
        (jj != hp2.m_HistogramCollection.end()); ++ii, ++jj) {
       transform(ii->begin(), ii->end(), jj->begin(), ii->begin(), _1 + _2);
   }

   return result;
} // +

ImageProcessing::ip_HistogramTemplate operator+(
        const ImageProcessing::ip_HistogramTemplate& ht,
        const OpenCvPlus::real& v) {
    ImageProcessing::ip_HistogramTemplate result(ht);
    std::vector<ImageProcessing::ip_HistogramTemplate::Histogram>::iterator ii;
    for (ii = result.m_HistogramCollection.begin();
         ii != result.m_HistogramCollection.end(); ++ii) {
        transform(ii->begin(), ii->end(), ii->begin(), _1 + v);
    }
    return result;
} // +

ImageProcessing::ip_HistogramTemplate operator+(
        const OpenCvPlus::real& v,
        const ImageProcessing::ip_HistogramTemplate& ht) {
    ImageProcessing::ip_HistogramTemplate result(ht);
    std::vector<ImageProcessing::ip_HistogramTemplate::Histogram>::iterator ii;
    for (ii = result.m_HistogramCollection.begin();
         ii != result.m_HistogramCollection.end(); ++ii) {
        transform(ii->begin(), ii->end(), ii->begin(), v + _1);
    }
    return result;
} // +

ImageProcessing::ip_HistogramTemplate operator*(
        const ImageProcessing::ip_HistogramTemplate& hp1,
        const ImageProcessing::ip_HistogramTemplate& hp2) {

    ImageProcessing::ip_HistogramTemplate result(hp1);

    std::vector<ImageProcessing::ip_HistogramTemplate::Histogram>::iterator ii;
    std::vector<ImageProcessing::ip_HistogramTemplate::Histogram>::
        const_iterator jj;

   for (ii = result.m_HistogramCollection.begin(),
        jj = hp2.m_HistogramCollection.begin();
        (ii != result.m_HistogramCollection.end()) &&
        (jj != hp2.m_HistogramCollection.end()); ++ii, ++jj) {
       transform(ii->begin(), ii->end(), jj->begin(), ii->begin(), _1 * _2);
   }

   return result;
} // *

ImageProcessing::ip_HistogramTemplate operator*(
        const ImageProcessing::ip_HistogramTemplate& ht,
        const OpenCvPlus::real& v) {
    ImageProcessing::ip_HistogramTemplate result(ht);
    std::vector<ImageProcessing::ip_HistogramTemplate::Histogram>::iterator ii;
    for (ii = result.m_HistogramCollection.begin();
         ii != result.m_HistogramCollection.end(); ++ii) {
        transform(ii->begin(), ii->end(), ii->begin(), _1 * v);
    }
    return result;
} // +

ImageProcessing::ip_HistogramTemplate operator*(
        const OpenCvPlus::real& v,
        const ImageProcessing::ip_HistogramTemplate& ht) {
    ImageProcessing::ip_HistogramTemplate result(ht);
    std::vector<ImageProcessing::ip_HistogramTemplate::Histogram>::iterator ii;
    for (ii = result.m_HistogramCollection.begin();
         ii != result.m_HistogramCollection.end(); ++ii) {
        transform(ii->begin(), ii->end(), ii->begin(), v * _1);
    }
    return result;
} // +

ImageProcessing::ip_HistogramTemplate operator/(
        const ImageProcessing::ip_HistogramTemplate& hp1,
        const ImageProcessing::ip_HistogramTemplate& hp2) {

    ImageProcessing::ip_HistogramTemplate result(hp1);

    std::vector<ImageProcessing::ip_HistogramTemplate::Histogram>::iterator ii;
    std::vector<ImageProcessing::ip_HistogramTemplate::Histogram>::
        const_iterator jj;

   for (ii = result.m_HistogramCollection.begin(),
        jj = hp2.m_HistogramCollection.begin();
        (ii != result.m_HistogramCollection.end()) &&
        (jj != hp2.m_HistogramCollection.end()); ++ii, ++jj) {
       transform(ii->begin(), ii->end(), jj->begin(), ii->begin(), _1 / _2);
   }

   return result;
} // /

ImageProcessing::ip_HistogramTemplate operator/(
        const ImageProcessing::ip_HistogramTemplate& ht,
        const OpenCvPlus::real& v) {
    ImageProcessing::ip_HistogramTemplate result(ht);
    std::vector<ImageProcessing::ip_HistogramTemplate::Histogram>::iterator ii;
    for (ii = result.m_HistogramCollection.begin();
         ii != result.m_HistogramCollection.end(); ++ii) {
        transform(ii->begin(), ii->end(), ii->begin(), _1 / v);
    }
    return result;
} // +

ImageProcessing::ip_HistogramTemplate operator/(
        const OpenCvPlus::real& v,
        const ImageProcessing::ip_HistogramTemplate& ht) {
    ImageProcessing::ip_HistogramTemplate result(ht);
    std::vector<ImageProcessing::ip_HistogramTemplate::Histogram>::iterator ii;
    for (ii = result.m_HistogramCollection.begin();
         ii != result.m_HistogramCollection.end(); ++ii) {
        transform(ii->begin(), ii->end(), ii->begin(), v / _1);
    }
    return result;
} // +

bool operator==(const ImageProcessing::ip_HistogramTemplate& hp1,
        const ImageProcessing::ip_HistogramTemplate& hp2) {

    std::vector<ImageProcessing::ip_HistogramTemplate::Histogram>::
        const_iterator ii;
    std::vector<ImageProcessing::ip_HistogramTemplate::Histogram>::
        const_iterator jj;

    bool result = true;

    for (ii = hp1.m_HistogramCollection.begin(),
        jj = hp2.m_HistogramCollection.begin();
        (ii != hp1.m_HistogramCollection.end()) &&
        (jj != hp2.m_HistogramCollection.end()); ++ii, ++jj) {
        result = inner_product(ii->begin(), ii->end(), jj->begin(), result,
                _1 && _2, _1 == _2);
        if (!result) {
            return false;
        }
    }

    return result;
} // ==
