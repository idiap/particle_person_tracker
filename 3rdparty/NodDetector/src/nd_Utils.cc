/**
 * @file src/nd_Utils.cc
 * @date 14 March 2013
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 * @author Laurent Nguyen <lnguyen@idiap.ch>
 *
 * @brief Utility methods for nod detector
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

// SYSTEM INCLUDES
#include <cassert>
#include <fstream>
#include <iterator>
#include <numeric>
#include <gsl/gsl_interp.h>

// LOCAL INCLUDES
#include <noddetector/nd_Utils.h>                  // declaration of this

using namespace std;

namespace NodDetector {

void interpolate(const vector<double>& ts, const vector<double>& values,
    const vector<double>& ts_new, vector<double>& values_new,
    nd_InterpolationType interpolation_type /* = ND_INTERP_LINEAR */) {

    const size_t input_array_size = ts.size();
    assert(input_array_size == values.size());
    assert(input_array_size > 0);
    const size_t output_array_size = ts_new.size();
    assert(output_array_size == values_new.size());
    assert(output_array_size > 0);

    const gsl_interp_type * git_inst;

    switch(interpolation_type) {
    case ND_INTERP_LINEAR:
        git_inst = gsl_interp_linear;
        break;
    case ND_INTERP_POLYNOMIAL:
        git_inst = gsl_interp_polynomial;
        break;
    case ND_INTERP_CSPLINE:
        git_inst = gsl_interp_cspline;
        break;
    }

    gsl_interp * gi = gsl_interp_alloc(git_inst, input_array_size);
    int res_init = gsl_interp_init (gi, &ts[0], &values[0], input_array_size);

    gsl_interp_accel * gi_accel = gsl_interp_accel_alloc();

    const double * tsnew_ptr = &ts_new[0];
    double * vnew_ptr = &values_new[0];
    double x, y;
    for (unsigned idx = 0; idx < output_array_size; ++idx) {
        x = *tsnew_ptr++;
        y = gsl_interp_eval(gi, &ts[0], &values[0], x, gi_accel);
        *vnew_ptr++ = y;
    }

    gsl_interp_accel_free (gi_accel);
    gsl_interp_free(gi);

} // interpolate

void interpolate_motion(double prev_ts,
    const vector<double>& ts, const vector<double>& values,
    const vector<double>& ts_new, vector<double>& values_new,
    nd_InterpolationType interpolation_type /* = ND_INTERP_LINEAR */ ) {

//#define INTERPOLATE_MOTION_TRACE

#ifdef INTERPOLATE_MOTION_TRACE
    ofstream ofs("/home/idiap/Documents/humavips_gar/bin/interpolate_motion_dump.txt");
    ostream_iterator<double> out_it(ofs, " ");
    ostream_iterator<long> long_out_it(ofs, " ");
    ofs << "Input data :" << endl;
    ofs << "Timestamps :";
    copy(ts.begin(), ts.end(), long_out_it);
    ofs << endl;
    ofs << "Values     :";
    copy(values.begin(), values.end(), out_it);
    ofs << endl;
    ofs << "Ts_new     :";
    copy(ts_new.begin(), ts_new.end(), long_out_it);
    ofs << endl;
    ofs << "Processing :" << endl;
#endif

    // point coordinates start from 0 and accumulate displacements
    vector<double> pt_coords(values.size() + 1, 0);
    partial_sum(values.begin(), values.end(), &pt_coords[1]);

#ifdef INTERPOLATE_MOTION_TRACE
    ofs << "    Pt_coords :";
    copy(pt_coords.begin(), pt_coords.end(), out_it);
    ofs << endl;
#endif

    // point timestamps are the same for 1..N,
    // for 0 we take ts_1 and substract average dt
    vector<double> pt_ts(values.size() + 1, 0);
    pt_ts[0] = prev_ts;
    copy(ts.begin(), ts.end(), &pt_ts[1]);

#ifdef INTERPOLATE_MOTION_TRACE
    ofs << "    Pt_ts     :";
    copy(pt_ts.begin(), pt_ts.end(), long_out_it);
    ofs << endl;
#endif

    // initialize new point timestamps
    vector<double> pt_ts_new(ts_new.size() + 1, 0);
    pt_ts_new[0] = pt_ts[0];
    copy(ts_new.begin(), ts_new.end(), &pt_ts_new[1]);

#ifdef INTERPOLATE_MOTION_TRACE
    ofs << "    Pt_ts_new :";
    copy(pt_ts_new.begin(), pt_ts_new.end(), long_out_it);
    ofs << endl;
#endif

    // interpolate points to new timestamps
    vector<double> pt_coords_new(pt_ts_new);
    interpolate(pt_ts, pt_coords, pt_ts_new, pt_coords_new, interpolation_type);

#ifdef INTERPOLATE_MOTION_TRACE
    ofs << "    Pt_coords_new :";
    copy(pt_coords_new.begin(), pt_coords_new.end(), out_it);
    ofs << endl;
#endif

    // convert point coordinates to motion vectors
    vector<double> mot_new(pt_coords_new);
    adjacent_difference(pt_coords_new.begin(), pt_coords_new.end(),
            mot_new.begin());
    copy(mot_new.begin() + 1, mot_new.end(), values_new.begin());

#ifdef INTERPOLATE_MOTION_TRACE
    ofs << "    Mot_new       :";
    copy(mot_new.begin(), mot_new.end(), out_it);
    ofs << endl;
#endif

} // interpolate_motion

} // namespace NodDetector
