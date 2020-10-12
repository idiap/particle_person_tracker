// Copyright (c) 2011-2020 Idiap Research Institute
//
// vfoa_CognitiveVfoaModel - model to compute visual focus of attention (VFOA)
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// LOCAL INCLUDES
#include <vfoa/vfoa_CognitiveVfoaModel.h>               // declaration of this

// SYSTEM INCLUDES
#include <boost/math/distributions/normal.hpp>          // normal distribution
#include <boost/foreach.hpp>                            // foreach loop
#include <sstream>                                      // STL string stream

using namespace std;
using namespace BICV;
using namespace OpenCvPlus;
using namespace BayesFilter;

///////////////////////////// LOCAL CONSTANTS ////////////////////////////////

static float initialize_unfocuse_gaussian_threshold(float distance);

static const string TRACKED_OBJECT_NAME_PREFIX = "Partner";
static const string UNFOCUSED_OBJECT_NAME = "UN";
static const string CAMERA_OBJECT_NAME = "Nao";
static const unsigned UNFOCUSED_OBJECT_ID = 0;
static const unsigned CAMERA_OBJECT_ID = 1;
static const float EYE_GAZE_RELAXATION_COEFF = 0.95;
static const float PAN_STDDEV = 50;
static const float TILT_STDDEV = 50;
static const float ROLL_STDDEV = 10;
static const float UNFOCUSED_GAUSIAN_THRESHOLD_DISTANCE = 10;
static const float UNFOCUSED_GAUSIAN_THRESHOLD =
        initialize_unfocuse_gaussian_threshold(
                UNFOCUSED_GAUSIAN_THRESHOLD_DISTANCE);
static const float UNFOCUSED_HELLINGER_THRESHOLD = .8;
static const float FOV_AZIMUTH_ANGLE = 30;
static const float LOOK_LEFT_ANGLE = -50;
static const float LOOK_RIGHT_ANGLE = 50;
static const float LOOK_DOWN_ANGLE = -15;
static const float LOOK_UP_ANGLE = 15;

namespace VFOA {

typedef cvp_HeadPoseDiscreteDomain::HeadPose HeadPose;

/////////////////////////////// PUBLIC ///////////////////////////////////////

vfoa_CognitiveVfoaModel::vfoa_CognitiveVfoaModel(
        const cvp_HeadPoseDiscreteDomain& head_pose_domain,
        int image_width, int image_height) :
        mImageWidth(image_width),
        mImageHeight(image_height),
        mHeadPoseDomain(head_pose_domain) {

    vfoa_CognitiveVfoaModelObjectInfo obj_info;

    // add the "unfocused" target
    obj_info.mName = UNFOCUSED_OBJECT_NAME;
    obj_info.mTracker = 0;
    obj_info.mId = UNFOCUSED_OBJECT_ID;
    mObjects.push_back(obj_info);

    // add the "camera" target
    obj_info.mName = CAMERA_OBJECT_NAME;
    obj_info.mTracker = 0;
    obj_info.mId = CAMERA_OBJECT_ID;
    mObjects.push_back(obj_info);

    mObjectsCount = CAMERA_OBJECT_ID + 1;

    // KLUDGE: hardcoded papers object!
//    obj_info.mName = "Papers";
//    obj_info.mTracker = 0;
//    obj_info.mId = mObjectsCount++;
//    obj_info.mParameters.m_TranslationX = 120;
//    obj_info.mParameters.m_TranslationY = 0;
//    obj_info.mParameters.m_Scale = 0.25;
//    obj_info.mParameters.m_Excentricity = 0.2;
//    mObjects.push_back(obj_info);

//    mObjectsCount = CAMERA_OBJECT_ID + 2;
} // vfoa_CognitiveVfoaModel

/* static */ void
vfoa_CognitiveVfoaModel::add_detected_object(bicv_HeadPoseTracker * tracker) {

    // check whether tracker is already added (compare pointers)
    bool exists_already = false;
    BOOST_FOREACH(const vfoa_CognitiveVfoaModelObjectInfo& oi, mObjects) {
        if (oi.mTracker == tracker) {
            exists_already = true;
            break;
        }
    }

    // add if the tracker was not registered before
    if (!exists_already) {
        static unsigned tracked_objects_count = 1;
        vfoa_CognitiveVfoaModelObjectInfo obj_info;
        obj_info.mName = TRACKED_OBJECT_NAME_PREFIX;
        obj_info.mTracker = tracker;
        obj_info.mId = mObjectsCount++;
        obj_info.mNumber = tracked_objects_count++;
        mObjects.push_back(obj_info);
    }
} // add_detected_object

/* static */ void
vfoa_CognitiveVfoaModel::remove_detected_object(bicv_HeadPoseTracker * tracker) {
    list<vfoa_CognitiveVfoaModelObjectInfo>::iterator ii = mObjects.begin();
    list<vfoa_CognitiveVfoaModelObjectInfo>::iterator last = mObjects.end();
    for (; ii != last; ++ii) {
        if (ii->mTracker == tracker) {
            break;
        }
    }

    if (ii != last) {
        mObjects.erase(ii);
    }

} // remove_detected_object

/* static */ void
vfoa_CognitiveVfoaModel::add_predefined_objects(const string& filename) {
    mObjectsCount++;
} // add_predefined_objects

/* static */ void
vfoa_CognitiveVfoaModel::remove_predefined_objects() {
} // remove_predefined_objects

/* static */ const vfoa_CognitiveVfoaModelObjectInfo&
vfoa_CognitiveVfoaModel::object_info(
        BICV::bicv_HeadPoseTracker * tracker) const {
    BOOST_FOREACH(const vfoa_CognitiveVfoaModelObjectInfo& obj_info, mObjects) {
        if (obj_info.mTracker == tracker) {
            return obj_info;
        }
    }
    return mObjects.front();
} // object_info

VfoaDistribution
vfoa_CognitiveVfoaModel::compute_vfoa_distribution_Gaussian(
        bicv_HeadPoseTracker * tracker) const {
    return compute_vfoa_distribution(tracker,
            &vfoa_CognitiveVfoaModel::compute_probability_Gaussian);
} // compute_vfoa_distribution_Gaussian

VfoaDistribution
vfoa_CognitiveVfoaModel::compute_vfoa_distribution_Hellinger(
        BICV::bicv_HeadPoseTracker * tracker) const {
    return compute_vfoa_distribution(tracker,
            &vfoa_CognitiveVfoaModel::compute_probability_Hellinger);
} // compute_vfoa_distribution_Hellinger

/////////////////////////////// PRIVATE //////////////////////////////////////

VfoaDistribution
vfoa_CognitiveVfoaModel::compute_vfoa_distribution(
        bicv_HeadPoseTracker * tracker,
        float (vfoa_CognitiveVfoaModel::* pfun)(
                bicv_HeadPoseTracker * person,
                const vfoa_CognitiveVfoaModelObjectInfo& addressee,
                const HeadPose& ref_head_pose) const) const {

    // compute reference head pose
    HeadPose ref_head_pose;
    float sum_pan = 0;
    unsigned num_pan = 0;
    bicv_HeadPoseParameters hpparams = mean(tracker->particle_distribution()).
            m_HeadPoseParamsCur;
    const float tracker_tx = hpparams.m_TranslationX;
    const float tracker_ty = hpparams.m_TranslationY;

    // camera object
    sum_pan += (mImageWidth / 2 - tracker_tx) * FOV_AZIMUTH_ANGLE / mImageWidth;
    num_pan++;

    BOOST_FOREACH(const vfoa_CognitiveVfoaModelObjectInfo& obj_info, mObjects) {
        // for everything that has a tracker and is not self, accumulate pan
        if (obj_info.mTracker) {
            if (obj_info.mTracker != tracker) {
                sum_pan +=
                    (tracker_tx > mean(obj_info.mTracker->particle_distribution()).
                    m_HeadPoseParamsCur.m_TranslationX) ?
                    LOOK_LEFT_ANGLE : LOOK_RIGHT_ANGLE;
                num_pan++;
            }
        } else if ((obj_info.mId != CAMERA_OBJECT_ID) &&
                (obj_info.mId != UNFOCUSED_OBJECT_ID)) {
            sum_pan += max( LOOK_LEFT_ANGLE, min( LOOK_RIGHT_ANGLE,
                static_cast<float>(
                cvp_arctan((obj_info.mParameters.m_TranslationX - tracker_tx) /
                fabs(obj_info.mParameters.m_TranslationY - tracker_ty)) *
                90.0 / 100.0)));
            num_pan++;
        }
    }

    if (num_pan > 0) {
        ref_head_pose.pan(sum_pan / num_pan);
    }

//    cout << "Tracker " << tracker_tx << ", refpose " << ref_head_pose << endl;

    // compute probabilities for every target (except self)
    float sum_weights = 0;
    list<VfoaDistribution::element_type> weighted_targets;
    BOOST_FOREACH(const vfoa_CognitiveVfoaModelObjectInfo& obj_info, mObjects) {
        if (obj_info.mTracker != tracker) {
            VfoaDistribution::element_type weighted_target;
            weighted_target.first = obj_info;
            weighted_target.second = (*this.*pfun)(tracker, obj_info,
                    ref_head_pose);
            sum_weights += weighted_target.second;
            weighted_targets.push_back(weighted_target);
        }
    }

    // normalize weights
    assert(sum_weights > 0);
    BOOST_FOREACH(VfoaDistribution::element_type& elt, weighted_targets) {
        elt.second /= sum_weights;
    }

    // construct distribution
    vector<VfoaDistribution::element_type> distr_elts;
    distr_elts.assign(weighted_targets.begin(), weighted_targets.end());

//    cout << VfoaDistribution(distr_elts) << endl;
    return VfoaDistribution(distr_elts);
} // compute_vfoa_distribution

float
vfoa_CognitiveVfoaModel::compute_probability_Gaussian(
        bicv_HeadPoseTracker * person,
        const vfoa_CognitiveVfoaModelObjectInfo& addressee,
        const HeadPose& ref_head_pose) const {

    using namespace boost::math;

    if (!addressee.mTracker) {
        if (UNFOCUSED_OBJECT_ID == addressee.mId) {
            return UNFOCUSED_GAUSIAN_THRESHOLD;
        } else if (CAMERA_OBJECT_ID == addressee.mId) {
            HeadPose mean_head_pose;
            bicv_HeadPoseParameters head_pose_params =
                mean(person->particle_distribution()).m_HeadPoseParamsCur;
            //float tracker_tx = head_pose_params.m_TranslationX;
//            mean_head_pose.pan(EYE_GAZE_RELAXATION_COEFF *
//                    (mImageWidth / 2 - tracker_tx) * FOV_AZIMUTH_ANGLE / mImageWidth +
//                    (1 - EYE_GAZE_RELAXATION_COEFF) * ref_head_pose.pan());
            mean_head_pose.pan(0);
            mean_head_pose.roll(0);
            mean_head_pose.tilt(0);

//            cout << "Tracker " << tracker_tx <<
//                    ", head pose " << head_pose_params.m_HeadPose <<
//                    ", Nao , mean " << mean_head_pose << endl;

            HeadPose head_pose(mean(person->particle_distribution()).
                    m_HeadPoseParamsCur.m_HeadPose);
            boost::math::normal_distribution<float> normal_pan_dist(
                    mean_head_pose.pan(), PAN_STDDEV);
            boost::math::normal_distribution<float> normal_tilt_dist(
                    mean_head_pose.tilt(), TILT_STDDEV);
            boost::math::normal_distribution<float> normal_roll_dist(
                    mean_head_pose.roll(), ROLL_STDDEV);

            return pdf(normal_pan_dist,  head_pose.pan()) *
                   pdf(normal_tilt_dist, head_pose.tilt()) *
                   pdf(normal_roll_dist, head_pose.roll());
        } else {
            // some statically defined object

            bicv_HeadPoseParameters head_pose_params =
                mean(person->particle_distribution()).m_HeadPoseParamsCur;
            HeadPose head_pose(head_pose_params.m_HeadPose);
            HeadPose mean_head_pose;

            mean_head_pose.pan( max( LOOK_LEFT_ANGLE, min( LOOK_RIGHT_ANGLE,
                    static_cast<float>(
                    cvp_arctan(0.2 * (addressee.mParameters.m_TranslationX -
                    head_pose_params.m_TranslationX) /
                    max(fabs(addressee.mParameters.m_TranslationY -
                    head_pose_params.m_TranslationY), 1.0f))) * 90.0f / 100.0f)));
            mean_head_pose.tilt( max( LOOK_DOWN_ANGLE, min( LOOK_UP_ANGLE,
                    static_cast<float>(
                    cvp_arctan(0.2 * (addressee.mParameters.m_TranslationY -
                    head_pose_params.m_TranslationY) /
                    max(fabs(addressee.mParameters.m_TranslationX -
                    head_pose_params.m_TranslationX), 1.0f))) * 90.0f / 100.0f)));
            mean_head_pose.roll(0);
            mean_head_pose.pan(
                    EYE_GAZE_RELAXATION_COEFF * mean_head_pose.pan() +
                    (1 - EYE_GAZE_RELAXATION_COEFF) * ref_head_pose.pan());

//            cout << "Tracker " << head_pose_params.m_TranslationX <<
//                    ", head pose " << head_pose <<
//                    ", object " << addressee.mName <<
//                    ", mean " << mean_head_pose << endl;

            boost::math::normal_distribution<float> normal_pan_dist(
                    mean_head_pose.pan(), PAN_STDDEV);
            boost::math::normal_distribution<float> normal_tilt_dist(
                    mean_head_pose.tilt(), TILT_STDDEV);
            boost::math::normal_distribution<float> normal_roll_dist(
                    mean_head_pose.roll(), ROLL_STDDEV);

            return pdf(normal_pan_dist,  -head_pose.pan()) *
                   pdf(normal_tilt_dist, head_pose.tilt()) *
                   pdf(normal_roll_dist, head_pose.roll());
        }
        return UNFOCUSED_GAUSIAN_THRESHOLD;
    } else {
        bicv_HeadPoseParameters head_pose_params =
            mean(person->particle_distribution()).m_HeadPoseParamsCur;
        HeadPose head_pose(head_pose_params.m_HeadPose);
        HeadPose mean_head_pose;

        bicv_HeadPoseParameters addr_head_pose_params = mean(
            addressee.mTracker->particle_distribution()).m_HeadPoseParamsCur;

        mean_head_pose.pan((addr_head_pose_params.m_TranslationX <
                head_pose_params.m_TranslationX) ? LOOK_LEFT_ANGLE :
                LOOK_RIGHT_ANGLE);
        mean_head_pose.tilt(cvp_arctan(
            (addr_head_pose_params.m_TranslationY -
                  head_pose_params.m_TranslationY) /
            fabs(addr_head_pose_params.m_TranslationX -
                  head_pose_params.m_TranslationX)) * 90.0 / 100.0);
        mean_head_pose.pan(
                EYE_GAZE_RELAXATION_COEFF * mean_head_pose.pan() +
                (1 - EYE_GAZE_RELAXATION_COEFF) * ref_head_pose.pan());
        mean_head_pose.roll(0);

//        cout << "Tracker " << head_pose_params.m_TranslationX <<
//                ", head pose " << head_pose_params.m_HeadPose <<
//                ", addressee " << addr_head_pose_params.m_TranslationX <<
//                ", mean " << mean_head_pose << endl;

        boost::math::normal_distribution<float> normal_pan_dist(
                mean_head_pose.pan(), PAN_STDDEV);
        boost::math::normal_distribution<float> normal_tilt_dist(
                mean_head_pose.tilt(), TILT_STDDEV);
        boost::math::normal_distribution<float> normal_roll_dist(
                mean_head_pose.roll(), ROLL_STDDEV);

        return pdf(normal_pan_dist,  -head_pose.pan()) *
               pdf(normal_tilt_dist, head_pose.tilt()) *
               pdf(normal_roll_dist, head_pose.roll());
    }
} // compute_probability_Gaussian

float
vfoa_CognitiveVfoaModel::compute_probability_Hellinger(
        bicv_HeadPoseTracker * person,
        const vfoa_CognitiveVfoaModelObjectInfo& addressee,
        const HeadPose& ref_head_pose) const {
    using namespace boost::math;

    HeadPose mean_head_pose;

    if (!addressee.mTracker) {
        if (UNFOCUSED_OBJECT_ID == addressee.mId) {
            return UNFOCUSED_HELLINGER_THRESHOLD;
        } else if (CAMERA_OBJECT_ID == addressee.mId) {
            bicv_HeadPoseParameters head_pose_params =
                mean(person->particle_distribution()).m_HeadPoseParamsCur;
            //float tracker_tx = head_pose_params.m_TranslationX;
            // correct the head pose when looking at camera
            // with reference direction
//            mean_head_pose.pan(EYE_GAZE_RELAXATION_COEFF *
//                    (mImageWidth / 2 - tracker_tx) * FOV_AZIMUTH_ANGLE / mImageWidth +
//                    (1 - EYE_GAZE_RELAXATION_COEFF) * ref_head_pose.pan());
            mean_head_pose.pan(0);
            mean_head_pose.tilt(0);
            mean_head_pose.roll(0);

//            cout << "Tracker " << tracker_tx <<
//                    ", head pose " << head_pose_params.m_HeadPose <<
//                    ", Nao , mean " << mean_head_pose << endl;
        } else {
            // some statically defined object
            bicv_HeadPoseParameters head_pose_params =
                mean(person->particle_distribution()).m_HeadPoseParamsCur;
            HeadPose head_pose(head_pose_params.m_HeadPose);

            mean_head_pose.pan( max( LOOK_LEFT_ANGLE, min( LOOK_RIGHT_ANGLE,
                    static_cast<float>(
                    cvp_arctan(0.2 * (addressee.mParameters.m_TranslationX -
                    head_pose_params.m_TranslationX) /
                    max(fabs(addressee.mParameters.m_TranslationY -
                    head_pose_params.m_TranslationY), 1.0f))) * 90.0f / 100.0f)));
            mean_head_pose.tilt( max( LOOK_DOWN_ANGLE, min( LOOK_UP_ANGLE,
                    static_cast<float>(
                    cvp_arctan(0.2 * (addressee.mParameters.m_TranslationY -
                    head_pose_params.m_TranslationY) /
                    max(fabs(addressee.mParameters.m_TranslationX -
                    head_pose_params.m_TranslationX), 1.0f))) * 90.0f / 100.0f)));
            mean_head_pose.roll(0);
            mean_head_pose.pan(
                    EYE_GAZE_RELAXATION_COEFF * mean_head_pose.pan() +
                    (1 - EYE_GAZE_RELAXATION_COEFF) * ref_head_pose.pan());

//            cout << "Tracker " << head_pose_params.m_TranslationX <<
//                    ", head pose " << head_pose <<
//                    ", object " << addressee.mName <<
//                    ", mean " << mean_head_pose << endl;
        }
    } else {
        bicv_HeadPoseParameters head_pose_params =
            mean(person->particle_distribution()).m_HeadPoseParamsCur;

        bicv_HeadPoseParameters addr_head_pose_params = mean(
            addressee.mTracker->particle_distribution()).m_HeadPoseParamsCur;

        mean_head_pose.pan((addr_head_pose_params.m_TranslationX <
                head_pose_params.m_TranslationX) ? LOOK_LEFT_ANGLE :
                LOOK_RIGHT_ANGLE);
//        mean_head_pose.tilt(cvp_arctan(
//            (addr_head_pose_params.m_TranslationY -
//                  head_pose_params.m_TranslationY) /
//            fabs(addr_head_pose_params.m_TranslationX -
//                  head_pose_params.m_TranslationX)) * 90.0 / 100.0);
        mean_head_pose.tilt(0);
        mean_head_pose.roll(0);
        mean_head_pose.pan(
                EYE_GAZE_RELAXATION_COEFF * mean_head_pose.pan() +
                (1 - EYE_GAZE_RELAXATION_COEFF) * ref_head_pose.pan());
//        cout << "Actor mean " << mean_head_pose << endl;
//        cout << "Tracker " << head_pose_params.m_TranslationX <<
//                ", head pose " << head_pose_params.m_HeadPose <<
//                ", addressee " << addr_head_pose_params.m_TranslationX <<
//                ", mean " << mean_head_pose << endl;
    }

    typedef bf_DiscreteDistribution<bicv_HeadPoseARTrackerState>
            ParticleDistribution;
    const ParticleDistribution& person_distr =
            person->particle_distribution();

    // evaluate particle filter distribution over head poses
    valarray<float> pf_head_pose_distr(mHeadPoseDomain.size());
    BOOST_FOREACH(const ParticleDistribution::element_type& elt,
            person_distr.elements()) {
        HeadPose hp = elt.first.m_HeadPoseParamsCur.m_HeadPose;
        pf_head_pose_distr[mHeadPoseDomain.id(
                mHeadPoseDomain.discretize(hp))] += elt.second;
    }
    pf_head_pose_distr /= pf_head_pose_distr.sum();

//    ostream_iterator<float> out_it (cout, " ");
//    copy(&pf_head_pose_distr[0], &pf_head_pose_distr[
//        pf_head_pose_distr.size()], out_it);
//    cout << endl;

    // prepare predicted 3D Gaussian distribution over head pose
//    cout << "model mean head pose " << mean_head_pose << endl;
    boost::math::normal_distribution<float> normal_pan_dist(
            mean_head_pose.pan(), PAN_STDDEV);
    boost::math::normal_distribution<float> normal_tilt_dist(
            mean_head_pose.tilt(), TILT_STDDEV);
    boost::math::normal_distribution<float> normal_roll_dist(
            mean_head_pose.roll(), ROLL_STDDEV);

    // evaluate predicted distribution over head poses
    // (uses mutual object locations and person's reference position)
    valarray<float> model_head_pose_distr(mHeadPoseDomain.size());
    for (unsigned i = 0; i < model_head_pose_distr.size(); ++i) {
        HeadPose hp = mHeadPoseDomain.value(i);
        model_head_pose_distr[i] =
                pdf(normal_pan_dist,  -hp.pan()) *
                pdf(normal_tilt_dist, hp.tilt()) *
                pdf(normal_roll_dist, hp.roll());
//        cout << "head pose " << hp << ", pdf=" << model_head_pose_distr[i] << endl;
    }
//    cout << "model distribution sum " << model_head_pose_distr.sum() << endl;
    model_head_pose_distr /= model_head_pose_distr.sum();
//    copy(&model_head_pose_distr[0], &model_head_pose_distr[
//        model_head_pose_distr.size()], out_it);
//    cout << endl;

    // compute Hellinger distance
    return exp(sqrt(pf_head_pose_distr * model_head_pose_distr).sum() - 1);

} // compute_probability_Hellinger

std::ostream& operator<<(std::ostream& out,
        const vfoa_CognitiveVfoaModelObjectInfo& obj) {
        out << '(' << obj.mId << ", " << obj.mName << ", "
            << obj.mNumber << ", ";
        out << obj.mParameters.m_HeadPose << ", "
            << obj.mTracker << ')' << endl;
        return out;
} // operator<<

} // namespace VFOA

//////////////////////////////// LOCAL ///////////////////////////////////////

float initialize_unfocuse_gaussian_threshold(float distance) {
    using namespace boost::math;
    boost::math::normal_distribution<float> normal_pan_dist(0, PAN_STDDEV);
    boost::math::normal_distribution<float> normal_tilt_dist(0, TILT_STDDEV);
    boost::math::normal_distribution<float> normal_roll_dist(0, ROLL_STDDEV);
    return pdf(normal_pan_dist,  distance) *
           pdf(normal_tilt_dist, distance) *
           pdf(normal_roll_dist, distance);
} // initialize_unfocuse_gaussian_threshold
