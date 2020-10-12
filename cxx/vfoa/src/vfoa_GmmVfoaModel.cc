// Copyright (c) 2011-2020 Idiap Research Institute
//
// vfoa_GmmVfoaModel - model for visual focus of attention (VFOA) computation
//                     based on trained GMM model
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//          Elisa Ricci    (elisa.ricci@idiap.ch)
//
// See COPYING file for the complete license text.

// SYSTEM INCLUDES
#include <limits>                         // standard numerical type limits
#include <fstream>                        // file streams
#include <algorithm>                      // container algorithms
#include <boost/archive/xml_iarchive.hpp> // boost serialization, XML input
#include <boost/archive/xml_oarchive.hpp> // boost serialization, XML output
#include <boost/lambda/lambda.hpp>        // boost lambda expressions
#include <boost/math/distributions/normal.hpp> // normal distribution

// LOCAL INCLUDES
#include <vfoa/vfoa_GmmVfoaModel.h>          // vfoa_GmmVfoaModel declarations
#include <vfoa/vfoa_Exceptions.h>            // VFOA exceptions

using namespace std;
using namespace VFOA;
using namespace OpenCvPlus;

namespace VFOA {

typedef cvp_HeadPoseDiscreteDomain::HeadPose HeadPose;

// pan and tilt support set definitions as intervals from min to max
static const VFOA::real PAN_ANGLE_SUPPORT_SIZE =
    cvp_HeadPoseDiscreteDomain::MAX_HEAD_POSE.pan() -
    cvp_HeadPoseDiscreteDomain::MIN_HEAD_POSE.pan();
static const VFOA::real TILT_ANGLE_SUPPORT_SIZE =
    cvp_HeadPoseDiscreteDomain::MAX_HEAD_POSE.tilt() -
    cvp_HeadPoseDiscreteDomain::MIN_HEAD_POSE.tilt();

static const VFOA::real VFOA_UNFOCUSED_THRESHOLD_HELLINGER = 64;

/////////////////////////////// PUBLIC ///////////////////////////////////////

//===================== INNER CLASS: ObjectInfo ==============================

const vfoa_GmmVfoaModel::ObjectInfo
VFOA::vfoa_GmmVfoaModel::ObjectInfo::UNFOCUSED =
        ObjectInfo::create("Unfocused", 0, 0, 0, 0);

vfoa_GmmVfoaModel::ObjectInfo
VFOA::vfoa_GmmVfoaModel::ObjectInfo::create(const std::string& name,
    real pan, real tilt, real stdpan, real stdtilt) {
    ObjectId id = generateObjectId();
    ObjectInfo obj_info(id, name, pan, tilt, stdpan, stdtilt);
    return obj_info;
} // create

template<class Archive>
void VFOA::vfoa_GmmVfoaModel::ObjectInfo::serialize(Archive & ar,
        const unsigned int ver) {
    ar & boost::serialization::make_nvp("name", mName)
       & boost::serialization::make_nvp("mean_pan", mMeanPan)
       & boost::serialization::make_nvp("mean_tilt", mMeanTilt)
       & boost::serialization::make_nvp("sdev_pan", mStddevPan)
       & boost::serialization::make_nvp("sdev_tilt", mStddevTilt)
       & boost::serialization::make_nvp("id", mId);
} // serialize

vfoa_GmmVfoaModel::ObjectId
VFOA::vfoa_GmmVfoaModel::ObjectInfo::generateObjectId() {
    static ObjectId mMaxObjectId = 0;
    if (std::numeric_limits<vfoa_GmmVfoaModel::ObjectId>::max() ==
        mMaxObjectId) {
        throw vfoa_Exception("Maximum number of target objects reached!");
    }
    return mMaxObjectId++;
} // generateObjectId

//============================= OPERATIONS ===================================

vfoa_GmmVfoaModel::ObjectInfo
VFOA::vfoa_GmmVfoaModel::compute_vfoa_ML(const HeadPose& head_pose) {

    using boost::math::normal;

    if (objects_number() == 0) {
        return vfoa_GmmVfoaModel::ObjectInfo::UNFOCUSED;
    }

    list<ObjectInfo> obj_infos = objects();

    // evaluate likelihood of the first object
    list<ObjectInfo>::iterator ii = obj_infos.begin();
    normal distr_pan(ii->mMeanPan, ii->mStddevPan);
    normal distr_tilt(ii->mMeanTilt, ii->mStddevTilt);
    VFOA::real max_lhood = pdf(distr_pan, head_pose.pan()) *
        pdf(distr_tilt, head_pose.tilt());
    ObjectInfo ml_obj_info = *ii;
    ++ii;

    // iterate over all objects and choose the one with max likelihood
    real lhood;
    while (ii != obj_infos.end()) {
        lhood = pdf(normal(ii->mMeanPan, ii->mStddevPan), head_pose.pan()) *
            pdf(normal(ii->mMeanTilt, ii->mStddevTilt), head_pose.tilt());
        if (lhood > max_lhood) {
            ml_obj_info = *ii;
            max_lhood = lhood;
        }
        ++ii;
    }

    // compare the max likelihood so far to the uniform distribution likelihood
    lhood = 1.0 / (PAN_ANGLE_SUPPORT_SIZE * TILT_ANGLE_SUPPORT_SIZE);
    if (lhood > max_lhood) {
        return vfoa_GmmVfoaModel::ObjectInfo::UNFOCUSED;
    } else {
        return ml_obj_info;
    }

} // compute_vfoa_ML

VFOA::real hellinger_score(const vector<VFOA::real>& distribution1,
        const vector<VFOA::real>& distribution2) {

    VFOA::real score = 0;
    size_t distr_size = distribution1.size();
    for (size_t i = 0; i < distr_size; ++i) {
        score += sqrt(distribution1[i] * distribution2[i]);
    }
    return 1 - score;

} // hellinger score

vfoa_GmmVfoaModel::ObjectInfo
VFOA::vfoa_GmmVfoaModel::compute_vfoa_DD_Hellinger(
        const vector<VFOA::real>& distribution) {

    using boost::math::normal;

    if (objects_number() == 0) {
        return vfoa_GmmVfoaModel::ObjectInfo::UNFOCUSED;
    }

    list<ObjectInfo> obj_infos = objects();

    // evaluate score (Hellinger distance) of the first object
    // H^2(P,Q) = \frac{1}{2} \int \left(
    //     \sqrt{\frac{dP}{d\lambda}} - \sqrt{\frac{dQ}{d\lambda}}
    //     \right)^2 d\lambda
    list<ObjectInfo>::iterator ii = obj_infos.begin();
    const vector<real>& obj_distribution =
            mObjectHeadPoseDistributionPool[ii->id()];
    real min_score = hellinger_score(distribution, obj_distribution);
    ObjectInfo best_obj_info = *ii;
    ++ii;

    // iterate over all objects and choose the one with the best score
    real score(0);
    while (ii != obj_infos.end()) {
        const vector<real>& obj_distribution =
                mObjectHeadPoseDistributionPool[ii->id()];
        score = hellinger_score(distribution, obj_distribution);
        if (score < min_score) {
            best_obj_info = *ii;
            min_score = score;
        }
        ++ii;
    }

    // apply threshold for unfocused targets
    if (score > VFOA_UNFOCUSED_THRESHOLD_HELLINGER) {
        return vfoa_GmmVfoaModel::ObjectInfo::UNFOCUSED;
    } else {
        return best_obj_info;
    }

} // compute_vfoa_DD_Hellinger

void VFOA::vfoa_GmmVfoaModel::add_object(const ObjectInfo& obj_info) {
    typedef pair<vfoa_GmmVfoaModel::ObjectId,
                 vfoa_GmmVfoaModel::ObjectInfo> ObjInfoPair;
    ObjectInfo obj_info_noconst = obj_info;
    mObjectPool.insert(ObjInfoPair(obj_info.id(), obj_info_noconst));

    vector<real> vfoa_distribution = compute_vfoa_distribution(obj_info);
    typedef pair<vfoa_GmmVfoaModel::ObjectId, vector<real> > ObjDistrPair;
    mObjectHeadPoseDistributionPool.insert(ObjDistrPair(obj_info.id(),
            vfoa_distribution));
} // add_object

void VFOA::vfoa_GmmVfoaModel::remove_object(const ObjectInfo& obj_info) {
    mObjectPool.erase(obj_info.id());
} // remove_object

size_t VFOA::vfoa_GmmVfoaModel::objects_number() const {
    return mObjectPool.size();
} // objects_number

list<VFOA::vfoa_GmmVfoaModel::ObjectInfo>
VFOA::vfoa_GmmVfoaModel::objects() const {
    list<VFOA::vfoa_GmmVfoaModel::ObjectInfo> result;
    transform(mObjectPool.begin(), mObjectPool.end(), back_inserter(result),
        select2nd<std::map<ObjectId, ObjectInfo>::value_type>());
    return result;
} // objects

/////////////////////////////// PRIVATE //////////////////////////////////////

vector<VFOA::real>
VFOA::vfoa_GmmVfoaModel::compute_vfoa_distribution(const ObjectInfo& obj_info) {

    using boost::math::normal;
    using namespace boost::lambda;

    // evaluate likelihood of the first object
    normal distr_pan(obj_info.mMeanPan, obj_info.mStddevPan);
    normal distr_tilt(obj_info.mMeanTilt, obj_info.mStddevTilt);

    // iterate over all head poses and compute likelihoods
    const vector<HeadPose>& head_poses = mHeadPoseDiscreteDomain->values();
    vector<real> lhoods(head_poses.size());
    real sum_lhoods = 0;
    HeadPose head_pose;
    for (size_t head_pose_idx = 0; head_pose_idx < head_poses.size();
            ++head_pose_idx) {
        head_pose = head_poses[head_pose_idx];
        lhoods[head_pose_idx] = pdf(distr_pan, head_pose.pan()) *
            pdf(distr_tilt, head_pose.tilt());
        sum_lhoods += lhoods[head_pose_idx];
    }

    // normalize likelihoods
    transform(lhoods.begin(), lhoods.end(), lhoods.begin(), _1 / sum_lhoods);
    return lhoods;

}

/////////////////////////////// GLOBAL ///////////////////////////////////////

void
save_vfoa_model(const vfoa_GmmVfoaModel& model, const string& filename) {

    using namespace boost::serialization;

    // prepare the output XML archive to serialize the model
    ofstream ofs(filename.c_str());
    if (!ofs) {
        throw vfoa_Exception("Cannot open output file " + filename);
    }
    boost::archive::xml_oarchive output_xml_archive(ofs);

    typedef list<vfoa_GmmVfoaModel::ObjectInfo> ObjInfoList;

    // save all the objects in the model to the archive
    ObjInfoList objects = model.objects();
    ObjInfoList::size_type num_objects = objects.size();
    output_xml_archive & make_nvp("size", num_objects);
    for (ObjInfoList::const_iterator ii = objects.begin(); ii != objects.end();
            ++ii) {
        output_xml_archive & make_nvp("object", *ii);
    }
} // save_vfoamodel

void
restore_vfoa_model(vfoa_GmmVfoaModel& model, const string& filename) {

    using namespace boost::serialization;

    // prepare the input XML archive to deserialize the model
    ifstream ifs(filename.c_str());
    if (!ifs) {
        throw vfoa_Exception("Cannot open input file " + filename);
    }

    boost::archive::xml_iarchive input_xml_archive(ifs);

    typedef list<vfoa_GmmVfoaModel::ObjectInfo> ObjInfoList;

    vfoa_GmmVfoaModel::ObjectInfo obj_info =
        vfoa_GmmVfoaModel::ObjectInfo::UNFOCUSED;

    // read the number of objects and restore them into the model
    ObjInfoList::size_type map_size;
    input_xml_archive & make_nvp("size", map_size);
    for (ObjInfoList::size_type ii = 0; ii < map_size; ++ii) {
        input_xml_archive & make_nvp("object", obj_info);
        model.add_object(obj_info);
    }
} // restore_vfoamodel

} // namespace VFOA
