/**
 * @file cxx/opencvplus/opencvplus/cvp_PimModel.h
 * @date 25 May 2012
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Probability index map (PIM) model class
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __CVP_PIMMODEL_H__
#define __CVP_PIMMODEL_H__

// SYSTEM INCLUDES
#include <vector>                              // STL vector
#include <boost/foreach.hpp>                   // foreach loop

// LOCAL INCLUDES
#include <opencvplus/cvp_Exceptions.h>         // various exceptions
#include <opencvplus/cvp_PimFeature.h>         // PIM feature

namespace OpenCvPlus {

/// @brief Probability index map (PIM) model class.
///
/// This class represents probability index map (PIM) model. It contains
/// PIM feature data and various supplementary information (e.g. model name)
/// and procedures (save, load etc.)
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    20.12.2012

class cvp_PimModel {

public:

    /// Type for PIM feature
    typedef OpenCvPlus::cvp_PimFeature<float> PimFeatureType;

    // LIFECYCLE

    /// Copy constructor to create PIM model based on the existing one.
    /// @param other PIM model to construct a copy from
    cvp_PimModel(const cvp_PimModel& other);

    /// Destructor, releases the allocated PIM model
    ~cvp_PimModel();

    // OPERATIONS

    /// Factory method to load PIM model from file.
    /// @param filename PIM model file name
    /// @return Constructed PIM model
    static cvp_PimModel* load_from_file(const std::string& filename);

    /// Save PIM model to a file.
    /// @param filename Name of a file to save the model to
    void save_to_file(const std::string& filename) const;

    /// Returns PIM model data - PIM feature (read-only access).
    /// @return PIM model data
    const PimFeatureType& data() const {
        return *m_PimFeature;
    }

    /// Returns PIM model data (read-write access).
    /// @return PIM model data
    PimFeatureType& data() {
        return *m_PimFeature;
    }

    /// PIM model name (read-only access).
    /// @return PIM model name
    const std::string& name() const {
        return m_Name;
    }

    /// Associated face bounding box position (read-only access).
    /// @return Associated face bounding box position
    const CvRect& face_rect() const {
        return m_FaceRect;
    }

    // OPERATORS

    /// Assignment operator.
    ///
    /// @param rhs PIM model to assign
    /// @return Reference to self with modified contents.
    cvp_PimModel& operator=(const cvp_PimModel& rhs);

    private:

    /// Constructor to create PIM model using its name, associated PIM feature
    /// and face rectangle coordinates.
    /// @param name Name of the model
    /// @param pim_feature PIM feature associated with this model
    /// @param rect Average face location relative to the PIM feature
    cvp_PimModel(const std::string& name, PimFeatureType * pim_feature,
        const CvRect& face_rect);

    // This model's name
    std::string m_Name;
    // PIM feature associated with this model
    PimFeatureType * m_PimFeature;
    // Face coordinates relative to the PIM feature
    CvRect m_FaceRect;

}; // class cvp_PimModel

} // namespace OpenCvPlus

#endif // __CVP_PIMMODEL_H__
