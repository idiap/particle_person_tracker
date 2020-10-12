/**
 * @file cxx/opencvplus/opencvplus/cvp_PaletteBundle.h
 * @date 03 February 2011
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Representation of a collection of colour palettes
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __CVP_PALETTEBUNDLE_H__
#define __CVP_PALETTEBUNDLE_H__

// SYSTEM INCLUDES
#include <vector>
#include <cmath>

// LOCAL INCLUDES
#include <opencvplus/cvp_Palette.h>

namespace OpenCvPlus {

/// @brief Storage class that stores palettes representing colour models.
///
/// This class is used to store and operate palettes of different colour
/// models. A palette can be a discrete distribution in a colour space
/// (RGB, YPbPr or others), or a continuous distribution represented by
/// a number of parameters.
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    13.09.2012

class cvp_PaletteBundle {

    public:

    // LIFECYCLE

    /// Copy constructor to create a palette bundle based on the existing one.
    /// @param palette_bundle Palette bundle to construct a copy from
    cvp_PaletteBundle(const cvp_PaletteBundle& palette_bundle);

    /// Destructor, releases allocated palettes
    ~cvp_PaletteBundle();

    // OPERATIONS

    /// Factory method to create palette bundles from a set of files
    /// containing individual palettes.
    /// @param palette_file_names Names of files containing individual palettes
    /// @return Palette bundle composed of specified palettes
    static cvp_PaletteBundle* load_from_files(
            const std::vector<std::string>& palette_file_names);

    /// Saves palettes from the current bundle to a set of files
    /// containing individual palettes.
    /// @param palette_fnames Names of files to save individual palettes to
    void save_to_files(const std::vector<std::string>& palette_fnames) const;

    /// Gets a palette by its index (read only access),
    /// the index must be the same as the index of the corresponding file name
    /// passed to the constructor of the bundle.
    /// Throws exception if the palette index is invalid.
    /// @param palette_index Index of a palette to retrieve
    /// @return Pointer to the palette
    const cvp_Palette* palette(unsigned palette_index) const {
        return m_Palettes[palette_index];
    }

    /// Gets a palette by its index (read-write access),
    /// the index must be the same as the index of the corresponding file name
    /// passed to the constructor of the bundle.
    /// Throws exception if the palette index is invalid.
    /// @param palette_index Index of a palette to retrieve
    /// @return Pointer to the palette
    cvp_Palette* palette(unsigned palette_index) {
        return m_Palettes[palette_index];
    }

    /// Number of palettes in the bundle.
    /// @return Number of palettes in the bundle.
    unsigned size() const {
        return m_Palettes.size();
    }

    // OPERATORS

    /// Assignment operator.
    ///
    /// @param rhs Palette bundle to assign
    /// @return Reference to self with modified contents.
    cvp_PaletteBundle& operator=(const cvp_PaletteBundle& rhs);

    private:

    /// Constructor to create a palette bundle based on a set of palettes.
    /// @param palettes Set of palettes to construct a copy from
    cvp_PaletteBundle(const std::vector<cvp_Palette*>& palettes);

    std::vector<cvp_Palette*> m_Palettes;

}; // class PaletteBundle

} // namespace OpenCvPlus

#endif // __CVP_PALETTEBUNDLE_H__
