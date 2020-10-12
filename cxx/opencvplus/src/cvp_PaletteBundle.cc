/**
 * @file cxx/opencvplus/src/cvp_PaletteBundle.cc
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

// SYSTEM INCLUDES
#include <cassert>                                        // assert
#include <fstream>                                        // STL file streams
#include <boost/foreach.hpp>                              // BOOST_FOREACH loop

// LOCAL INCLUDES
#include <opencvplus/cvp_PaletteBundle.h>              // declaration of this

using namespace std;

namespace OpenCvPlus {

/////////////////////////////// PUBLIC ///////////////////////////////////////

cvp_PaletteBundle::cvp_PaletteBundle(const cvp_PaletteBundle& palette_bundle) :
    m_Palettes(palette_bundle.size(), 0) {
    for (unsigned palette_index = 0; palette_index < palette_bundle.size();
            ++palette_index) {
        m_Palettes[palette_index] =
                new cvp_Palette(*palette_bundle.palette(palette_index));
    }
} // PaletteBundle

cvp_PaletteBundle::~cvp_PaletteBundle() {
    BOOST_FOREACH(cvp_Palette * palette, m_Palettes) {
        delete palette;
    }
} // ~PaletteBundle

/* static */ cvp_PaletteBundle*
cvp_PaletteBundle::load_from_files(
        const std::vector<std::string>& palette_infos) {
    vector<cvp_Palette*> palettes(palette_infos.size());
    unsigned palettes_constructed = 0;

    try {
        BOOST_FOREACH(const string& palette_file, palette_infos) {
            cvp_Palette * palette = cvp_Palette::load_from_file(palette_file);
            palettes[palettes_constructed++] = palette;
        }
        return new cvp_PaletteBundle(palettes);
    } catch (...) {
        for (unsigned idx = 0; idx < palettes_constructed; ++idx) {
            delete palettes[idx];
        }
        throw;
    }
} // load_from_files

void
cvp_PaletteBundle::save_to_files(
        const std::vector<std::string>& palette_fnames) const {
    const unsigned n_palettes = m_Palettes.size();
    assert(palette_fnames.size() == n_palettes);
    for (unsigned i = 0; i < n_palettes; ++i) {
        m_Palettes[i]->save_to_file(palette_fnames[i]);
    }
} // save_to_files

cvp_PaletteBundle& cvp_PaletteBundle::operator=(const cvp_PaletteBundle& rhs) {
    if (this != &rhs) {
        BOOST_FOREACH(cvp_Palette * palette, m_Palettes) {
            delete palette;
        }
        const unsigned palettes_num = rhs.size();
        m_Palettes.resize(palettes_num);
        for (unsigned palette_idx = 0; palette_idx < palettes_num;
                ++palette_idx) {
            m_Palettes[palette_idx] = new cvp_Palette(
                    *(rhs.palette(palette_idx)));
        }
    }
    return *this;
} // =

/////////////////////////////// PRIVATE //////////////////////////////////////

cvp_PaletteBundle::cvp_PaletteBundle(const std::vector<cvp_Palette*>& palettes) :
    m_Palettes(palettes) {
} // PaletteBundle

} // namespace OpenCvPlus
