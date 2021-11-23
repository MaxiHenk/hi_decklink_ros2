//
// Created by tibo on 15/01/18.
//

#include <cassert>

#include "decklink_sdk/LinuxCOM.h"

#include "libdecklink/types.hpp"
#include "libdecklink/errors.hpp"
#include "libdecklink/display_mode.hpp"

DeckLink::DisplayMode::DisplayMode(IDeckLinkDisplayMode *mode) {
    assert(mode);
    
    _width = mode->GetWidth();
    _height = mode->GetHeight();
    _raw_mode = mode->GetDisplayMode();
    
    _flags = BitMask<DisplayModeFlags>(mode->GetFlags());
    
    _field_dominance = static_cast<FieldDominance>(mode->GetFieldDominance());
    
    // The framerate is stored in a weird format.
    BMDTimeValue  time_value;
    BMDTimeScale time_scale;
    if (const HRESULT result = mode->GetFrameRate(&time_value, &time_scale) != S_OK) {
        BOOST_THROW_EXCEPTION(decklink_driver_error()
            << errmsg("Unkown failure when calling GetFrameRate on IDeckLinkDisplayMode")
            << decklink_error_code(static_cast<HResult>(result))
        );
    }
    _framerate = static_cast<double>(time_scale) / static_cast<double>(time_value);
    
    char *buf= nullptr;
    if (const HRESULT result = mode->GetName(const_cast<const char**>(&buf)) != S_OK) {
        BOOST_THROW_EXCEPTION(decklink_driver_error()
            << errmsg("Unkown failure when calling GetName on IDeckLinkDisplayMode")
            << decklink_error_code(static_cast<HResult>(result))
        );
    }
    assert(buf);
    _name = std::string(buf);
    free(buf);
    
    _valid = true;
}
