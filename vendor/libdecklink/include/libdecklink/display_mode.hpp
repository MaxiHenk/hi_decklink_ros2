//
// Created by tibo on 15/01/18.
//

#pragma once

#include <string>

#include "decklink_sdk/DeckLinkAPI.h"

#include "libdecklink/types.hpp"
#include "libdecklink/bitmask.hpp"

namespace DeckLink {
    
    class DisplayMode {
        
    public:
        /**
         * Construct an Empty DisplayMode object. The \a m_is_initialised variable will be false.
         * Accessing any of the getters of the object will result in a DeckLink::runtime_error
         * being thrown.
         */
        DisplayMode() = default;
        
        /**
         * Initialise a #DisplayMode object from an \c IDeckLinkDisplayMode object. In this case the
         * object will be correctly initialised.
         *
         * @param mode
         *      A pointer to the \c IDeckLinkDisplayMode from which to retrieve the display mode
         *      information.
         */
        explicit DisplayMode(IDeckLinkDisplayMode *mode);
    
        DisplayMode(const DisplayMode& other) = default;
        
        /**
         * Was this instance initialised from a valid \c IDeckLinkDisplayMode. If the object was
         * default constructed then no display mode was provided and the width, height, etc
         * variables are unset.
         *
         * @return Whether the display mode is initialised
         */
        bool is_valid() const { return _valid; }
    
        // Getters
    
        ImageFormat get_image_format() const {
            return static_cast<ImageFormat>(_raw_mode);
        }
        
        long get_width() const { return _width; }
        
        long get_height() const { return _height; }
        
        double get_framerate() const { return _framerate; }
        
        const std::string& get_name() const { return _name; }
        
        FieldDominance get_field_dominance() const { return _field_dominance; }
        
        const BitMask<DisplayModeFlags>& get_flags() const { return _flags; }
        
        BMDDisplayMode get_raw_mode() const { return _raw_mode; }
        
        bool has_3D_support() const { return bool(_flags & DisplayModeFlags::Supports3D); }
        
    protected:
    
    private: /* Member Variables */
        
        // Simple Interface
        
        /// Was this DisplayMode initialised from a valid IDeckLink Display Mode ?
        bool _valid = false;
    
        /// The width of the image in pixels
        long _width = -1;
        
        /// The height of the image in pixels
        long _height = -1;
    
        /// The framerate of the video feed
        double _framerate = -1.0;
        
        FieldDominance _field_dominance;
        
        /// The name of the mode
        std::string _name;
    
        BMDDisplayMode _raw_mode = 0;
    
        BitMask<DisplayModeFlags> _flags;
    };
}
