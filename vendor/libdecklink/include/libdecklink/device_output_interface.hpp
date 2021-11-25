//
// Created by tibo and paola on 15/02/18.
//

#pragma once

#include <boost/noncopyable.hpp>

#include "decklink_sdk/DeckLinkAPI.h"

#include "libdecklink/errors.hpp"
#include "libdecklink/display_mode.hpp"
#include "libdecklink/video_output_frame.hpp"
#include "libdecklink/decklink_handle_deleter.hpp"

namespace DeckLink {
    
    class Device;
    
    class DeviceOutputInterface: private boost::noncopyable {
        
        friend class Device;
    
    public: /* Public Methods */
        
        ~DeviceOutputInterface();
        
        
        /**
         * Enable the output interface.
         *
         * @param display_mode The display mode to use for video frames
         * @param pixel_format The pixel format to use for video frames
         * @param flags The flags to use to configure the interface. Default is usually good.
         
         * @return this (for method chaining)
         */
        DeviceOutputInterface& enable(
            DisplayMode display_mode,
            PixelFormat pixel_format,
            VideoOutputFlags flags = VideoOutputFlags::Default
        );
        
        /**
         * Disable the interface.
         *
         * This is a no-op if the interface has not been loaded yet.
         *
         * @return this (for method chaining)
         */
        DeviceOutputInterface& disable();
        
        /**
         * Get a list of all the display modes supported by this device.
         *
         * @return this (for method chaining)
         */
        std::vector<DisplayMode> get_all_supported_display_modes();
        
        // TODO: Deprecate Me
        boost::optional<DisplayMode> get_display_mode(ImageFormat image_format);
        
        /**
         * Is the specified Image Format + Pixel Format supported by the card ?
         *
         * @param display_mode The image format to try
         * @param pixel_format The associated pixel formar
         * @return Whether the combination is supported by the device
         */
        bool supports_pixel_format(const DisplayMode& display_mode, const PixelFormat pixel_format);
    
        /**
         * How is the specified Image Format + Pixel Format combination supported by the hardware ?
         *
         * Whilst DeviceOutputInterface::supports_pixel_format only tells you whether a display mode
         * is supported or not this function allows you to differentiate between `supported` and
         * `supproted with conversion`.
         *
         * @param display_mode The Image Format to try
         * @param pixel_format The associated pixel format
         * @param output_flags The associated #VideoOutputFlags
         * @return Whether the combination is supported by the device
         */
        DisplayModeSupport get_display_mode_support(
            const DisplayMode& display_mode,
            const PixelFormat pixel_format,
            const VideoOutputFlags output_flags = VideoOutputFlags::Default
        );
        
        /**
         * Get all the supported pixel formats for a specified display mode.
         *
         * @param display_mode The display_mode for which to retrieve the supported pixel formats
         * @return A list of all the pixel formats supported by this display mode
         */
        std::vector<PixelFormat> get_supported_pixel_formats(const DisplayMode& display_mode);
        
        // Keyer Interface
    
        /**
         * Enable the keyer using internal or external keying.
         *
         * @param is_external Whether to use internal or external keying
         * @return this (for method chaining)
         */
        DeviceOutputInterface& enable_keyer(bool is_external = false);
        
        /**
         * Disable the keyer.
         *
         * If the `enable_keyer` has not already been called this is a no-op.
         *
         * @return this (for method chaining)
         */
        DeviceOutputInterface& disable_keyer();
    
    
        /**
         * Set the opacity of the keyed images. This is in addition to any opacity specified in the
         * alpha channel of the keyed images.
         *
         * @param level
         *  The opacity as a number between 0 and 255 where 0 is fully transparent and 255 is fully
         *  opaque.
         * @return this (for method chaining)
         */
        DeviceOutputInterface& set_opacity(const uint8_t level);
    
        /**
         * Set the opacity of the keyed images. This is in addition to any opacity specified in the
         * alpha channel of the keyed images.
         *
         * @param level
         *  The opacity as a number between 0.0 and 1.0 where 0.0 is fully transparent and 1.0 is
         *  fully opaque. Values out of this range will cause an out_of_range error to be thrown.
         * @return this (for method chaining)
         */
        DeviceOutputInterface& set_opacity(const float level);
    
        /**
         * Progressively blend an image in over a number of frames
         *
         * @param number_of_frames The number of frames over which to do the blending
         * @return this (for method chaining)
         */
        DeviceOutputInterface& ramp_up_opacity(uint32_t number_of_frames);
    
        /**
         * Progressively blend an image out over a number of frames
         *
         * @param number_of_frames The number of frames over which to do the blending
         * @return this (for method chaining)
         */
        DeviceOutputInterface& ramp_down_opacity(uint32_t number_of_frames);
    
    
    
        VideoOutputFrame create_video_frame(
            const DisplayMode& display_mode, const PixelFormat& pixel_format);
    
        DeviceOutputInterface& display_video_frame(const VideoOutputFrame& outFrame);
    
    
    protected: /* Protected Methods - Constructors */
        
        DeviceOutputInterface() = default;
        
        explicit DeviceOutputInterface(Device *parent_device);
        
        DeviceOutputInterface(DeviceOutputInterface&& src) noexcept;
        
        DeviceOutputInterface& operator=(DeviceOutputInterface&& rhs) noexcept;

    protected: /* Protected Methods - Loaders */
        
        /// Load the underlying output interface
        void load_impl();
        
        /// Load the udnerlying keyer interface
        void load_keyer_interface();
    
    
    private: /* Private Members */
        
        Device *_parent_device = nullptr;
        
        bool _output_enabled = false;
        bool _keyer_enabled = false;
    
        std::unique_ptr<IDeckLinkOutput, IDeckLinkHandleDeleter> _impl;
        std::unique_ptr<IDeckLinkKeyer, IDeckLinkHandleDeleter> _keyer_impl;
        
    };
    
}
