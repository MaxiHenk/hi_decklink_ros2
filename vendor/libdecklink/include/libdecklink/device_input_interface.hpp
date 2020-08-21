//
// Created by tibo on 16/01/18.
//

#pragma once

#include <boost/noncopyable.hpp>

#include "decklink_sdk/DeckLinkAPI.h"

#include "libdecklink/bitmask.hpp"
#include "libdecklink/display_mode.hpp"
#include "libdecklink/capture_callback.hpp"
#include "libdecklink/decklink_handle_deleter.hpp"


namespace DeckLink {

    class Device;
    
    class DeviceInputInterface: private boost::noncopyable {
    
        friend class Device;
        
    public: /* Public Methods */
        
        DeviceInputInterface& start();
        
        DeviceInputInterface& pause();
        
        DeviceInputInterface& stop();
        
        DeviceInputInterface& flush();
        
        DeviceInputInterface& enable();
    
        DeviceInputInterface& enable(
            const ImageFormat image_format,
            const PixelFormat pixel_format,
            const BitMask<VideoInputFlags>& flags = {VideoInputFlags::Default}
        );
        
        DeviceInputInterface& disable();
        
        DeviceInputInterface& set_callback(CaptureCallback&& callback);
        
        std::vector<DisplayMode> get_all_supported_display_modes();
        
        bool supports_pixel_format(
            const ImageFormat image_format,
            const PixelFormat pixel_format,
            const BitMask<VideoInputFlags>& flags = {VideoInputFlags::Default}
        );
        
        std::vector<PixelFormat> get_supported_pixel_formats(const ImageFormat image_format);

        // Advanced stuff

        /**
         * Get a raw pointer to the underlying device.
         * This allows you to call additional methods that may not be wrapped by the
         * DeviceInputInterface class.
         *
         * @return A pointer to the device
         */
        IDeckLinkInput* get_raw_device();
        
    protected: /* Protected Methods */
        
        DeviceInputInterface() {}
        
        explicit DeviceInputInterface(Device* parent_device);
    
        DeviceInputInterface(DeviceInputInterface&& src) noexcept;
    
        DeviceInputInterface& operator= (DeviceInputInterface&& rhs) noexcept;
        
        DisplayModeSupport supports_video_mode(
            const ImageFormat image_format,
            const PixelFormat pixel_format,
            const BitMask<VideoInputFlags>& flags = {VideoInputFlags::Default}
        );
        
        void load_impl();
        
    private: /* Private Members */
        
        Device* _parent_device = nullptr;
        std::unique_ptr<IDeckLinkInput, IDeckLinkHandleDeleter> _impl;
        
        bool _enabled = false;
    
        CaptureCallback _callback;
    };
    
}
