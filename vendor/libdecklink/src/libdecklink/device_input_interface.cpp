//
// Created by tibo on 16/01/18.
//

#include <libdecklink/device.hpp>

#include "libdecklink/types.hpp"
#include "libdecklink/interface_helper.hpp"
#include "libdecklink/device_input_interface.hpp"

//
//  Public Methods
//

namespace DeckLink {
    
    DeviceInputInterface::DeviceInputInterface(DeviceInputInterface&& src) noexcept {
        *this = std::move(src);
    }
    
    
    DeviceInputInterface& DeviceInputInterface::operator=(DeviceInputInterface&& rhs) noexcept {
        if (this != &rhs) {
            _callback = rhs._callback;
            if (_callback._owner)
                _callback._owner = this;
            
            _parent_device = rhs._parent_device;
            _enabled = rhs._enabled;
            _impl = std::move(rhs._impl);
    
            // Make sure to disable the rhs
            rhs._parent_device = nullptr;
            rhs._enabled = false;
        }

        return *this;
    }
    
    
    DeviceInputInterface& DeviceInputInterface::start() {
        if (!_impl) load_impl();
        
        if (!_enabled) {
            enable();
        }
        
        _impl->StartStreams();
        
        return *this;
    }
    
    
    DeviceInputInterface& DeviceInputInterface::pause() {
        if (!_impl) load_impl();
    
        const auto result = _impl->PauseStreams();
    
        if (result != S_OK) {
            BOOST_THROW_EXCEPTION(decklink_driver_error()
                << errmsg("Unexpected error")
                << decklink_command("IDeckLinkInput::PauseStreams")
                << decklink_error_code(static_cast<HResult>(result))
            );
        }
    
        return *this;
    }
    
    
    DeviceInputInterface& DeviceInputInterface::stop() {
        if (!_impl) load_impl();
        
        const auto result = _impl->StopStreams();
        
        // E_ACCESSDENIED means that the stream is already stopped
        if (result != S_OK && result != E_ACCESSDENIED) {
            BOOST_THROW_EXCEPTION(decklink_driver_error()
                << errmsg("Unexpected error")
                << decklink_command("IDeckLinkInput::StopStreams")
                << decklink_error_code(static_cast<HResult>(result))
            );
        }
        
        return *this;
    }
    
    
    DeviceInputInterface& DeviceInputInterface::flush() {
        if (!_impl) load_impl();
    
        const auto result = _impl->FlushStreams();
    
        if (result != S_OK) {
            BOOST_THROW_EXCEPTION(decklink_driver_error()
                << errmsg("Unexpected error")
                << decklink_command("IDeckLinkInput::FlushStreams")
                << decklink_error_code(static_cast<HResult>(result))
            );
        }
    
        return *this;
    }
    
    
    DeviceInputInterface& DeviceInputInterface::enable() {
        if (!_parent_device->supports_input_format_detection()) {
            BOOST_THROW_EXCEPTION(runtime_error()
                << errmsg(
                    "This device does not support automatic input format detection. The input must be enabled manually.")
                << decklink_device(_parent_device->get_display_name())
            );
        }
        
        // For automatic format detection we need to grab a valid display mode and pixel format
        const auto display_modes = get_all_supported_display_modes();
        if (display_modes.empty()) {
            BOOST_THROW_EXCEPTION(runtime_error()
                << errmsg("The input device has no supported display modes")
                << decklink_device(_parent_device->get_display_name())
            );
        };
        
        auto flags = VideoInputFlags::Default;
        if (_parent_device->supports_input_format_detection())
            flags = VideoInputFlags::EnableFormatDetection;
        
        const auto display_mode = display_modes.front();
        const auto pixel_formats = get_supported_pixel_formats(display_mode.get_image_format());
        
        // If a display mode is supported it should have at least on valid pixel format
        assert(!pixel_formats.empty());
        
        // Now we can enable the interface
        enable(display_mode.get_image_format(), pixel_formats.front(), { flags });
        
        _enabled = true;
        return *this;
    }
    
    
    DeviceInputInterface&DeviceInputInterface::enable(
        const ImageFormat image_format,
        const PixelFormat pixel_format,
        const BitMask<VideoInputFlags>& flags
    ) {
        if (!_impl) load_impl();
    
        DisplayModeSupport support = supports_video_mode(image_format, pixel_format, flags);
        if (support == DisplayModeSupport::NotSupported) {
            BOOST_THROW_EXCEPTION(runtime_error()
                << errmsg("The desired input configuration is not supported on this device: " + ImageFormat_::pretty_print(image_format) + " - " + PixelFormat_::pretty_print(pixel_format))
                << decklink_device(_parent_device->get_display_name())
            );
        }
    
        HRESULT result = _impl->EnableVideoInput(
              static_cast<BMDDisplayMode>(image_format)
            , static_cast<BMDPixelFormat>(pixel_format)
            , static_cast<BMDVideoInputFlags>(flags.bits())
        );
        if (result != S_OK) {
            std::string errdesc;
            switch(result) {
                case E_INVALIDARG:
                    errdesc = "Invalid video mode, pixel format or flags";
                    break;
                case E_ACCESSDENIED:
                    errdesc = "Unable to access the hardware or input stream currently active";
                    break;
                case E_OUTOFMEMORY:
                    errdesc = "Unable to create a new frame";
                    break;
                default:
                    errdesc = "Unknown error";
                    break;
            }
            
            BOOST_THROW_EXCEPTION(decklink_driver_error()
                << errmsg(errdesc)
                << decklink_device(_parent_device->get_display_name())
                << decklink_command("IDeckLinkVideoInput::EnableVideoInput")
            );
        }
        
        _enabled  = true;
        return *this;
    }
    
    
    DeviceInputInterface& DeviceInputInterface::disable() {
        if (!_impl) load_impl();
    
        const auto result = _impl->DisableVideoInput();
    
        // E_ACCESSDENIED means that the stream is already stopped
        if (result != S_OK) {
            BOOST_THROW_EXCEPTION(decklink_driver_error()
                << errmsg("Unable to stop disable video input")
                << decklink_command("IDeckLinkInput::StopStreams")
                << decklink_device(_parent_device->get_display_name())
                << decklink_error_code(static_cast<HResult>(result))
            );
        }
    
        return *this;
    }
    
    
    DeviceInputInterface& DeviceInputInterface::set_callback(CaptureCallback&& callback) {
        stop();
        
        _callback = callback;
        _callback._owner = this;
        assert(this);
        
        if (const HRESULT result = _impl->SetCallback(&_callback) != S_OK) {
            BOOST_THROW_EXCEPTION(decklink_driver_error()
                << errmsg("Unable to set callback")
                << decklink_command("IDeckLinkInput::SetCallback")
                << decklink_device(_parent_device->get_display_name())
                << decklink_error_code(static_cast<HResult>(result))
            );
        }
        
        return *this;
    }
    
    
    std::vector<DisplayMode> DeviceInputInterface::get_all_supported_display_modes() {
        if (!_impl) load_impl();
        
        std::vector<DisplayMode> display_modes;
        
        IDeckLinkDisplayMode* mode = nullptr;
        IDeckLinkDisplayModeIterator* it = nullptr;
        
        if (const HRESULT result = _impl->GetDisplayModeIterator(&it) != S_OK) {
            BOOST_THROW_EXCEPTION(decklink_driver_error()
                << errmsg("Unknown failure when calling GetDisplayModeIterator")
                << decklink_command("IDeckLilnkInput::GetDisplayModeIterator")
                << decklink_device(_parent_device->get_display_name())
                << decklink_error_code(static_cast<HResult>(result))
            );
        }
        
        assert(it);
        while (it->Next(&mode) == S_OK) {
            display_modes.emplace_back(mode);
        }
        
        return display_modes;
    }
    
    
    bool DeviceInputInterface::supports_pixel_format(
        const ImageFormat image_format,
        const PixelFormat pixel_format,
        const BitMask<VideoInputFlags>& flags
    ) {
        if (!_impl) load_impl();
        
        DisplayModeSupport support = supports_video_mode(image_format, pixel_format, flags);
        
        // Supported can be SupportedWithConversion or Supported. Both are OK for us
        return (support != DisplayModeSupport::NotSupported);
    }
    
    
    std::vector<PixelFormat> DeviceInputInterface::get_supported_pixel_formats(
        const ImageFormat image_format
    ) {
        std::vector<PixelFormat> formats;
        
        for (const auto pixel_format: PixelFormat_::Values) {
            if (pixel_format != PixelFormat::Unknown && supports_pixel_format(image_format, pixel_format))
                formats.push_back(pixel_format);
        }
        
        return formats;
    }

    //
    // Input / Output Interface
    //
    
    IDeckLinkInput *DeviceInputInterface::get_raw_device() {
        assert(this);
        if (!_impl) load_impl();
        
        assert(_impl);
        return _impl.get();
    }
    
    //
    // Protected methods
    //
    
    DeviceInputInterface::DeviceInputInterface(Device *parent_device)
        : _parent_device(parent_device)
    {
        if (!parent_device) {
            BOOST_THROW_EXCEPTION(null_pointer_error()
                << errmsg("Cannot create an input device from a null pointer")
            );
        }
    }
    
    
    DisplayModeSupport DeviceInputInterface::supports_video_mode(
        const ImageFormat image_format,
        const PixelFormat pixel_format,
        const BitMask<VideoInputFlags>& flags
    ) {
        BMDDisplayModeSupport is_supported;
        HRESULT result = _impl->DoesSupportVideoMode(
              static_cast<BMDDisplayMode>(image_format)
            , static_cast<BMDPixelFormat>(pixel_format)
            , static_cast<BMDVideoInputFlags>(flags.bits())
            , &is_supported
            , /* resultDisplayMode = */ nullptr
        );
    
        if (result != S_OK) {
            BOOST_THROW_EXCEPTION(decklink_driver_error()
                << errmsg("An unknown error occured whilst calling IDeckLinkInput::DoesSupportVideoMode on " + ImageFormat_::pretty_print(image_format))
                << decklink_device(_parent_device->get_display_name())
                << decklink_error_code(static_cast<HResult>(result))
            );
        }
        
        return static_cast<DisplayModeSupport>(is_supported);
    }
    
    
    void DeviceInputInterface::load_impl() {
        load_interface(
            _parent_device->_device_impl.get(), IID_IDeckLinkInput, _impl, "input"
        );
        assert(_impl);
    }
    
}
