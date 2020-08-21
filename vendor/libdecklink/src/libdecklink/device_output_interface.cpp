//
// Created by tibo and paola on 15/02/18.
//
#include <iostream>

#include <libdecklink/device.hpp>

#include "libdecklink/types.hpp"
#include "libdecklink/interface_helper.hpp"
#include "libdecklink/device_output_interface.hpp"

//
//  Public Methods
//

namespace DeckLink {

    DeviceOutputInterface::DeviceOutputInterface(DeviceOutputInterface &&src) noexcept {
        *this = std::move(src);
    }


    DeviceOutputInterface &DeviceOutputInterface::operator=(DeviceOutputInterface &&rhs) noexcept {
        if (this != &rhs) {

            _parent_device = rhs._parent_device;
            _output_enabled = rhs._output_enabled;
            _impl = std::move(rhs._impl);
            _keyer_impl = std::move(rhs._keyer_impl);

            // Make sure to disable the rhs
            rhs._parent_device = nullptr;
            rhs._output_enabled = false;
        }

        return *this;
    }

    DeviceOutputInterface::~DeviceOutputInterface() {
        if (_impl) disable();
    }


    VideoOutputFrame DeviceOutputInterface::create_video_frame(
            const DisplayMode &display_mode,
            const PixelFormat &pixel_format
    ) {

        if (!_impl) load_impl();

        IDeckLinkMutableVideoFrame *frame;
        HRESULT result = _impl->CreateVideoFrame(
                static_cast<int32_t>(display_mode.get_width()),
                static_cast<int32_t>(display_mode.get_height()),
                static_cast<int32_t>(display_mode.get_width() * PixelFormat_::get_depth(pixel_format)),
                static_cast<BMDPixelFormat>(pixel_format),
                static_cast<BMDVideoOutputFlags>(VideoOutputFlags::Default),
                &frame
        );

        if (result != S_OK) {
            BOOST_THROW_EXCEPTION(decklink_driver_error()
                << errmsg("Unable to create video frame")
                << decklink_command("IDeckLinkOutput::CreateVideoFrame")
                << decklink_error_code(static_cast<HResult>(result))
            );
        };

        return VideoOutputFrame(frame);
    }


    DeviceOutputInterface &DeviceOutputInterface::display_video_frame(
            const VideoOutputFrame& frame
    ) {
        if (!_impl) load_impl();

        HRESULT result = _impl->DisplayVideoFrameSync(frame.get_raw_ptr());

        if (result != S_OK) {
            std::string errdesc;
            switch (result) {
                case E_INVALIDARG:
                    errdesc = "Unable to display frame: invalid video mode, pixel format or flags";
                    break;
                case E_ACCESSDENIED:
                    errdesc = "Unable to access the hardware or input stream currently active";
                    break;
                default:
                    errdesc = "Unknown error";
                    break;
            }

            BOOST_THROW_EXCEPTION(decklink_driver_error()
                << errmsg(errdesc)
                << decklink_device(_parent_device->get_display_name())
                << decklink_command("IDeckLinkVideoOutput::DisplayVideoFrameSync")
            );
        }

        return *this;
    }


    DeviceOutputInterface &DeviceOutputInterface::enable(
            DisplayMode display_mode,
            PixelFormat pixel_format,
            VideoOutputFlags flags
    ) {
        if (!_impl) load_impl();

        BMDDisplayModeSupport is_supported;
        HRESULT result = _impl->DoesSupportVideoMode(
                static_cast<BMDDisplayMode>(display_mode.get_image_format()),
                static_cast<BMDPixelFormat>(pixel_format),
                static_cast<BMDVideoInputFlags>(flags),
                &is_supported, /* resultDisplayMode = */
                nullptr
        );

        if (result != S_OK) {
            BOOST_THROW_EXCEPTION(decklink_driver_error()
                << errmsg("Unable to query device for display mode support")
                << decklink_device(_parent_device->get_display_name())
                << decklink_error_code(static_cast<HResult>(result))
            );
        }

        if (!is_supported) {
            BOOST_THROW_EXCEPTION(runtime_error()
                << errmsg(
                    "The desired output configuration is not supported on this device: "
                        + ImageFormat_::pretty_print(display_mode.get_image_format()) + " - "
                        + PixelFormat_::pretty_print(pixel_format)
                )
                << decklink_device(_parent_device->get_display_name())
            );
        }

        result = _impl->EnableVideoOutput(
                static_cast<BMDDisplayMode>(display_mode.get_image_format()),
                static_cast<BMDVideoOutputFlags>(flags)
        );
        if (result != S_OK) {
            std::string errdesc;
            switch (result) {
                case E_ACCESSDENIED:
                    errdesc = "Unable to access the hardware or output stream currently active";
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
                << decklink_command("IDeckLinkVideoOutput::EnableVideoOutput")
            );
        }

        _output_enabled = true;
        return *this;
    }


    DeviceOutputInterface &DeviceOutputInterface::disable() {
        if (!_output_enabled) return *this;
        if (!_impl) load_impl();

        const auto result = _impl->DisableVideoOutput();

        if (result != S_OK) {
            BOOST_THROW_EXCEPTION(decklink_driver_error()
                << errmsg("Unable to disable video output")
                << decklink_command("IDeckLinkOutput::DisableVideoOutput")
                << decklink_error_code(static_cast<HResult>(result))
            );
        }

        _output_enabled = false;
        return *this;
    }


    std::vector<DisplayMode> DeviceOutputInterface::get_all_supported_display_modes() {
        if (!_impl) load_impl();

        std::vector<DisplayMode> display_modes;

        IDeckLinkDisplayMode *mode = nullptr;
        IDeckLinkDisplayModeIterator *it = nullptr;

        if (const HRESULT result = _impl->GetDisplayModeIterator(&it) != S_OK) {
            BOOST_THROW_EXCEPTION(decklink_driver_error()
                << errmsg("Unknown error")
                << decklink_command("IDeckLinkOutput::Disable VideoOutput")
                << decklink_error_code(static_cast<HResult>(result))
            );
        }

        assert(it);
        while (it->Next(&mode) == S_OK) {
            display_modes.emplace_back(mode);
        }

        return display_modes;
    }


    boost::optional<DisplayMode> DeviceOutputInterface::get_display_mode(
        ImageFormat image_format
    ) {
        const auto display_modes = get_all_supported_display_modes();

        auto found = std::find_if(
            display_modes.begin(), display_modes.end(),
            [&image_format](DisplayMode item) {
                return item.get_image_format() == image_format;
            });
        
        if (found != display_modes.end())
            return *found;
    
        return {};
    }


    bool DeviceOutputInterface::supports_pixel_format(
            const DisplayMode &display_mode,
            PixelFormat pixel_format
    ) {
        // Supported can be Supported or SupportedWithConversion
        // Both are OK for us
        const auto supported = get_display_mode_support(display_mode, pixel_format);

        return (supported != DisplayModeSupport::NotSupported);
    }


    std::vector<PixelFormat> DeviceOutputInterface::get_supported_pixel_formats(
            const DisplayMode &display_mode
    ) {
        std::vector<PixelFormat> formats;

        for (const auto format: PixelFormat_::Values) {
            if (format != PixelFormat::Unknown && supports_pixel_format(display_mode, format))
                formats.push_back(format);
        }

        return formats;
    }


    DisplayModeSupport DeviceOutputInterface::get_display_mode_support(
        const DisplayMode& display_mode,
        const PixelFormat pixel_format,
        const VideoOutputFlags output_flags
    ) {
        if (!_impl) load_impl();

        BMDDisplayModeSupport is_supported;
        const HRESULT result = _impl->DoesSupportVideoMode(
                display_mode.get_raw_mode(),
                static_cast<BMDPixelFormat>(pixel_format),
                static_cast<BMDVideoOutputFlags>(output_flags),
                &is_supported, /* resultDisplayMode = */
                nullptr
        );

        if (result != S_OK) {
            BOOST_THROW_EXCEPTION(decklink_driver_error()
                << errmsg(
                    "An unknown error occured whilst calling IDeckLinkOutput::DoesSupportVideoMode "
                    "on " + display_mode.get_name()
                )
                << decklink_device(_parent_device->get_display_name())
            );
        }

        return static_cast<DisplayModeSupport>(is_supported);
    }


    //
    // Protected Methods - Constructors
    //

    DeviceOutputInterface::DeviceOutputInterface(Device *parent_device)
            : _parent_device(parent_device) {
        if (!parent_device) {
            BOOST_THROW_EXCEPTION(null_pointer_error()
                << errmsg("Cannot create an output device from a null pointer")
            );
        }
    }
    
    //
    // Protected Methods - Loaders
    //
    
    
    void DeviceOutputInterface::load_impl() {
        load_interface(
                _parent_device->_device_impl.get(), IID_IDeckLinkOutput, _impl, "output"
        );
        assert(_impl);
    }

    void DeviceOutputInterface::load_keyer_interface() {
        load_interface(
                _parent_device->_device_impl.get(), IID_IDeckLinkKeyer, _keyer_impl, "keyer"
        );
        assert(_keyer_impl);
    }
    

    DeviceOutputInterface& DeviceOutputInterface::enable_keyer(bool is_external) {
        if (!_keyer_impl) load_keyer_interface();

        const HRESULT result = _keyer_impl->Enable(is_external);
        if (result != S_OK) {
            BOOST_THROW_EXCEPTION(decklink_driver_error()
                << errmsg("Unable to enable keying on this device")
                << decklink_command("IDeckLinkKeyer::Enable")
                << decklink_device(_parent_device->get_long_name())
                << decklink_error_code(static_cast<HResult>(result))
            );
        }
        
        _keyer_enabled = true;
        return *this;
    };
    
    
    DeviceOutputInterface& DeviceOutputInterface::set_opacity(const uint8_t level) {
        if (!_keyer_impl) load_keyer_interface();
        
        const HRESULT result = _keyer_impl->SetLevel(level);
        if (result != S_OK) {
            BOOST_THROW_EXCEPTION(decklink_driver_error()
                << errmsg("Unable to change the opacity on the keyer")
                << decklink_command("IDeckLinkKeyer::SetLevel")
                << decklink_device(_parent_device->get_long_name())
                << decklink_error_code(static_cast<HResult>(result))
            );
        }
        
        return *this;
    }
    
    
    DeviceOutputInterface& DeviceOutputInterface::set_opacity(const float level) {
        if (level < 0.0f or level > 1.0f)
            BOOST_THROW_EXCEPTION(out_of_range_error()
                << errmsg("Opacity should be in the inclusive range [ 0.0; 1.0 ]")
            );
        
        return set_opacity(static_cast<uint8_t>(level * 255));
    }
    
    
    DeviceOutputInterface& DeviceOutputInterface::ramp_up_opacity(uint32_t number_of_frames) {
        if (!_keyer_impl) load_keyer_interface();
        
        const HRESULT result = _keyer_impl->RampUp(number_of_frames);
        if (result != S_OK) {
            BOOST_THROW_EXCEPTION(decklink_driver_error()
                << errmsg("Unable to ramp up the opacity on the keyer")
                << decklink_command("IDeckLinkKeyer::RampUp")
                << decklink_device(_parent_device->get_long_name())
                << decklink_error_code(static_cast<HResult>(result))
            );
        }
        return *this;
    }
    
    DeviceOutputInterface& DeviceOutputInterface::ramp_down_opacity(uint32_t number_of_frames) {
        if (!_keyer_impl) load_keyer_interface();
        
        const HRESULT result = _keyer_impl->RampDown(number_of_frames);
        if (result != S_OK) {
            BOOST_THROW_EXCEPTION(decklink_driver_error()
                << errmsg("Unable to ramp down the opacity on the keyer")
                << decklink_command("IDeckLinkKeyer::RampDown")
                << decklink_device(_parent_device->get_long_name())
                << decklink_error_code(static_cast<HResult>(result))
            );
        }
        
        return *this;
    }
    
    DeviceOutputInterface& DeviceOutputInterface::disable_keyer() {
        if (!_keyer_enabled) return *this;
        if (!_keyer_impl) load_keyer_interface();
    
        const auto result = _keyer_impl->Disable();
    
        if (result != S_OK) {
            BOOST_THROW_EXCEPTION(decklink_driver_error()
                << errmsg("Unable to disable keyer output")
                << decklink_command("IDeckLinkKeyer::Disable")
                << decklink_error_code(static_cast<HResult>(result))
            );
        }
    
        _keyer_enabled = false;
        return *this;
    };
}
