//
// Created by tibo on 17/01/18.
//

#include <iostream>

#include <libdecklink/device.hpp>
#include <libdecklink/capture_callback.hpp>

#include "handle_errors.hpp"

void show_help() {
    std:: cout << "deckview <device-name> | --list\n"
               << "\n"
               << "Show the output of a BlackMagic DeckLink video capture card in real time.\n"
               << " \n"
               << "USAGE:\n"
               << "    --list   List all of the detected DeckLink devices\n"
               << "\n";
}

void on_new_frame(const DeckLink::VideoInputFrame& frame) {
    static unsigned long count = 0;
    
    std::string timecode_str = "N/A";
    try {
        timecode_str = frame.timecode(DeckLink::TimecodeFormat::Serial);
    } catch (const DeckLink::decklink_driver_error&) {
        // Do nothing .. we just couldn't get a timecode
    }
    
    ++count;
    std::cout << "Frame #" << count << " - " << frame.width() << "x" << frame.height()
              << " @ " << DeckLink::PixelFormat_::pretty_print(frame.pixel_format())
              << " (size: " << frame.size() << " bytes)"
              << " timecode: " << timecode_str
              << std::fixed << std::setprecision(2)
              << " stream time: " << frame.stream_time()
              << " hardware time: " << frame.hardware_reference_timestamp()
              << std::defaultfloat
              << std::endl;
              
}

void on_format_changed(
    DeckLink::InputFormatChangedEvent notification_event,
    const DeckLink::DisplayMode& new_display_mode,
    DeckLink::DetectedVideoInputFormatFlags /* detected_signal_flag */
) {
    auto notification_event_raw = static_cast<BMDNotifications>(notification_event);
    
    // Check for video field changes
    if (notification_event_raw & bmdVideoInputFieldDominanceChanged) {
        BMDFieldDominance fieldDominance;
        
        const auto field_dominance = new_display_mode.get_field_dominance();
        std::cout << "Input field dominance changed to " << to_string(field_dominance) << " ("
                  << DeckLink::FieldDominance_::get_description(field_dominance) << ")\n";
    }
    
    // Check if the pixel format has changed
    if (notification_event_raw & bmdVideoInputColorspaceChanged) {
        std::cout << "Input color space changed to (SOMETHING I HAVEN?T IMPLEMENTED)\n";
                 // << DeckLink::DetectedVideoInputFormatFlags_::pretty_print(detected_signal_flag) << "\n";
    }
    
    // Check if the video mode has changed
    if (notification_event_raw & bmdVideoInputDisplayModeChanged) {
        std::cout << "Input display mode changed to: " << new_display_mode.get_name() << "\n";
    }
}

void on_error(const DeckLink::VideoInputError err) {
    std::cout << "Video input error: " << to_string(err) << "\n";
}

int main(int argc, char** argv) {
    if (argc != 2) { show_help(); return -1; }
    
    std::string device_name{ argv[1] };
    if ((device_name == "--list") or (device_name == "-l")) {
        const auto devices = DeckLink::Device::List();
        
        std::cout << "Listing detected devices:\n";
        for (const auto& device: devices) {
            std::cout << " - " << device.get_display_name()
                      << " ( " << device.get_vendor_name()
                      << " " << device.get_model_name() << " )\n";
        }
        return 0;
    }
    
    auto has_device = DeckLink::Device::Get(device_name);
    if (!has_device) {
        std::cerr << "Error: Unable to locate device named: " << device_name << ".\n";
        return -1;
    }
    
    try {
        auto& device = *has_device;
        std::cout << " Using " << device.get_display_name()
                  << " ( " << device.get_vendor_name()
                  << " " << device.get_model_name() << " ) - Press <ENTER> to exit\n";
    
        DeckLink::CaptureCallback delegate(on_new_frame, on_format_changed, on_error);
        delegate.set_timecode_format(DeckLink::TimecodeFormat::RP188Any);
    
        device.input()
              .set_callback(std::move(delegate))
              .enable()
              .start();
    
        getchar();
        printf("Exiting.\n");
    
        device.input()
              .stop()
              .disable();
    
    }  catch (const DeckLink::decklink_driver_error &ex) {
        print_driver_error_details(ex);
    
    } catch (const DeckLink::runtime_error& ex) {
        std::cout << "\n"
                  << "An error occurred! \n"
                  << boost::diagnostic_information(ex) << "\n";
    }
    
    std::cout << "Shutting down cleanly\n";
    return 0;
}
