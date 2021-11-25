//
// Created by tibo on 17/01/18.
//

#include <iostream>

#include <libdecklink/device.hpp>
#include <libdecklink/types.hpp>

#include "handle_errors.hpp"

using std::to_string;
using namespace DeckLink;

const int max_name_len = 35;
const int max_result_len = 13;

void show_help() {
    std:: cout << "deckstatus device-name\n"
               << "\n"
               << "Show the status information of the specified BlackMagic DeckLink video capture card.\n"
               << " \n";
}

void show_status(
    const std::string& name,
    const std::string& value,
    const std::string& description = ""
) {
    std::cout << " - "
              << std::setw(max_name_len) << std::right << name << ": "
              << std::setw(max_result_len) << std::left << value;
    
    if (!description.empty())
        std::cout << " (" << description << ")";
    
    std::cout << "\n";
}

int main(int argc, char** argv) {
    if (argc != 2) { show_help(); return -1; }
    
    try {
    
        std::string device_name{argv[1]};
        if ((device_name == "--list") or (device_name == "-l")) {
            const auto devices = Device::List();
        
            std::cout << "Listing detected devices:\n";
            for (const auto& device: devices) {
                std::cout << " - " << device.get_display_name()
                          << " ( " << device.get_vendor_name()
                          << " " << device.get_model_name() << " )\n";
            }
            return 0;
        }
    
        auto has_device = Device::Get(device_name);
        if (!has_device) {
            std::cerr << "Error: Unable to locate device named: " << device_name << ".\n";
            return -1;
        }
    
        auto& device = *has_device;
    
        std::cout << device.get_long_name() << " status: \n";
    
        const auto busy_state = device.get_int_status(StatusID::BusyState);
        if (busy_state) {
            const auto busy_state_value = static_cast<BusyState>(*busy_state);
            const std::string desc = BusyState_::get_description(busy_state_value);
            show_status("Busy State", to_string(busy_state_value), desc);
        
        }
        else {
            std::cout << " - Busy state: Unknown\n";
        }
    
        const auto duplex_mode = device.get_int_status(StatusID::DuplexMode);
        if (duplex_mode) {
            const auto duplex_mode_value = static_cast<DuplexMode>(*duplex_mode);
            const std::string desc = DuplexMode_::get_description(duplex_mode_value);
            show_status("Duplex Mode", to_string(duplex_mode_value), desc);
        }
        else {
            std::cout << " - Duplex mode: Unknown\n";
        }
    
        const auto video_signal_locked = device.get_bool_status(StatusID::VideoInputSignalLocked);
        if (video_signal_locked) {
            show_status("Input video signal locked", (*video_signal_locked ? "Yes" : "No"));
        }
        else {
            std::cout << " - Input video signal locked: Unknown\n";
        }
    
        const auto ref_signal_locked = device.get_bool_status(StatusID::ReferenceSignalLocked);
        if (ref_signal_locked) {
            show_status("Reference signal locked", (ref_signal_locked ? "Yes" : "No"));
        }
        else {
            std::cout << " - Reference signal locked: Unknown\n";
        }
    
        std::cout << " -\n";
        const auto current_input_video_mode = device.get_int_status(StatusID::CurrentVideoInputMode);
        if (current_input_video_mode) {
            const auto current_input_video_mode_value =
                static_cast<ImageFormat>(*current_input_video_mode);
            const std::string name = ImageFormat_::pretty_print(current_input_video_mode_value);
            show_status("Current input video image format", name);
        }
        else {
            std::cout << " - Current input video image format: Unknown\n";
        }
    
        const auto current_input_video_flags = device.get_int_status(StatusID::CurrentVideoInputFlags);
        if (current_input_video_flags) {
            const BitMask<DeckLink::VideoInputFlags> current_input_video_flags_value(*current_input_video_flags);
    
            std::cout << " - " << std::setw(max_name_len) << std::right
                      << "Current Video Input Flags" << ": ";
            
            bool first_flag = true;
            for (const auto flag: DeckLink::VideoInputFlags_::Values) {
                if ( !(current_input_video_flags_value & flag) )
                    continue;
                
                if (!first_flag) {
                    std::cout << " - " << std::setw(max_name_len) << std::right << "" << ": ";
                }
                
                std::cout << std::setw(max_result_len) << std::left << to_string(flag);
                first_flag = false;
            }
            
            // No flags were set
            if (first_flag) std::cout << "\n";

        }
        else {
            std::cout << " - Current input video flags: Unknown\n";
        }
    
        const auto current_input_video_px_fmt =
            device.get_int_status(StatusID::CurrentVideoInputPixelFormat);
        if (current_input_video_px_fmt) {
            const auto current_input_video_px_fmt_value =
                static_cast<PixelFormat>(*current_input_video_px_fmt);
            const std::string name = to_string(current_input_video_px_fmt_value);
            show_status("Current input video pixel format",
                PixelFormat_::pretty_print(current_input_video_px_fmt_value));
        }
        else {
            std::cout << " - Current input video pixel format: Unknown\n";
        }
    
        std::cout << " -\n";
        const auto
            detected_input_video_mode = device.get_int_status(StatusID::DetectedVideoInputMode);
        if (duplex_mode) {
            const auto detected_input_video_mode_value =
                static_cast<ImageFormat>(*detected_input_video_mode);
            const std::string name = ImageFormat_::pretty_print(detected_input_video_mode_value);
            show_status("Detected input video image format", name);
        }
        else {
            std::cout << " - Detected input video image format: Unknown\n";
        }
    
        const auto
            detected_input_video_flags = device.get_int_status(StatusID::DetectedVideoInputFlags);
        if (detected_input_video_flags) {
            const BitMask<DeckLink::VideoInputFlags> detected_input_video_flags_value(*detected_input_video_flags);

            std::cout << " - " << std::setw(max_name_len) << std::right
                      << "Detected Video Input Flags" << ": ";
    
            bool first_flag = true;
            for (const auto flag: DeckLink::VideoInputFlags_::Values) {
                if ( !(detected_input_video_flags_value & flag) )
                    continue;
        
                if (!first_flag) {
                    std::cout << " - " << std::setw(max_name_len) << std::right << "" << ": ";
                }
        
                std::cout << std::setw(max_result_len) << std::left << to_string(flag);
                first_flag = false;
            }
    
            // No flags were set
            if (first_flag) std::cout << "\n";
        }
        else {
            std::cout << " - Detected input video flags: Unknown\n";
        }
    
        std::cout << " -\n";
        const auto
            current_output_video_mode = device.get_int_status(StatusID::CurrentVideoOutputMode);
        if (current_output_video_mode) {
            const auto current_output_video_mode_value =
                static_cast<ImageFormat>(*current_output_video_mode);
            const std::string name = ImageFormat_::pretty_print(current_output_video_mode_value);
            show_status("Current output video image format", name);
        }
        else {
            std::cout << " - Current output video image format: Unknown\n";
        }
    
        const auto
            current_output_video_flags = device.get_int_status(StatusID::CurrentVideoOutputFlags);
        if (current_output_video_flags) {
            const BitMask<DeckLink::VideoOutputFlags> current_output_video_flags_value(*current_output_video_flags);
    
            std::cout << " - " << std::setw(max_name_len) << std::right
                      << "Current output video flags: ";
    
            bool first_flag = true;
            for (const auto flag: DeckLink::VideoOutputFlags_::Values) {
                if ( !(current_output_video_flags_value & flag) )
                    continue;
        
                if (!first_flag) {
                    std::cout << " - " << std::setw(max_name_len) << std::right << "" << ": ";
                }
        
                std::cout << std::setw(max_result_len) << std::left << to_string(flag);
                first_flag = false;
            }
    
            // No flags were set
            if (first_flag) std::cout << "\n";
        }
        else {
            std::cout << " - Current output video flags: Unknown\n";
        }
    
        const auto current_output_video_px_fmt =
            device.get_int_status(StatusID::LastVideoOutputPixelFormat);
        if (current_output_video_px_fmt) {
            const auto current_output_video_px_fmt_value =
                static_cast<PixelFormat>(*current_output_video_px_fmt);
            const std::string name = to_string(current_output_video_px_fmt_value);
            show_status("Last output video pixel format",
                PixelFormat_::pretty_print(current_output_video_px_fmt_value));
        }
        else {
            std::cout << " - Last output video pixel format: Unknown\n";
        }
    
    } catch (const DeckLink::decklink_driver_error &ex) {
        print_driver_error_details(ex);

    } catch (const DeckLink::runtime_error& ex) {
        std::cout << "\n"
                  << "An error occurred! \n"
                  << boost::diagnostic_information(ex) << "\n";
    }
    
    return 0;
}
