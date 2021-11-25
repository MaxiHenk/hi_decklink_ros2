//
// Created by tibo on 17/01/18.
//

#include <iostream>

#include <libdecklink/device.hpp>
#include <libdecklink/capture_callback.hpp>

#include "handle_errors.hpp"

using namespace DeckLink;

void show_help() {
    std::cout << "deckquery device-name\n"
              << "\n"
              << "Query the specified BlackMagic DeckLink video capture card for the specified attribute.\n"
              << "\n"
              << "USAGE:\n"
              << "  --list                 List the connected devices\n"
              << "  --input-video-modes    List the video modes supported on the input\n"
              << "  --output-video-modes   List the video modes supported on the output\n"
              << "  --list-attributes      List all the device attributes known to this program\n"
              << "  --query-attribute      Query the device for a specific AttributeID\n"
              << "  --query-all-attributes Query the device for all known AttributeIDs\n"
              << "\n"
              << "QUERY FORMAT:\n"
              << ""
              << "\n";
}


std::string get_attribute(DeckLink::Device& device, const AttributeID attribute) {
    std::ostringstream oss;
    
    try {
        switch (AttributeID_::get_value_type(attribute)) {
            case AttributeType::Flag: {
                const auto result = device.get_bool_attribute(attribute);
                oss << (result ? std::to_string(*result) : "");
                break;
            }
            
            case AttributeType::Int: {
                const auto result = device.get_int_attribute(attribute);
                oss << (result ? std::to_string(*result) : "");
                break;
            }
            
            case AttributeType::Float: {
                const auto result = device.get_float_attribute(attribute);
                oss << (result ? std::to_string(*result) : "");
                break;
            }
            
            case AttributeType::String: {
                const auto result = device.get_string_attribute(attribute);
                oss << (result ? *result : "");
                break;
            }
        }
    }
    catch (const DeckLink::decklink_driver_error& err) {
        // The driver will sometimes return errors for perfectly valid attributes.
        // It's ugly but we're going to swallow them and pretend that everything is fine
        return "";
    }
    
    return oss.str();
}


int main(int argc, char **argv) {
    // We need at least one option
    if (argc < 2) {
        show_help();
        return -1;
    }

    try {
        std::string device_name{argv[1]};
        if ((device_name == "--list") or (device_name == "-l")) {
            const auto devices = DeckLink::Device::List();
            
            std::cout << "Listing detected devices:\n";
            for (const auto &device: devices) {
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
        auto& device = *has_device;

        // Here we need at least 3 options
        if (argc < 3) {
            show_help();
            return -1;
        }
        std::string command{argv[2]};
 
        const int display_name_len = 19;
        const int px_fmt_len = 13;

        if (command == "--input-video-modes") {

            std::cout << "Supported input display modes for " << device.get_long_name() << ":\n\n"
                      << "       Image Formats | Pixel Formats \n";
            for (const auto &display_mode: device.input().get_all_supported_display_modes()) {
                
                std::string display_mode_str = display_mode.get_name();
                if (display_mode.has_3D_support())
                    display_mode_str += " (+3D)";
                    
                std::cout << std::setw(display_name_len) << std::right
                          << display_mode_str << "  | ";

                for (const auto px_fmt: PixelFormat_::Values) {
                    if (px_fmt == PixelFormat::Unknown) continue;

                    const auto image_format = display_mode.get_image_format();
                    const bool supported = device.input().supports_pixel_format(image_format, px_fmt);
                    
                    if (supported) {
                        const std::string px_fmt_str = PixelFormat_::pretty_print(px_fmt);
                            std::cout << std::setw(px_fmt_len) << px_fmt_str << "   ";
                    } else {
                        std::cout << std::setw(px_fmt_len) << std::internal << " ---- " << "   ";
                    }
                }
                std::cout << "\n";
            }
            
            return 0;

        } else if (command == "--output-video-modes") {
            std::cout << "Supported output display modes for " << device.get_long_name() << ":\n\n"
                      << "   Image Formats | Pixel Formats \n";
            for (const auto &display_mode: device.output().get_all_supported_display_modes()) {
                std::cout << std::setw(display_name_len) << std::right
                          << display_mode.get_name() << "  | ";
                for (const auto px_fmt: DeckLink::PixelFormat_::Values) {
                    if (px_fmt == PixelFormat::Unknown) continue;

                    const auto supported = device.output().get_display_mode_support(display_mode, px_fmt, VideoOutputFlags::Default);

                    const std::string px_fmt_str = PixelFormat_::pretty_print(px_fmt);
                    switch (supported) {
                        case DisplayModeSupport::NotSupported:
                            std::cout << std::setw(px_fmt_len) << std::internal << " ---- " << "   ";
                            break;

                        case DisplayModeSupport::Supported:
                            std::cout << std::setw(px_fmt_len) << px_fmt_str << "   ";
                            break;

                        case DisplayModeSupport::SupportedWithConversion:
                            std::cout << std::setw(px_fmt_len) << px_fmt_str << "*  ";
                            break;
                    }
                }
                std::cout << "\n";
            }
            
            std::cout << "  * Indicates that the display mode is only supported with conversion.";
            
            return 0;
            
        } else if (command == "--list-attributes") {
            std::cout << "List of known attributes: \n";
            for (const auto attribute: AttributeID_::Values) {
                std::cout << "  - " << to_string(attribute)
                          << " (" << to_string(AttributeID_::get_value_type(attribute))
                          << "): " << AttributeID_::get_explanation(attribute) << "\n";
            }
            
            return 0;
            
        } else if (command == "--query-attribute") {
            if (argc < 4) {
                std::cerr << "Error: Attribute name not specified\n\n";
                show_help();
                exit(-1);
            }
            
            const auto attribute = DeckLink::to_AttributeID(argv[3]);
            std::cout << "[" << device.get_long_name() << "] :: "
                      << to_string(attribute) << " = " << get_attribute(device, attribute) << "\n";
            
            return 0;
        } else if (command == "--query-all-attributes") {
            for (const auto attribute: AttributeID_::Values) {
                std::string attribute_value = get_attribute(device, attribute);
                
                // If the attribute is empty something failed indicating that the attribute is not
                // supported on this device
                if (!attribute_value.empty()) {
                    std::cout << "[" << device.get_long_name() << "] :: "
                              << to_string(attribute) << " = " << attribute_value << "\n";
                }
                std::cout << "\n";
            }
        
            return 0;
        }
    
        std::cerr << "\nUnknown command: " << argv[2] << "\n";
        show_help();
        
    } catch (const DeckLink::decklink_driver_error& ex) {
        print_driver_error_details(ex);
        
    } catch (const DeckLink::runtime_error& ex) {
        std::cout << "\n"
                  << "An error occurred! \n"
                  << boost::diagnostic_information(ex) << "\n";
    }
    
    return -1;
}


