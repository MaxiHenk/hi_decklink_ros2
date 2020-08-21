//
// Created by tibo on 22/03/18.
//

#pragma once

#include <libdecklink/errors.hpp>

inline void print_driver_error_details(const DeckLink::decklink_driver_error& ex) {
    std::cout << "\nA low-level DeckLink Driver command failed: \n";
    
    if (const auto device = boost::get_error_info<DeckLink::decklink_device>(ex)) {
        std::cout << "  DeckLink::Device     : " << *device << "\n";
    }
    
    if (const auto command = boost::get_error_info<DeckLink::decklink_command>(ex)) {
        std::cout << "  Command Name         : " << *command << "\n";
    }
    
    if (const auto return_code = boost::get_error_info<DeckLink::decklink_error_code>(ex)) {
        std::cout << "  Command Return Status: " << to_string(*return_code) << "\n";
    }
    
    if (const auto message = boost::get_error_info<DeckLink::errmsg>(ex)) {
        std::cout << "  Command Error Message: " << *message << "\n";
    }
    
    if (const auto attribute = boost::get_error_info<DeckLink::decklink_attribute_flag>(ex)) {
        std::cout << "  DeckLink Attribute: " << to_string(*attribute) << "\n";
    }
    
    std::cout << "\n === Complete Error Information === \n"
              << boost::diagnostic_information(ex, true) << "\n";
}
