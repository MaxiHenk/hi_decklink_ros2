//
// Created by tibo on 18/01/18.
//


#include <iostream>

#include <libdecklink/device.hpp>
#include <libdecklink/types.hpp>

#include "handle_errors.hpp"

using std::to_string;
using namespace DeckLink;

void show_help() {
    std:: cout << "deckmonitor [--list | device-name]\n"
               << "\n"
               << "Show the status change notifications of the specified BlackMagic DeckLink video capture card in real time.\n"
               << " \n"
               << "USAGE:\n"
               << "  --list, -l  List all the detected DeckLink devices\n";
               
}

void on_status_changed(StatusID status_id, uint64_t param2) {
    std::cout << " - " << to_string(status_id) << " changed, is now " << param2 << "\n";
}

int main(int argc, char** argv) {
    if (argc != 2) {
        show_help();
        return -1;
    }
    
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
    
    try {
        auto has_device = Device::Get(device_name);
        if (!has_device) {
            std::cerr << "Error: Unable to locate device named: " << device_name << ".\n";
            return -1;
        }
    
        auto& device = *has_device;
    
        // NotificationCallback notification_cb(on_status_changed);
        NotificationCallback notification_cb;
        notification_cb.set_on_status_changed_cb(on_status_changed);
        device.set_notifications_callback(std::move(notification_cb));
    
        std::cout << "Press <ENTER> to exit.\n";
        std::cout << "Monitoring " << device.get_long_name() << "  for status changes .. \n";
        std::getchar();
    
    } catch (const DeckLink::decklink_driver_error &ex) {
        print_driver_error_details(ex);
    
    } catch (const DeckLink::runtime_error &ex) {
        std::cout << "\n"
                  << "An error occurred! \n"
                  << boost::diagnostic_information(ex) << "\n";
    }
    
    
    return 0;
}
