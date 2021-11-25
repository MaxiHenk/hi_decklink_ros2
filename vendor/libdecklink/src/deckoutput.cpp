//
// Created by tibo and paola on 21/02/18.
//

#include <iostream>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include "libdecklink/types.hpp"
#include "libdecklink/device.hpp"
#include "libdecklink/video_output_frame.hpp"

#include "handle_errors.hpp"

// Disable a bunch of warnings in stb_image.h
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdouble-promotion"
#pragma GCC diagnostic ignored "-Wconstant-conversion"
#pragma GCC diagnostic ignored "-Wunused-parameter"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#pragma GCC diagnostic pop

// TODO: proper program options

using namespace DeckLink;

struct Image {
    int width = -1, height = -1, depth = -1;
    unsigned char *data = nullptr;
};

void show_usage(po::options_description opts) {
    std::cout << "deckoutput [--list | --device :device-name: | --image-format :format: | \n"
              << "            --source :image-file: ]\n"
              << "\n"
              << "Write the test image to the specified DeckLink device.\n"
              << "\n"
              << "USAGE"
              << opts << " \n";
}

int main(int argc, char **argv) {
    
    po::positional_options_description positional_opts;
    positional_opts.add("device", 1);
    
    po::options_description main_opts("Options");
    main_opts.add_options()
    ("help", "Show this help.")
    (
        "list,l",
        po::bool_switch(),
        "The configuration file to use. See the README for more information"
    ) (
        "device,d",
        po::value<std::string>(),
        "The file in which to save the intrinsic camera calibration parameters"
    ) (
        "image-format,i",
        po::value<std::string>(),
        "The image format to use when outputting the image. See `deckquery` for a complete list "
    ) (
        "source-image,s",
        po::value<std::string>(),
        "The image to output to the device"
    );
    
    po::variables_map cmdline;
    try {
        auto parsed_opts = po::command_line_parser(argc, argv)
            .options(main_opts)
            .positional(positional_opts)
            .run();
        
        po::store(parsed_opts, cmdline);
        if (cmdline.count("help")) {
            std::cout << main_opts << std::endl;
            exit(-1);
        }
        
        po::notify(cmdline);
    } catch (po::required_option& ex) {
        std::cerr << "Error: required option \"" << ex.get_option_name()
                  << "\" missing from command line arguments" << std::endl;
        
        show_usage(main_opts);
        exit(-1);
    } catch (po::unknown_option &ex) {
        std::cerr << "Error: unknown option \"" << ex.get_option_name() << "\"\n" << main_opts
                  << std::endl;
        
        show_usage(main_opts);
        exit(-1);
    }

    try {
        if (cmdline["list"].as<bool>()) {
            const auto devices = Device::List();

            std::cout << "Listing detected devices:\n";
            for (const auto &device: devices) {
                std::cout << " - " << device.get_display_name()
                          << " ( " << device.get_vendor_name()
                          << " " << device.get_model_name() << " )\n";
            }
            return 0;
        }
        
        if (!cmdline.count("device")) {
            std::cerr << "If the <--list> option is not specified the <--device> option is mandatory.\n";
            show_usage(main_opts);
            exit(-1);
        }
        
        const std::string device_name = cmdline["device"].as<std::string>();
        auto has_device = Device::Get(device_name);
        if (!has_device) {
            std::cerr << "Error: Unable to locate a device named: " << device_name << ".\n";
            return -1;
        }

        auto &device = *has_device;

        // check if the selected display mode is supported
        DisplayMode display_mode;
        const auto pixel_format = DeckLink::PixelFormat::BGRA_8Bit;
        
        if (!cmdline.count("image-format")) {
            const auto display_modes = device.output().get_all_supported_display_modes();
            display_mode = display_modes.front();
            
            std::cout << "No display mode specified. Using first supported display mode: "
                      << display_mode.get_name() << "\n";
        } else {
            const auto image_format = DeckLink::to_ImageFormat(cmdline["image-format"].as<std::string>());
    
            const auto res = device.output().get_display_mode(image_format);
            if (!res) {
                std::cerr << "Image format \"" << to_string(image_format) << "\" is not "
                          << "supported by this device" << std::endl;
                return -1;
            }
    
            display_mode = *res;
        }
    
        std::string source_image_file = "keyer_overlay.png";
        if (cmdline.count("source-image")) {
            source_image_file = cmdline["source-image"].as<std::string>();
        }
        
        device.output()
              .enable(display_mode, PixelFormat::BGRA_8Bit, VideoOutputFlags::Default)
              .enable_keyer()
              .set_opacity(uint8_t(100));
    
        Image source_image;
        source_image.data = stbi_load(
            source_image_file.c_str(),
            &source_image.width,
            &source_image.height,
            &source_image.depth, 0
        );
    
        if (!source_image.data) {
            BOOST_THROW_EXCEPTION(runtime_error() << errmsg("Unable to load the overlaying image"));
        };
    
        // Make sure that the image we loaded is the right size for the format
        if (source_image.depth != 4) {
            BOOST_THROW_EXCEPTION(runtime_error() << errmsg("Expected 4 channel (RGBA) image"));
        }
    
        if (source_image.width != display_mode.get_width()) {
            BOOST_THROW_EXCEPTION(runtime_error()
                << errmsg("Invalid image size: width. For \"" + to_string(display_mode.get_image_format()) +"\" images"
                    + " width should be " + std::to_string(display_mode.get_width()) + " pixels"
                )
            );
        }
    
        if (source_image.height != display_mode.get_height()) {
            BOOST_THROW_EXCEPTION(runtime_error()
                << errmsg("Invalid image size: height. For \"" + to_string(display_mode.get_image_format()) +"\" images"
                    + " height should be " + std::to_string(display_mode.get_height()) + " pixels"
                )
            );
        }
        
        auto frame = device.output().create_video_frame(display_mode, pixel_format);

        frame.load(source_image.data, row_bytes(pixel_format, source_image.width) * source_image.height);
        device.output()
              .display_video_frame(frame);

        char c;
        while (true) {
            std::cout << "Press 'q' to quit ..." << std::endl;
            std::cin >> c;

            if (c == 'q') break;
        }

        device.output().disable();

    } catch (const DeckLink::decklink_driver_error &ex) {
        print_driver_error_details(ex);

    } catch (const DeckLink::runtime_error &ex) {
        std::cout << "\n"
                  << "An error occurred! \n"
                  << boost::diagnostic_information(ex) << "\n";
    }

    return 0;

}
