//
// Created by tibo on 12/01/18.
//

#pragma once

#include <boost/exception/all.hpp>

#include "libdecklink/types.hpp"

namespace DeckLink {
    
    struct exception_base: virtual std::exception, virtual boost::exception {};
    
    struct runtime_error: virtual exception_base {};
    typedef boost::error_info<struct tag_errmsg, const std::string> errmsg;
    
    struct decklink_driver_error: virtual runtime_error {};
    
    typedef boost::error_info<struct tag_decklink_method_name, const std::string> decklink_method;
    typedef boost::error_info<struct tag_decklink_device, const std::string> decklink_device;
    typedef boost::error_info<struct tag_decklink_command, const std::string> decklink_command;
    typedef boost::error_info<struct tag_decklink_error_code, const HResult> decklink_error_code;
    typedef boost::error_info<struct tag_decklink_attribute_flag, const AttributeID> decklink_attribute_flag;
    typedef boost::error_info<struct tag_decklink_configuration_flag, const ConfigurationID> decklink_configuration_flag;
    typedef boost::error_info<struct tag_decklink_status_flag, const StatusID> decklink_status_flag;
    
    
    struct out_of_range_error: virtual runtime_error {};
    struct type_error: virtual runtime_error {};
    struct null_pointer_error: virtual runtime_error {};
    struct not_implemented_error: public runtime_error {};
    
    struct key_error: virtual runtime_error{};
    typedef boost::error_info<struct tag_errkey, const std::string> errkey;
    
    struct io_error: virtual runtime_error {};
    
    struct file_error: virtual io_error {};
    typedef boost::error_info<struct tag_errfile, const std::string> errfile;
    
    struct not_a_file_error: virtual file_error {};
    struct file_open_error: virtual file_error {};
    struct file_read_error: virtual file_error {};
    struct file_parse_error: virtual file_error {};
    
    struct empty_image_error: virtual runtime_error {};
    
    struct size_error: virtual runtime_error {};
    typedef boost::error_info<struct tag_size_expected, const size_t> expected_size;
    typedef boost::error_info<struct tag_size_actual, const size_t> actual_size;}

