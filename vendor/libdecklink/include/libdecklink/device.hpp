//
// Created by tibo on 12/01/18.
//

#pragma once

#include <vector>
#include <memory>

#include <boost/optional.hpp>
#include <boost/noncopyable.hpp>

#include "decklink_sdk/DeckLinkAPI.h"
#include "libdecklink/errors.hpp"
#include "libdecklink/types.hpp"
#include "libdecklink/decklink_handle_deleter.hpp"
#include "libdecklink/device_input_interface.hpp"
#include "libdecklink/device_output_interface.hpp"
#include "libdecklink/notification_callback.hpp"

namespace DeckLink {
    
    class Device: private boost::noncopyable {
    
        friend class DeviceInputInterface;
        friend class DeviceOutputInterface;
        
    public: /* Static Methods */
        
        static std::vector<Device> List();
        
        static boost::optional<Device> Get(const std::string& name);
    
    public: /* Methods */
        
        /**
         * Default constructor.
         *
         * This constructs a dummy Device that cannot actually be used for anything. It just serves
         * to provide a default constructor.
         */
        Device() {}
        
        explicit Device(IDeckLink* device_ptr);
        
        Device(Device&& src) noexcept;
        
        Device& operator= (Device&& rhs) noexcept;
        
        virtual ~Device();
        
        // Configuration methods
  
        // TODO : Trash optionals - we should always get a value. Unless maybe we swallow not implemented errors
        
        std::string get_model_name() const;
        std::string get_display_name() const;
        std::string get_vendor_name() const;
        
        std::string get_long_name() const;
        
        /// Read a boolean attribute from the card
        boost::optional<bool> get_bool_attribute(const AttributeID flag) const;
        void set_bool_attribute(const AttributeID flag, bool value);
        
        // Read an integer (\c int64_t)  attribute from the card
        boost::optional<int64_t> get_int_attribute(const AttributeID flag) const;
        void set_int_attribute(const AttributeID flag, int value);
    
        // Read a floating point (\c double) attribute from the card
        boost::optional<double> get_float_attribute(const AttributeID flag) const;
        void  set_float_attribute(const AttributeID flag, float value);
        
        /// Read a string attribute from the card
        boost::optional<std::string> get_string_attribute(const AttributeID flag) const;
        void       set_string_attribute(const AttributeID flag, std::string value);
    
        /// Read a boolean configuration value from the card
        boost::optional<bool> get_bool_configuration(const ConfigurationID flag) const;
        void set_bool_configuration(const ConfigurationID flag, bool value);
    
        /// Read an integer (\c int64_t) configuration value from the card
        boost::optional<int64_t> get_int_configuration(const ConfigurationID flag) const;
        void set_int_configuration(const ConfigurationID flag, int value);
    
        /// Read a floating point (\c double) configuration value from the card
        boost::optional<double> get_float_configuration(const ConfigurationID flag) const;
        void  set_float_configuration(const ConfigurationID flag, float value);
    
        /// Read a string configuration value from the card
        boost::optional<std::string> get_string_configuration(const ConfigurationID flag) const;
        void       set_string_configuration(const ConfigurationID flag, std::string value);
    
        /// Read a boolean status value from the card
        boost::optional<bool> get_bool_status(const StatusID flag) const;
        void set_bool_status(const StatusID flag, bool value);
    
        /// Read an integer (\c int64_t) status value from the card
        boost::optional<int64_t> get_int_status(const StatusID flag) const;
        void set_int_status(const StatusID flag, int value);
        
        // Important options - easy access
        
        bool supports_input_format_detection() const;
        
        // Notifications
        
        void set_notifications_callback(NotificationCallback&& cb);
        void clear_notification_callback();

        // Input / Output Interfaces
        
        /// Retrieve the #DeviceInputInterface for this device
        DeviceInputInterface& input();
    
        /// Retrieve the DeckLink::DeviceOutputInterface for this device
        DeviceOutputInterface& output();
    
        /**
         * Get a raw pointer to the underlying device.
         * This allows you to call additional methods that may not be wrapped by the Device class.
         *
         * @return A pointer to the device
         */
        IDeckLink* get_raw_device();



    protected: /* Methods */
        
        
        /**
         * Load the AttributesInterface.
         *
         * The Attributes interface is required to query attributes on the card and is used by
         * the `get_xxx_attribute` family of functions.
         *
         * @note This function is marked const even though it really isn't. The \c _attributes_impl
         * interface is lazy loaded so every time we call a \c *_attributes method we have to check
         * if the interface has been loaded and if not load it.
         * Since we want the \c get_*_attribute methods to be const we need this method to be const.
         */
        void load_attributes_interface() const;
        
        void load_configuration_interface() const;
        
        void load_status_interface() const;
        
        void load_notification_interface() const;
    
        /**
         * Centralise handling of the `AttributesInterface`, `ConfigurationInterface` and
         * `StatusInterface` return codes in a single place.
         *
         * @tparam AttributeType The type of value to return (\c bool, \c int64_t, ...)
         *
         * @param result The return status of the \c GetXXX method
         * @param attribute_value The value to return if the returc code was good
         *
         * @return The \p attribute_value if there were no errors
         */
        template <typename AttributeType>
        boost::optional<AttributeType> parse_configuration_query_result(
            HResult result,
            AttributeType attribute_value
        ) const;
        

    private: /* Member Variables */

        DeviceInputInterface _input_interface;
        DeviceOutputInterface _output_interface;
        
        bool _notification_callback_subscribed = false;
        NotificationCallback _notification_callback;
        
        mutable std::unique_ptr<IDeckLink, IDeckLinkHandleDeleter> _device_impl;
        mutable std::unique_ptr<IDeckLinkStatus, IDeckLinkHandleDeleter> _status_impl;
        mutable std::unique_ptr<IDeckLinkAttributes, IDeckLinkHandleDeleter> _attributes_impl;
        mutable std::unique_ptr<IDeckLinkConfiguration, IDeckLinkHandleDeleter> _configuration_impl;
        mutable std::unique_ptr<IDeckLinkNotification, IDeckLinkHandleDeleter> _notification_impl;
        
    };
    
    
    template <typename AttributeType>
    boost::optional<AttributeType> Device::parse_configuration_query_result(
        HResult result,
        AttributeType attribute_value
    ) const {
        switch (result) {
            case HResult::Ok:
                return attribute_value;
            
            case HResult::NotImplemented:
                return { };
            
            case HResult::InvalidArg :
                BOOST_THROW_EXCEPTION(decklink_driver_error()
                    << decklink_error_code(result)
                    << errmsg("The option exists but was queried with the wrong type, e.g. queried an Int attribute with `get_attribute_flag`")
                );
            
            case HResult::Fail:
                BOOST_THROW_EXCEPTION(decklink_driver_error()
                    << decklink_error_code(result)
                    << errmsg("The query failed for an unknown reason")
                );
                
            default:
                BOOST_THROW_EXCEPTION(decklink_driver_error()
                    << decklink_error_code(result)
                    << errmsg("An unknown error occurred")
                );
        }
    }
    
}
