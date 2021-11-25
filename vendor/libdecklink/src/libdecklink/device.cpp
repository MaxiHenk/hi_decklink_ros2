//
// Created by tibo on 12/01/18.
//

#include "libdecklink/device.hpp"
#include "libdecklink/interface_helper.hpp"

namespace DeckLink {
    
    //
    // Static Methods
    //
    
    std::vector<Device> Device::List() {
        std::vector<Device> devicelist;
        
        IDeckLinkIterator* it = CreateDeckLinkIteratorInstance();
        if (!it) {
            BOOST_THROW_EXCEPTION(runtime_error()
                << errmsg("Unable to create a 'DeckLinkIterator' instance. Are the libdecklink drivers correctly installed ?")
            );
        }
        
        IDeckLink *device_ptr;
        while (it->Next(&device_ptr) == S_OK) {
            if (!device_ptr) {
                BOOST_THROW_EXCEPTION(null_pointer_error() << errmsg("device_ptr is null"));
            }
            devicelist.emplace_back(device_ptr);
        }
    
        if (devicelist.empty()) {
            BOOST_THROW_EXCEPTION(runtime_error()
                << errmsg("No devices found on this machine.")
            );
        }
        
        it->Release();
    
        return devicelist;
    }
    
    boost::optional<Device> Device::Get(const std::string& name) {
        std::vector<Device> devices = Device::List();
        
        for (auto& device: devices) {
            if (device.get_display_name() == name) {
                return std::move(device);
            }
        }
        
        return {};
    }
    
    
    //
    // Public Class Methods
    //
    
    Device::Device(IDeckLink* device_ptr)
        : _device_impl(device_ptr)
        , _input_interface(this)
        , _output_interface(this)

    {
        if (!device_ptr) {
            BOOST_THROW_EXCEPTION(null_pointer_error()
                << errmsg("Cannot create a device from a null pointer")
            );
        }
    }
    
    Device::Device(Device&& src) noexcept {
        *this = std::move(src);
    }
    
    Device& Device::operator=(Device&& rhs) noexcept {
        if (this != &rhs) {
            if (_notification_callback_subscribed) {
                BOOST_THROW_EXCEPTION(runtime_error()
                    << errmsg("Cannot std::move DeckLink::Device whilst NotificationCallback is subscribed. Unsubscribe it first")
                );
            }
            
            _input_interface = std::move(rhs._input_interface);
            _input_interface._parent_device = this;

            _output_interface = std::move(rhs._output_interface);
            _output_interface._parent_device = this;
            
            _notification_callback._parent_device = this;
            
            _device_impl = std::move(rhs._device_impl);
            _status_impl = std::move(rhs._status_impl);
            _attributes_impl = std::move(rhs._attributes_impl);
            _configuration_impl = std::move(rhs._configuration_impl);
            _notification_impl = std::move(rhs._notification_impl);
        }
        
        return *this;
    }
    
    
    Device::~Device() {
        if (_notification_callback_subscribed)
            clear_notification_callback();
    }
    
    
    std::string Device::get_model_name() const {
        if (const auto result = get_string_attribute(AttributeID::ModelName))
            return *result;
        
        return "N/A";
    }
    
    
    std::string Device::get_display_name() const {
        if (const auto result = get_string_attribute(AttributeID::DisplayName))
            return *result;
    
        return "N/A";
    }
    
    
    std::string Device::get_vendor_name() const {
        if (const auto result = get_string_attribute(AttributeID::VendorName))
            return *result;
    
        return "N/A";
    }
    
    
    std::string Device::get_long_name() const {
        return get_display_name() + " (" + get_vendor_name() + " " + get_model_name() +")";
    }
    
    
    boost::optional<bool> Device::get_bool_attribute(const AttributeID flag) const {
        if (!_attributes_impl) load_attributes_interface();
        
        try {
            bool attribute_value = false;
            const auto raw_flag = static_cast<BMDDeckLinkAttributeID>(flag);
            HRESULT result = _attributes_impl->GetFlag(raw_flag, &attribute_value);
            
            return parse_configuration_query_result(static_cast<HResult>(result), attribute_value);
        } catch (const decklink_driver_error &err) {
            err << decklink_attribute_flag(flag);
            throw;
        }
    }
    
    
    void Device::set_bool_attribute(const AttributeID /* flag */, bool /* value */) {
        BOOST_THROW_EXCEPTION(not_implemented_error());
    }
    
    
    boost::optional<int64_t> Device::get_int_attribute(const AttributeID flag) const {
        if (!_attributes_impl) load_attributes_interface();
    
        try {
            int64_t attribute_value = false;
            const auto raw_flag = static_cast<BMDDeckLinkAttributeID>(flag);
            HRESULT result = _attributes_impl->GetInt(raw_flag, &attribute_value);
        
            return parse_configuration_query_result(static_cast<HResult>(result), attribute_value);
        } catch (const decklink_driver_error &err) {
            err << decklink_attribute_flag(flag);
            throw;
        }
    }
    
    
    void Device::set_int_attribute(const AttributeID /* flag */, int /* value */) {
        BOOST_THROW_EXCEPTION(not_implemented_error());
    }
    
    
    boost::optional<double> Device::get_float_attribute(const AttributeID flag) const {
        if (!_attributes_impl) load_attributes_interface();
    
        try {
            double attribute_value = false;
            const auto raw_flag = static_cast<BMDDeckLinkAttributeID>(flag);
            HRESULT result = _attributes_impl->GetFloat(raw_flag, &attribute_value);
        
            return parse_configuration_query_result(static_cast<HResult>(result), attribute_value);
        } catch (const decklink_driver_error &err) {
            err << decklink_attribute_flag(flag);
            throw;
        }
    }
    
    
    void Device::set_float_attribute(const AttributeID /* flag */, float /* value */) {
        BOOST_THROW_EXCEPTION(not_implemented_error());
    }
    
    
    boost::optional<std::string> Device::get_string_attribute(const AttributeID flag) const {
        if (!_attributes_impl) load_attributes_interface();
    
        try {
            char* buf = nullptr;
            const auto raw_flag = static_cast<BMDDeckLinkAttributeID>(flag);
            HRESULT result = _attributes_impl->GetString(raw_flag, const_cast<const char**>(&buf));

            std::string result_str = buf? std::string(buf) : std::string();
            
            // VendorName is a static string that should not be freed
            if (result == S_OK && flag != AttributeID::VendorName) free(buf);
            
            return parse_configuration_query_result(static_cast<HResult>(result), result_str);
        } catch (const decklink_driver_error &err) {
            err << decklink_attribute_flag(flag);
            throw;
        }
    }
    
    
    void Device::set_string_attribute(const AttributeID /* flag */, std::string /* value */) {
        BOOST_THROW_EXCEPTION(not_implemented_error());
    }
    
    
    boost::optional<bool> Device::get_bool_configuration(const ConfigurationID flag) const {
        if (!_configuration_impl) load_configuration_interface();
    
        try {
            bool configuration_value = false;
            const auto raw_flag = static_cast<BMDDeckLinkConfigurationID >(flag);
            HRESULT result = _configuration_impl->GetFlag(raw_flag, &configuration_value);
        
            return parse_configuration_query_result(static_cast<HResult>(result), configuration_value);
        } catch (const decklink_driver_error &err) {
            err << decklink_configuration_flag(flag);
            throw;
        }
    }
    
    
    void Device::set_bool_configuration(const ConfigurationID /* flag */, bool /* value */) {
        BOOST_THROW_EXCEPTION(not_implemented_error());
    }
    
    
    boost::optional<int64_t> Device::get_int_configuration(const ConfigurationID flag) const {
        if (!_configuration_impl) load_configuration_interface();
    
        try {
            int64_t configuration_value = false;
            const auto raw_flag = static_cast<BMDDeckLinkConfigurationID >(flag);
            HRESULT result = _configuration_impl->GetInt(raw_flag, &configuration_value);
        
            return parse_configuration_query_result(static_cast<HResult>(result), configuration_value);
        } catch (const decklink_driver_error &err) {
            err << decklink_configuration_flag(flag);
            throw;
        }
    }
    
    
    void Device::set_int_configuration(const ConfigurationID /* flag */, int /* value */) {
        BOOST_THROW_EXCEPTION(not_implemented_error());
    }
    
    
    boost::optional<double> Device::get_float_configuration(const ConfigurationID flag) const {
        if (!_configuration_impl) load_configuration_interface();
    
        try {
            double configuration_value = false;
            const auto raw_flag = static_cast<BMDDeckLinkConfigurationID >(flag);
            HRESULT result = _configuration_impl->GetFloat(raw_flag, &configuration_value);
        
            return parse_configuration_query_result(static_cast<HResult>(result), configuration_value);
        } catch (const decklink_driver_error &err) {
            err << decklink_configuration_flag(flag);
            throw;
        }
    }
    
    
    void Device::set_float_configuration(const ConfigurationID /* flag */, float /* value */) {
        BOOST_THROW_EXCEPTION(not_implemented_error());
    }
    
    
    boost::optional<std::string> Device::get_string_configuration(const ConfigurationID flag) const {
        if (!_configuration_impl) load_attributes_interface();
    
        try {
            char* buf = nullptr;
            const auto raw_flag = static_cast<BMDDeckLinkConfigurationID >(flag);
            HRESULT result = _configuration_impl->GetString(raw_flag, const_cast<const char**>(&buf));
        
            std::string result_str = buf? std::string(buf) : std::string();
            if (result == S_OK) free(buf);
    
            return parse_configuration_query_result(static_cast<HResult>(result), result_str);
        } catch (const decklink_driver_error &err) {
            err << decklink_configuration_flag(flag);
            throw;
        }
    }
    
    
    void Device::set_string_configuration(const ConfigurationID /* flag */, std::string /* value */) {
        BOOST_THROW_EXCEPTION(not_implemented_error());
    }
    
    
    boost::optional<bool> Device::get_bool_status(const StatusID flag) const {
        if (!_status_impl) load_status_interface();
    
        try {
            bool status_value = false;
            const auto raw_flag = static_cast<BMDDeckLinkStatusID >(flag);
            HRESULT result = _status_impl->GetFlag(raw_flag, &status_value);
        
            return parse_configuration_query_result(static_cast<HResult>(result), status_value);
        } catch (const decklink_driver_error &err) {
            err << decklink_status_flag(flag);
            throw;
        }
    }
    
    
    void Device::set_bool_status(const StatusID /* flag */, bool /* value */) {
        BOOST_THROW_EXCEPTION(not_implemented_error());
    }
    
    
    boost::optional<int64_t> Device::get_int_status(const StatusID flag) const {
        if (!_status_impl) load_status_interface();
    
        try {
            int64_t status_value = false;
            const auto raw_flag = static_cast<BMDDeckLinkStatusID>(flag);
            HRESULT result = _status_impl->GetInt(raw_flag, &status_value);
        
            return parse_configuration_query_result(static_cast<HResult>(result), status_value);
        } catch (const decklink_driver_error &err) {
            err << decklink_status_flag(flag);
            throw;
        }    }
    
    
    void Device::set_int_status(const StatusID /* flag */, int /* value */) {
        BOOST_THROW_EXCEPTION(not_implemented_error());
    }
    
    
    //
    // Named configuration values
    //
    
    bool Device::supports_input_format_detection() const {
        const auto result = get_bool_attribute(AttributeID::SupportsInputFormatDetection);
        
        // The optional was not empty and the contained value was true
        return (result and *result);
    }
    
    
    //
    // Callbacks
    //
    
    void Device::set_notifications_callback(NotificationCallback&& cb) {
        if (!_notification_impl) load_notification_interface();
        
        // FIXME: This is going to break horribly if we ever move after setting the callback
        // since at that point the driver is calling into uninitialised memory
        _notification_callback = cb;
        _notification_callback._parent_device = this;
        
        _notification_callback_subscribed = true;
        const HRESULT result = _notification_impl->Subscribe(bmdStatusChanged, &_notification_callback);
        
        if (result !=  S_OK) {
            _notification_callback_subscribed = false;
            if (result == E_INVALIDARG) {
                BOOST_THROW_EXCEPTION(null_pointer_error()
                    << errmsg("address  of callback was null. This is impossible!")
                    << decklink_command("IDeckLinkNotification::Subscribe")
                );
            } else {
                BOOST_THROW_EXCEPTION(decklink_driver_error()
                    << errmsg(result == E_FAIL? "Operation failed" : "An unexpected error occurred")
                    << decklink_command("IDeckLinkNotification::Subscribe")
                    << decklink_error_code(static_cast<HResult>(result))
                );
            }
        }
    }
    
    
    void Device::clear_notification_callback() {
        if (!_notification_callback_subscribed) return;
    
        const HRESULT result = _notification_impl->Unsubscribe(bmdStatusChanged, &_notification_callback);
    
        if (result !=  S_OK) {
            _notification_callback_subscribed = false;
            if (result == E_INVALIDARG) {
                BOOST_THROW_EXCEPTION(null_pointer_error()
                    << errmsg("address  of callback was null. This is impossible!")
                    << decklink_command("IDeckLinkNotification::Unsubscribe")
                );
            } else {
                BOOST_THROW_EXCEPTION(decklink_driver_error()
                    << errmsg(result == E_FAIL? "Operation failed" : "An unexpected error occurred")
                    << decklink_command("IDeckLinkNotification::Unsubscribe")
                    << decklink_error_code(static_cast<HResult>(result))
                );
            }
        }
    }
    
    //
    // Input / Output Interface
    //
    
    DeviceInputInterface& Device::input() {
        return _input_interface;
    }
    
    //
    // Advanced
    //
    
    IDeckLink *Device::get_raw_device() {
        return _device_impl.get();
    }

    DeviceOutputInterface& Device::output() {
        return _output_interface;
    }
    
    //
    // Protected Class Methods
    //
    
    
    void Device::load_attributes_interface() const {
        if (!_device_impl) {
            BOOST_THROW_EXCEPTION(null_pointer_error()
                << errmsg("DeckLink::Device has not been initialised with a DeckLink device pointer yet")
            );
        }
        
        load_interface(
            _device_impl.get(), IID_IDeckLinkAttributes, _attributes_impl, "attributes"
        );
    }
    
    
    void Device::load_configuration_interface() const {
        if (!_device_impl) {
            BOOST_THROW_EXCEPTION(null_pointer_error()
                << errmsg("DeckLink::Device has not been initialised with a DeckLink device pointer yet")
            );
        }
    
        load_interface(
            _device_impl.get(), IID_IDeckLinkConfiguration, _configuration_impl, "configuration"
        );
    }
    
    
    void Device::load_status_interface() const {
        if (!_device_impl) {
            BOOST_THROW_EXCEPTION(null_pointer_error()
                << errmsg("DeckLink::Device has not been initialised with a DeckLink device pointer yet")
            );
        }
    
        load_interface(
            _device_impl.get(), IID_IDeckLinkStatus, _status_impl, "status"
        );
    }
    
    
    void Device::load_notification_interface() const {
        if (!_device_impl) {
            BOOST_THROW_EXCEPTION(null_pointer_error()
                << errmsg("DeckLink::Device has not been initialised with a DeckLink device pointer yet")
            );
        }
    
        load_interface(
            _device_impl.get(), IID_IDeckLinkNotification, _notification_impl, "notification"
        );
    }
}
