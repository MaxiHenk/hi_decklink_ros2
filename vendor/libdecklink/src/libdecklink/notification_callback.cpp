//
// Created by tibo on 18/01/18.
//

#include "libdecklink/notification_callback.hpp"

namespace DeckLink {
    
    NotificationCallback::NotificationCallback(
        const status_changed_cb_t&& cb
    ) {
        set_on_status_changed_cb(std::move(cb));
    }
    
    
    HRESULT NotificationCallback::Notify(
        BMDNotifications topic,
        uint64_t param1,
        uint64_t param2
    ) {
        // Topic can be status or preferences. We are not interested in the latter
        if (topic != bmdStatusChanged)
            return S_OK;
        
        if (_on_status_changed_cb) {
            _on_status_changed_cb(static_cast<StatusID>(param1), param2);
        }
            
        return S_OK;
    }
    
    
    void NotificationCallback::set_on_status_changed_cb(
        const status_changed_cb_t&& on_status_changed_cb
    ) {
        _on_status_changed_cb = on_status_changed_cb;
    }
    
}
