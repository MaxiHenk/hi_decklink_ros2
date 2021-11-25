//
// Created by tibo on 18/01/18.
//

#pragma once

#include <functional>

#include "decklink_sdk/DeckLinkAPI.h"

#include "libdecklink/types.hpp"
#include "libdecklink/errors.hpp"
#include "libdecklink/dummy_reference_counted.hpp"

namespace DeckLink {
    
    class Device;
    
    class NotificationCallback: public IDeckLinkNotificationCallback {
        friend class Device;
        
    public: /* Types */
        
        using status_changed_cb_t = std::function<void(StatusID, uint64_t)>;
        
    public:
        NotificationCallback() = default;
        
        NotificationCallback(const status_changed_cb_t&& cb);
        
        HRESULT Notify(BMDNotifications topic, uint64_t param1, uint64_t param2) override;
    
        void set_on_status_changed_cb(const status_changed_cb_t&& on_status_changed_cb);
    
        DUMMY_REF_COUNT_METHODS
        
    protected:
    
    private:
        
        Device* _parent_device;
        status_changed_cb_t _on_status_changed_cb;
    
    };
    
}
