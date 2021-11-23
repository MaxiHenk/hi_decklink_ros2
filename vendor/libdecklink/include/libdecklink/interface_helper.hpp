//
// Created by tibo on 16/01/18.
//

#pragma once


#include "decklink_sdk/DeckLinkAPI.h"
#include "libdecklink/errors.hpp"
#include "libdecklink/decklink_handle_deleter.hpp"

namespace DeckLink {
    
    template<typename InterfaceType>
    void load_interface(
        IDeckLink *device,
        REFIID interface_id,
        std::unique_ptr<InterfaceType, IDeckLinkHandleDeleter>& dest,
        const std::string& interface_name
    ) {
        if (!device) {
            BOOST_THROW_EXCEPTION(null_pointer_error() << errmsg("device was null"));
        }
        
        void *interface_ptr = nullptr;
        const HRESULT result = device->QueryInterface(interface_id, &interface_ptr);
        
        if (result != S_OK) {
            BOOST_THROW_EXCEPTION(decklink_driver_error()
                << errmsg("Unable to retrieve " + interface_name + " interface")
                << decklink_error_code(static_cast<HResult>(result))
            );
        }
        
        assert(interface_ptr);
        dest.reset(static_cast<InterfaceType*>(interface_ptr));
    }
}
