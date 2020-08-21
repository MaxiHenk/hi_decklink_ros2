//
// Created by tibo on 16/01/18.
//

#pragma once

namespace DeckLink {
    
    struct IDeckLinkHandleDeleter {
        
        /**
         * Custom deleter used with the std::unique_ptr to ensure that the DeckLink SDK resource is correctly freed.
         *
         * @tparam DeckLinkImpl A IDeckLink... type of object
         * @param ptr A pointer to the IDeckLink... to release
         */
        template<typename DeckLinkImpl>
        void operator()(DeckLinkImpl *ptr) {
            ptr->Release();
        }
    };
    
}
