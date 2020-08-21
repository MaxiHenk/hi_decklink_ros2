//
// Created by tibo and paola on 21/02/18.
//

#include <libdecklink/device.hpp>

#include "libdecklink/interface_helper.hpp"
#include "libdecklink/video_output_frame.hpp"

namespace DeckLink {

    VideoOutputFrame::VideoOutputFrame(IDeckLinkMutableVideoFrame *raw_output_frame)
        : VideoFrame(static_cast<IDeckLinkVideoFrame *>(raw_output_frame))
        , _raw_output_frame(raw_output_frame)
    {
        // Do nothing - Specifically we shouldn't call AddRef since we should be the only
        // owner of the pointer
    }

    VideoOutputFrame::~VideoOutputFrame() {
        _raw_output_frame->Release();
    }
    
    
    VideoOutputFrame& VideoOutputFrame::load(const std::vector<unsigned char>& data) {
        if (data.empty()) BOOST_THROW_EXCEPTION(empty_image_error());
        
        const auto data_size = data.size() * sizeof(data[0]);
        if (data_size != size())
            BOOST_THROW_EXCEPTION(size_error()
                << errmsg("VideoOutputFrame buffer and provided buffer sizes do not match")
                << actual_size(data_size)
                << expected_size(size())
            );
        
        std::copy(data.begin(), data.end(), bytes());
        
        return *this;
    }
    
    
    VideoOutputFrame& VideoOutputFrame::load(const unsigned char *data_start, size_t data_size) {
        if (!data_start)
            BOOST_THROW_EXCEPTION(null_pointer_error() << errmsg("`data_start` cannot be null"));
            
        if (data_size != size())
            BOOST_THROW_EXCEPTION(size_error()
                << errmsg("VideoOutputFrame buffer and provided buffer sizes do not match")
                << actual_size(data_size)
                << expected_size(size())
            );
        
        std::copy(data_start, data_start + data_size, bytes());
        
        return *this;
    }
}
