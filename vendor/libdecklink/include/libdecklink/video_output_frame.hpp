//
// Created by tibo and paola on 21/02/18.
//

#pragma once

#include "libdecklink/video_frame.hpp"

namespace DeckLink {
    
    class VideoOutputFrame : public VideoFrame {
    public:
        explicit VideoOutputFrame(IDeckLinkMutableVideoFrame* raw_output_frame);

        ~VideoOutputFrame() override;
    
        VideoOutputFrame& load(const std::vector<unsigned char>& data);
        
        VideoOutputFrame& load(const unsigned char *data_start, size_t data_size);
        

    protected:
    
    private:
        IDeckLinkMutableVideoFrame* _raw_output_frame;
    };
    
}
