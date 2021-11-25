//
// Created by tibo on 30/01/18.
//

#pragma once

#include "libdecklink/types.hpp"
#include "libdecklink/video_frame.hpp"

namespace DeckLink {
    
    class VideoInputFrame : public VideoFrame {
    public:
        explicit VideoInputFrame(IDeckLinkVideoInputFrame* raw_input_frame);
        
        ~VideoInputFrame() override;
    
        double stream_time() const;
        double hardware_reference_timestamp() const;
        
    protected:
    
    private:
        IDeckLinkVideoInputFrame* _raw_input_frame;
    };
    
    /**
     * Compute the length of bytes of a single image row.
     *
     * This calculation is trivial for simple pixel formats but for formats where the size of a pixel in bytes is non-integer we have to consider alignment. The values here are taken from the DeckLink BlackMagic SDK docs shipped in this repo in `/doc/`.
     *
     * @param pixel_format The pixel format of the frame
     * @param width The width of the frame in pixels
     * @return The number of bytes per frame row
     */
    size_t row_bytes(PixelFormat pixel_format, int width);
    
}
