//
// Created by tibo on 30/01/18.
//

#include <libdecklink/types.hpp>
#include "libdecklink/errors.hpp"
#include "libdecklink/video_input_frame.hpp"

namespace DeckLink {
    
    VideoInputFrame::VideoInputFrame(IDeckLinkVideoInputFrame *raw_input_frame)
        : VideoFrame(static_cast<IDeckLinkVideoFrame*>(raw_input_frame))
        , _raw_input_frame(raw_input_frame)
    {
        // Ensure that the frame will remain alive for as long as this class does
        _raw_input_frame->AddRef();
    }
    
    
    VideoInputFrame::~VideoInputFrame() {
        _raw_input_frame->Release();
    }
    
    
    double VideoInputFrame::stream_time() const {
        BMDTimeValue frame_time, frame_duration;
        BMDTimeScale time_scale = 24000;
        const HRESULT result = _raw_input_frame->GetStreamTime(
            &frame_time, &frame_duration, time_scale);
        
        if (result != S_OK) {
            BOOST_THROW_EXCEPTION(decklink_driver_error()
                << errmsg("Unknown Error")
                << decklink_command("IDeckLinkVideoInputFrame::GetStreamTime")
                << decklink_error_code(static_cast<HResult>(result))
            );
        }
        
        return static_cast<double>(frame_time) / static_cast<double>(time_scale);
    }
    
    
    double VideoInputFrame::hardware_reference_timestamp() const {
        BMDTimeValue frame_time, frame_duration;
        BMDTimeScale time_scale = 24000;
        const HRESULT result = _raw_input_frame->GetHardwareReferenceTimestamp(
            time_scale, &frame_time, &frame_duration);
    
        if (result != S_OK) {
            BOOST_THROW_EXCEPTION(decklink_driver_error()
                << errmsg("Unknown Error")
                << decklink_command("IDeckLinkVideoInputFrame::GetStreamTime")
                << decklink_error_code(static_cast<HResult>(result))
            );
        }
    
        return static_cast<double>(frame_time) / static_cast<double>(time_scale);
    }
    
    
    size_t row_bytes(PixelFormat pixel_format, int width) {
        switch(pixel_format) {
            case PixelFormat::YUV_8Bit:
                return static_cast<size_t>( width * 2 );
    
            case PixelFormat::YUV_10Bit:
                return static_cast<size_t>( (width + 47) / 48 * 128 );
    
            case PixelFormat::ARGB_8Bit:
            case PixelFormat::BGRA_8Bit:
                return static_cast<size_t>( width * 4 );
            
            case PixelFormat::RGB_12Bit:
            case PixelFormat::RGBLE_12Bit:
                return static_cast<size_t>( (width * 36) / 8 );
    
            case PixelFormat::RGB_10Bit:
            case PixelFormat::RGBXLE_10Bit:
            case PixelFormat::RGBX_10Bit:
                return static_cast<size_t>( (width + 63) / 64 * 256 );
    
            // For the remaining pixel formats the number of bytes per row can not be computed ahead of time
            default:
                return 0;
        }
    }
}
