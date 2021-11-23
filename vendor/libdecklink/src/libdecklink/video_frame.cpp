//
// Created by tibo on 30/01/18.
//

#include "libdecklink/errors.hpp"
#include "libdecklink/video_frame.hpp"

namespace DeckLink {
    
    VideoFrame::VideoFrame(IDeckLinkVideoFrame *raw_frame) : _raw_frame(raw_frame)
    {
        // Do Nothing
    }
    
    
    long VideoFrame::width() const {
        if (_width < 0) {
            _width = _raw_frame->GetWidth();
        }
        
        return _width;
    }
    
    
    long VideoFrame::height() const {
        if (_height < 0) {
            _height = _raw_frame->GetHeight();
        }
    
        return _height;
    }
    
    
    long VideoFrame::row_bytes() const {
        if (_row_bytes < 0) {
            _row_bytes = _raw_frame->GetRowBytes();
        }
    
        return _row_bytes;
    }
    
    
    size_t VideoFrame::size() const {
        return static_cast<size_t>( height() * row_bytes());
    }
    
    
    PixelFormat VideoFrame::pixel_format() const {
        if (_px_format == PixelFormat::Unknown) {
            _px_format = static_cast<PixelFormat>(_raw_frame->GetPixelFormat());
        }
        
        return _px_format;
    }
    
    
    unsigned char * VideoFrame::bytes() const {
        if (!_frame_data) {
            HRESULT result = _raw_frame->GetBytes(reinterpret_cast<void **>(&_frame_data));
            
            if (result != S_OK) {
                BOOST_THROW_EXCEPTION(decklink_driver_error()
                    << errmsg("Unknown Error")
                    << decklink_command("IDeckLinkVideoFrame::GetBytes")
                    << decklink_error_code(static_cast<HResult>(result))
                );
            }
        }
    
        return _frame_data;
    }
    
    
    std::string VideoFrame::timecode(TimecodeFormat format) const {
        IDeckLinkTimecode *timecode;
        
        HRESULT result = _raw_frame->GetTimecode(
            static_cast<BMDTimecodeFormat>(format), &timecode
        );
        
        if (result != S_OK) {
            std::string errdesc;
            switch(result) {
                case E_FAIL:
                    errdesc = "Unexpected Error";
                    break;
                case E_ACCESSDENIED:
                    errdesc = "An invalid or unsupported timecode format was requested.";
                    break;
                case S_FALSE:
                    errdesc = "The requested timecode format was not present or valid in the ancillary data.";
                    break;
                default:
                    errdesc = "Unknown Error";
            }
            
            BOOST_THROW_EXCEPTION(decklink_driver_error()
                << errmsg(errdesc)
                << decklink_command("IDeckLinkVideoInputFrame::GetTimecode")
                << decklink_error_code(static_cast<HResult>(result))
            );
        }
        
        assert(timecode);
        
        const char* timecode_str = nullptr;
        result = timecode->GetString(&timecode_str);
        
        if (result != S_OK) {
            BOOST_THROW_EXCEPTION(decklink_driver_error()
                << errmsg("Unknown Error")
                << decklink_command("IDeckLinkTimecode::GetString")
                << decklink_error_code(static_cast<HResult>(result))
            );
        }
        
        std::string result_str(timecode_str);
        free(const_cast<char*>(timecode_str));
        
        return result_str;
    }
    
    
    void *VideoFrame::start() {
        return bytes();
    }
    
    
    void *VideoFrame::end() {
        return static_cast<unsigned char*>(bytes()) + (row_bytes() * height() + 1);
    }
}
