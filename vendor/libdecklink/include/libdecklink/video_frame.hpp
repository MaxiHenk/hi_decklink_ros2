//
// Created by tibo on 30/01/18.
//

#pragma once

#include "libdecklink/types.hpp"

namespace DeckLink {
    
    /**
     * Wrapper for IDeckLinkVideoFrame*
     *
     * @note
     *  Timecodes are not completely supported for the simple reason that I have no hardware with
     *  timecodes with which to test. The VideoFrame::timecode() function will return a string
     *  representation if the timecode could be found.
     */
    class VideoFrame {
    public:
        explicit VideoFrame(IDeckLinkVideoFrame* raw_frame);
        
        virtual ~VideoFrame() = default;
        
        long width() const;
        long height() const;
        
        /// Get the size of a row accounting for extra padding at the end of rows
        long row_bytes() const;
        size_t size() const;
        
        PixelFormat pixel_format() const;
        // FrameFlags frame_flags() const;
        
        unsigned char * bytes() const ;
        
        std::string timecode(TimecodeFormat format) const;
        
        /// Return a pointer to the start of the enclosed data
        void* start();
        
        /// Return a past-the-end pointer to the enclosed data
        void* end();
        
        IDeckLinkVideoFrame* get_raw_ptr() const { return _raw_frame; }
        
    protected:
    
    private:
        
        IDeckLinkVideoFrame* _raw_frame = nullptr;
        
        // Cached values
        
        mutable long _width = -1;
        mutable long _height = -1;
        mutable long _row_bytes = -1;
        
        mutable unsigned char* _frame_data = nullptr;
        
        mutable PixelFormat _px_format = PixelFormat::Unknown;
        // FrameFlags _frame_flags;
        
        
    };
    
}
