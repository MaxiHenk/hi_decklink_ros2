//
// Created by tibo on 16/01/18.
//

#include <iostream>

#include "libdecklink/types.hpp"
#include "libdecklink/display_mode.hpp"
#include "libdecklink/capture_callback.hpp"
#include "libdecklink/device_input_interface.hpp"

namespace DeckLink {
    
    //
    // Public Methods
    //
    
    CaptureCallback::CaptureCallback(CaptureCallback::frame_cb_t&& cb)
        : _on_frame_cb(cb)
    {
    
    }
    
    
    CaptureCallback::CaptureCallback(
        CaptureCallback::frame_cb_t&& frame_cb,
        CaptureCallback::video_format_changed_cb_t&& format_cb
    )
        : _on_frame_cb(frame_cb)
        , _on_video_format_changed_cb(format_cb)
    {
        // Do nothing
    }
    
    
    CaptureCallback::CaptureCallback(
        CaptureCallback::frame_cb_t&& frame_cb,
        CaptureCallback::video_format_changed_cb_t&& format_cb,
        CaptureCallback::error_cb_t&& error_cb
    )   : _on_frame_cb(frame_cb)
        , _on_video_format_changed_cb(format_cb)
        , _on_error_cb(error_cb)
    {
        // Do nothing
    }
    
    
    void CaptureCallback::set_on_new_frame_cb(const CaptureCallback::frame_cb_t&& cb) {
        _on_frame_cb = cb;
    }
    
    
    void CaptureCallback::set_on_input_format_changed_cb(
        const video_format_changed_cb_t&& cb
    ) {
        _on_video_format_changed_cb = cb;
    }
    
    
    void CaptureCallback::set_on_error_cb(const CaptureCallback::error_cb_t&& cb) {
        _on_error_cb = cb;
    }
    
    //
    // Callbacks
    //
    
    HRESULT CaptureCallback::VideoInputFormatChanged(
        BMDVideoInputFormatChangedEvents notification_events_raw,
        IDeckLinkDisplayMode *new_display_mode_raw,
        BMDDetectedVideoInputFormatFlags detected_signal_flags_raw
    ) {
        const auto notification_events =
            static_cast<InputFormatChangedEvent>(notification_events_raw);
        const DisplayMode new_display_mode(new_display_mode_raw);
        const auto detected_signal_flags =
            static_cast<DetectedVideoInputFormatFlags>(detected_signal_flags_raw);
        
        if (_on_video_format_changed_cb) {
            _on_video_format_changed_cb(
                notification_events, new_display_mode, detected_signal_flags
            );
        }

        const auto pixel_formats =
            _owner->get_supported_pixel_formats(new_display_mode.get_image_format());
        _owner->pause();
        _owner->enable(new_display_mode.get_image_format(), pixel_formats.front());
        _owner->flush();
        _owner->start();
        
        return S_OK;
    }

    
    HRESULT CaptureCallback::VideoInputFrameArrived(
        IDeckLinkVideoInputFrame *video_frame,
        IDeckLinkAudioInputPacket * /* unused */
    ) {
        // Cheat, if the frame is good we're going to reset the bad frame counter anyway
        ++_bad_frame_counter;
        
        if (!video_frame) {
            _on_error_cb(VideoInputError::NullFrame);
            return E_FAIL;
        }
        
        if(!video_frame->GetFlags() & bmdFrameHasNoInputSource) {
            _on_error_cb(VideoInputError::NoInputSource);
            return E_FAIL;
        }
        
        _bad_frame_counter = 0; // We know we have a good frame now
        
        // If we don't have a callback to call we don't actually have anything to do
        if (_on_frame_cb) {
            VideoInputFrame frame(video_frame);
            _on_frame_cb(frame);
        }
        
        return S_OK;
    }
    
    //
    // Getters and Setters
    //
    
    int CaptureCallback::get_bad_frame_threshold() const {
        return _bad_frame_threshold;
    }
    
    
    void CaptureCallback::set_bad_frame_threshold(int bad_frame_threshold) {
        _bad_frame_threshold = bad_frame_threshold;
    }
    
    
    TimecodeFormat CaptureCallback::get_timecode_format() const {
        return *_timecode_format;
    }
    
    
    void CaptureCallback::set_timecode_format(TimecodeFormat timecode_format) {
        _timecode_format = timecode_format;
    }
    
    //
    // Protected Methods
    //
 

}
