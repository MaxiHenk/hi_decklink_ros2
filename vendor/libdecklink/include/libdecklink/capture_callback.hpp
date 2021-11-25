//
// Created by tibo on 16/01/18.
//

#pragma once

#include <functional>
#include <boost/optional.hpp>

#include "decklink_sdk/DeckLinkAPI.h"
#include "libdecklink/types.hpp"
#include "libdecklink/errors.hpp"
#include "libdecklink/video_input_frame.hpp"
#include "libdecklink/dummy_reference_counted.hpp"

namespace DeckLink {
    
    class DeviceInputInterface;
    
    class CaptureCallback : virtual public IDeckLinkInputCallback {
        
        friend class DeviceInputInterface;
    
    public: /* types */
        
        /**
         * Type of the new frame callback function.
         *
         * This is the type of the callback function that is called when a new frame is received.
         */
        using frame_cb_t = std::function<void(const VideoInputFrame&)>;
    
        /**
         * Type of the video format changed callback function.
         *
         * This is the type of the callback function that is called when a video format change is
         * detected.
         */
        using video_format_changed_cb_t = std::function<void(
            InputFormatChangedEvent, const DisplayMode&, DetectedVideoInputFormatFlags)>;
    
        /**
         * Type of the error callback function.
         *
         * This is the type of the callback function that is called when an error occurs giving the
         * user a chance to handle it.
         */
        using error_cb_t = std::function<void(VideoInputError)>;
    
    public:
        CaptureCallback() = default;
        CaptureCallback(CaptureCallback&) = default;
        
        CaptureCallback& operator= (CaptureCallback&)= default;
        
        /**
         * Constructor
         * @param cb The callback function to call when a new frame is received
         */
        explicit CaptureCallback(frame_cb_t&& cb);
    
        /**
         * Constructor
         * @param frame_cb The callback function to call when a new frame is received
         * @param format_cb The callback function to call when a video format change is detected
         */
        explicit CaptureCallback(frame_cb_t&& frame_cb, video_format_changed_cb_t&& format_cb);
        
        /**
         * Constructor
         * @param frame_cb The callback function to call when a new frame is received
         * @param format_cb The callback function to call when a video format change is detected
         * @param error_cb The callback function to call when an error is produced
         */
        explicit CaptureCallback(
            frame_cb_t&& frame_cb,
            video_format_changed_cb_t&& format_cb,
            error_cb_t&& error_cb
        );
        
        void set_on_new_frame_cb(const frame_cb_t&& cb);
        
        void set_on_input_format_changed_cb(const video_format_changed_cb_t&& cb);
        
        void set_on_error_cb(const error_cb_t&& cb);
    
        //
        // Getters & Setters
        //
        
        /**
         * The bad frame threshold is number of consecutive empty frames that should be considered
         * an error. Dropped frames will happen every now and then and might not always be
         * considered an error but more than 15 consecutive might definitely be.
         *
         * This gives you some freedom to customize this behaviour.
         */
        void set_bad_frame_threshold(int bad_frame_threshold);
        int get_bad_frame_threshold() const;
    
        void set_timecode_format(TimecodeFormat timecode_format);
        TimecodeFormat get_timecode_format() const;
        
        DUMMY_REF_COUNT_METHODS

    public: /* Callbacks */
        
        /**
         * This method is called by the hardware when a video input format change is detected.
         *
         * The most common case for this is when the stream has been enabled with input format
         * detection. In this case we set the stream format to a default, probably incorrect, value
         * and let the hardware give us the right stream options when it receives the first frame.
         */
        HRESULT VideoInputFormatChanged(
            BMDVideoInputFormatChangedEvents notification_events_raw,
            IDeckLinkDisplayMode *new_display_mode,
            BMDDetectedVideoInputFormatFlags
            detected_signal_flags_raw
        ) override;
        
        
        /**
         * This is the function called by the DeckLink driver when a new frame is received.
         *
         * @param video_frame
         *  The input video frame. It is only valid for the duration of the callback. If we need to
         *  keep it around we have to call IDeckLinkVideoFrame::AddRef on it and subsequently
         *  IDeckLinkVideoFrame::Release to free it.
         *
         *  The pointer may be null in some situations. This is usually indicative of a problem
         *  with the input (i.e. is it even turned on correctly ?)
         *
         * @return The return value is ignored by the caller.
         */
        HRESULT VideoInputFrameArrived(
            IDeckLinkVideoInputFrame *video_frame,
            IDeckLinkAudioInputPacket * /* unused */
        ) override;
        
    private: /* Private Members */
        
        // State Variables
        int _bad_frame_counter = 0;
        int _bad_frame_threshold = 24;
        
        // Other stuff
        
        DeviceInputInterface* _owner = nullptr;
        
        boost::optional<TimecodeFormat> _timecode_format;
    
        frame_cb_t _on_frame_cb;
        video_format_changed_cb_t _on_video_format_changed_cb;
        error_cb_t _on_error_cb;
    };
    
}
