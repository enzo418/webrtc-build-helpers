#pragma once

#include "webrtc/webrtc_config.h"
//

#include <memory>
#include <set>
#include <string>

#include "webrtc/api/video/video_frame.h"
#include "webrtc/api/video/video_source_interface.h"
#include "webrtc/media/base/video_broadcaster.h"
#include "webrtc/rtc_base/logging.h"

//
#include "webrtc/api/video/i420_buffer.h"

// libyuv helpers
#include "webrtc/libyuv/convert.h"
#include "webrtc/libyuv/video_common.h"

namespace broadrtc {
  class BroadcastStateChangeObserver {
   public:
    virtual void OnBroadcastStarted() = 0;
    virtual void OnBroadcastStopped() = 0;
  };

  /**
   * @brief Abstract class that represents a source of video frames, to which
   * sinks (clients) can be attached.
   */
  class BroadcastSource : public rtc::VideoSourceInterface<webrtc::VideoFrame> {
   public:
    virtual ~BroadcastSource() {}

    void AddOrUpdateSink(rtc::VideoSinkInterface<webrtc::VideoFrame>* sink,
                         const rtc::VideoSinkWants& wants) override {
      m_sinksAdded.insert(reinterpret_cast<intptr_t>(sink));
      m_broadcaster.AddOrUpdateSink(sink, wants);
    }

    void RemoveSink(
        rtc::VideoSinkInterface<webrtc::VideoFrame>* sink) override {
      m_broadcaster.RemoveSink(sink);

      m_sinksAdded.erase(reinterpret_cast<intptr_t>(sink));

      if (m_sinksAdded.empty()) {
        OnNoAudience();
      }
    }

    void OnFrame(const webrtc::VideoFrame& frame) {
      m_broadcaster.OnFrame(frame);
    }

    /**
     * @brief Start the source, so it eventually calls OnFrame.
     *
     * @remark Most likely you will want to override this function, start you
     * capture loop or register some callback, and call the base class Start()
     * function.
     *
     * @remark This function is called by BroadcastTrackSource when it is added
     * to a PeerConnection.
     *
     * @return true
     * @return false
     */
    virtual bool Start() {
      m_stateChangeObserver->OnBroadcastStarted();
      return true;
    }

    /**
     * @brief Stop the source.
     *
     * @remark This function is not called by any broadcast class. So, if your
     * source doesn't need to be stopped, you may ignore it. Else, it will
     * likely to be called from OnNoAudience().
     *
     * @return true
     * @return false
     */
    virtual bool Stop() {
      m_stateChangeObserver->OnBroadcastStopped();
      return true;
    }

    virtual bool IsRunning() = 0;

    virtual int GetWidth() = 0;
    virtual int GetHeight() = 0;

   protected:
    /**
     * @brief Called when there are no more clients connected to this source.
     * You can choose to stop it or not.
     */
    virtual void OnNoAudience() {
      RTC_LOG(LS_INFO) << "A BroadcastSource no longer has an audience";
    }

   private:
    void SetStateChangeObserver(BroadcastStateChangeObserver* observer) {
      assert(observer != nullptr);
      m_stateChangeObserver = observer;
    }

    friend class BroadcastTrackSource;  // access to SetStateChangeObserver

   private:
    rtc::VideoBroadcaster m_broadcaster;

    // used to count the number of clients
    std::set<intptr_t> m_sinksAdded;

    BroadcastStateChangeObserver* m_stateChangeObserver;
  };
}  // namespace broadrtc