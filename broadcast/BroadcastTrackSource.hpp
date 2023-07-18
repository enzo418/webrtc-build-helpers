#pragma once

#include "webrtc/webrtc_config.h"
//

#include <memory>
#include <string>
#include <vector>

#include "BroadcastSource.hpp"
#include "webrtc/api/peer_connection_interface.h"
#include "webrtc/pc/video_rtp_track_source.h"
#include "webrtc/rtc_base/ref_counted_object.h"

namespace broadrtc {
  class BroadcastTrackSource : public webrtc::VideoTrackSource,
                               public BroadcastStateChangeObserver {
   public:
    bool GetStats(Stats* stats) override {
      stats->input_height = m_source->GetHeight();
      stats->input_width = m_source->GetWidth();

      return true;
    }

    static rtc::scoped_refptr<BroadcastTrackSource> Create(
        std::unique_ptr<BroadcastSource> source) {
      return rtc::make_ref_counted<BroadcastTrackSource>(std::move(source));
    }

    bool Start() {
      if (m_source->IsRunning()) {
        return true;
      }

      return m_source->Start();
    }

    webrtc::MediaSourceInterface::SourceState state() const override {
      return m_state;
    }

    void OnBroadcastStarted() override {
      m_state = webrtc::MediaSourceInterface::SourceState::kLive;
    }

    void OnBroadcastStopped() override {
      m_state = webrtc::MediaSourceInterface::SourceState::kEnded;
    }

   protected:
    BroadcastTrackSource(std::unique_ptr<BroadcastSource> source)
        : VideoTrackSource(false), m_source(std::move(source)) {
      m_source->SetStateChangeObserver(this);
      assert(this->m_source->Start());
    }

    rtc::VideoSourceInterface<webrtc::VideoFrame>* source() override {
      return m_source.get() /*implicit cast*/;
    }

   private:
    std::unique_ptr<BroadcastSource> m_source;

    // use this instead of SetState
    webrtc::MediaSourceInterface::SourceState m_state;
  };
}  // namespace broadrtc