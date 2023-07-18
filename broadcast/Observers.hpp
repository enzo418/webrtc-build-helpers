#pragma once

#include "webrtc/webrtc_config.h"
//

#include <future>

#include "webrtc/api/peer_connection_interface.h"

namespace broadrtc {
  class DummyConnectionObserver : public webrtc::PeerConnectionObserver {
    void OnSignalingChange(
        webrtc::PeerConnectionInterface::SignalingState) override {}

    void OnDataChannel(
        rtc::scoped_refptr<webrtc::DataChannelInterface>) override {}

    void OnIceGatheringChange(
        webrtc::PeerConnectionInterface::IceGatheringState) override {}

    void OnIceCandidate(const webrtc::IceCandidateInterface*) override {}
  };

  typedef std::promise<const webrtc::SessionDescriptionInterface*> PromiseSDP;
  typedef rtc::scoped_refptr<webrtc::PeerConnectionInterface> PC;

  /**
   * @brief listener class that sets the promise to the value generated.
   */
  class SetSessionDescriptionObserver
      : public webrtc::SetSessionDescriptionObserver {
   public:
    static rtc::scoped_refptr<SetSessionDescriptionObserver> Create(
        const PC& pc, PromiseSDP& promise) {
      return rtc::make_ref_counted<SetSessionDescriptionObserver>(pc, promise);
    }

    virtual void OnSuccess() {
      const webrtc::SessionDescriptionInterface* sdp =
          m_pc->local_description() ? m_pc->local_description()
                                    : m_pc->remote_description();
      m_promise.set_value(sdp);
    }

    virtual void OnFailure(webrtc::RTCError error) {
      RTC_LOG(LS_ERROR) << "Couldn't set the description: " << error.message();
      m_promise.set_value(nullptr);
    }

   protected:
    SetSessionDescriptionObserver(const PC& pc, PromiseSDP& promise)
        : m_pc(pc), m_promise(promise) {};

   private:
    PC m_pc;
    PromiseSDP& m_promise;
  };

  class CreateSessionDescriptionObserver
      : public webrtc::CreateSessionDescriptionObserver {
   public:
    static rtc::scoped_refptr<CreateSessionDescriptionObserver> Create(
        const PC& pc, PromiseSDP& promise) {
      return rtc::make_ref_counted<CreateSessionDescriptionObserver>(pc,
                                                                     promise);
    }
    virtual void OnSuccess(webrtc::SessionDescriptionInterface* desc) {
      m_pc->SetLocalDescription(
          SetSessionDescriptionObserver::Create(m_pc, m_promise).get(), desc);
    }
    virtual void OnFailure(webrtc::RTCError error) {
      RTC_LOG(LS_ERROR) << error.message();
      m_promise.set_value(NULL);
    }

   protected:
    CreateSessionDescriptionObserver(const PC& pc, PromiseSDP& promise)
        : m_pc(pc), m_promise(promise) {};

   private:
    PC m_pc;
    PromiseSDP& m_promise;
  };
}  // namespace broadrtc