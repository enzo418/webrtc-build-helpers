#pragma once

#include "webrtc/webrtc_config.h"
//

#include <future>
#include <iostream>
#include <stdexcept>

#include "webrtc/api/peer_connection_interface.h"

namespace broadrtc {
  /**
   * @brief The base connection observer class that does not react to events,
   * but has the peer connection that is being observed.
   *
   */
  class BasicConnectionObserver : public webrtc::PeerConnectionObserver {
   public:
    void SetPeerConnection(
        rtc::scoped_refptr<webrtc::PeerConnectionInterface> peerConnection) {
      m_peerConnection = peerConnection;
    }

   public:
    virtual void OnSignalingChange(
        webrtc::PeerConnectionInterface::SignalingState) override {}

    virtual void OnDataChannel(
        rtc::scoped_refptr<webrtc::DataChannelInterface>) override {}

    virtual void OnIceGatheringChange(
        webrtc::PeerConnectionInterface::IceGatheringState) override {}

    virtual void OnIceCandidate(const webrtc::IceCandidateInterface*) override {
    }

    virtual void OnConnectionChange(
        webrtc::PeerConnectionInterface::PeerConnectionState) override {}

   protected:
    rtc::scoped_refptr<webrtc::PeerConnectionInterface> m_peerConnection;
  };

  /**
   * @brief This observer will close the connection if the connection state
   * changes to failed, which usually happens after a timeout on disconnect.
   */
  class TerminatorConnectionObserver : public BasicConnectionObserver {
   public:
    void OnConnectionChange(webrtc::PeerConnectionInterface::PeerConnectionState
                                new_state) override {
      // The previous state doesn't apply (it's not closed) and any
      // RTCIceTransports are in the "failed" state.
      if (new_state ==
          webrtc::PeerConnectionInterface::PeerConnectionState::kFailed) {
        if (m_peerConnection.get() != nullptr) {
          m_peerConnection->Close();
        } else {
          throw std::runtime_error(
              "The peer connection is null. Make sure to call "
              "SetPeerConnection or use your own "
              "webrtc::PeerConnectionObserver.");
        }
      }
    }
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