#pragma once

#include "webrtc/webrtc_config.h"
//

#include <iostream>
#include <regex>

#include "BroadcastTrackSource.hpp"
#include "Observers.hpp"
#include "Peer.hpp"

//
#include "webrtc/api/audio_codecs/audio_decoder_factory.h"
#include "webrtc/api/audio_codecs/audio_encoder_factory.h"
#include "webrtc/api/audio_codecs/builtin_audio_decoder_factory.h"
#include "webrtc/api/audio_codecs/builtin_audio_encoder_factory.h"
#include "webrtc/api/create_peerconnection_factory.h"
#include "webrtc/api/peer_connection_interface.h"
#include "webrtc/api/rtp_sender_interface.h"
#include "webrtc/api/scoped_refptr.h"
#include "webrtc/api/task_queue/default_task_queue_factory.h"
#include "webrtc/api/video_codecs/video_decoder_factory.h"
#include "webrtc/api/video_codecs/video_decoder_factory_template.h"
#include "webrtc/api/video_codecs/video_decoder_factory_template_dav1d_adapter.h"
#include "webrtc/api/video_codecs/video_decoder_factory_template_libvpx_vp8_adapter.h"
#include "webrtc/api/video_codecs/video_decoder_factory_template_libvpx_vp9_adapter.h"
#include "webrtc/api/video_codecs/video_decoder_factory_template_open_h264_adapter.h"
#include "webrtc/api/video_codecs/video_encoder_factory.h"
#include "webrtc/api/video_codecs/video_encoder_factory_template.h"
#include "webrtc/api/video_codecs/video_encoder_factory_template_libaom_av1_adapter.h"
#include "webrtc/api/video_codecs/video_encoder_factory_template_libvpx_vp8_adapter.h"
#include "webrtc/api/video_codecs/video_encoder_factory_template_libvpx_vp9_adapter.h"
#include "webrtc/api/video_codecs/video_encoder_factory_template_open_h264_adapter.h"
#include "webrtc/media/base/video_broadcaster.h"
#include "webrtc/modules/audio_device/include/fake_audio_device.h"
#include "webrtc/modules/video_capture/video_capture_factory.h"
#include "webrtc/pc/video_track_source.h"
#include "webrtc/rtc_base/checks.h"
#include "webrtc/rtc_base/ssl_adapter.h"
#include "webrtc/rtc_base/strings/json.h"
#include "webrtc/rtc_base/thread.h"

namespace broadrtc {
  /**
   * @brief This class is the main interface between the audience and us.
   * Usage:
   *  - Create a new instance of this class
   *  - Call ConnectToChannel with the desired source id to get the client id
   * and the offer, an Session Description json string compatible with
   *    https://www.w3.org/TR/webrtc/#rtcsessiondescription-class
   *  - Call SetAnswer with the id of the client and the answer, an Session
   *    Description json string compatible with
   *    https://www.w3.org/TR/webrtc/#rtcsessiondescription-class
   *  - Call EndConnection with the id of the client to end the connection.
   */
  class BroadcastWebRTC {
   public:
    enum LoggingSeverity {
      LS_VERBOSE,
      LS_INFO,
      LS_WARNING,
      LS_ERROR,
      LS_NONE,
    };

   public:
    BroadcastWebRTC() {
      m_signalingThread = rtc::Thread::Create();
      m_workerThread = rtc::Thread::Create();

      m_workerThread->SetName("worker", NULL);
      m_workerThread->Start();
      m_signalingThread->SetName("signaling", NULL);
      m_signalingThread->Start();

      rtc::scoped_refptr<webrtc::AudioDeviceModule> audio_module(
          new webrtc::FakeAudioDeviceModule());

      m_peerConnectionFactory = webrtc::CreatePeerConnectionFactory(
          nullptr /* network_thread */,
          m_workerThread.get() /* worker_thread */, m_signalingThread.get(),
          audio_module /* adm */, webrtc::CreateBuiltinAudioEncoderFactory(),
          webrtc::CreateBuiltinAudioDecoderFactory(),
          std::make_unique<webrtc::VideoEncoderFactoryTemplate<
              webrtc::LibvpxVp8EncoderTemplateAdapter,
              webrtc::OpenH264EncoderTemplateAdapter>>(),
          std::make_unique<webrtc::VideoDecoderFactoryTemplate<
              webrtc::LibvpxVp8DecoderTemplateAdapter,
              webrtc::OpenH264DecoderTemplateAdapter>>(),
          nullptr /* audio_mixer */, nullptr /* audio_processing */);

      if (!m_peerConnectionFactory.get()) {
        RTC_LOG(LS_ERROR) << "Failed to initialize PeerConnectionFactory";
        throw std::runtime_error("Failed to initialize PeerConnectionFactory");
      }
    }

    virtual ~BroadcastWebRTC() {
      m_peerConnections.clear();
      m_sources.clear();

      // don't stop the threads :)
    }

    static void InitWebRTC(LoggingSeverity logSeverity = LS_WARNING) {
      rtc::LogMessage::LogToDebug((rtc::LoggingSeverity)logSeverity);
      rtc::LogMessage::LogTimestamps();
      rtc::LogMessage::LogThreads();

      RTC_LOG(LS_INFO) << "RTC Log level: " << rtc::LogMessage::GetLogToDebug();

      // rtc::ThreadManager::Instance()->WrapCurrentThread();
      rtc::InitializeSSL();
    }

    static void DeinitWebRTC() {
      rtc::CleanupSSL();
      rtc::ThreadManager::Instance()->UnwrapCurrentThread();
    }

    /**
     * @brief Connects to a channel and returns the client id and the offer.
     * NOTE: As of now, I do not reuse peer connections.
     *
     * @param sourceId The id of the source to connect to.
     * @return std::pair<std::string, std::string> The client id and the offer.
     *
     * TODO: Make custom exceptions
     * @throws std::runtime_error if the channel could not be created.
     * @throws std::runtime_error if the peer connection could not be created.
     * @throws std::runtime_error if the offer could not be created.
     */
    std::pair<Peer::PeerID, std::string> ConnectToChannel(
        const std::string& sourceId,
        std::unique_ptr<BasicConnectionObserver> connectionObserver = nullptr) {
      RTC_LOG(LS_INFO) << "ConnectToChannel " << sourceId;

      rtc::scoped_refptr<BroadcastTrackSource> source;
      m_mutexSourcesMap.lock();
      auto found = m_sources.find(sourceId);
      if (found == m_sources.end()) {
        m_mutexSourcesMap.unlock();
        OnChannelNotFound(sourceId);
        return ConnectToChannel(sourceId, std::move(connectionObserver));
      }
      m_mutexSourcesMap.unlock();

      source = found->second;

      Peer& pc = CreatePeerConnection(std::move(connectionObserver));

      this->AddMediaTracks(sourceId, source, pc);

      return std::make_pair(pc.id, CreateOffer(pc));
    }

    /**
     * @brief Sets the answer for the client with the given id.
     *
     * @param clientId The id of the client.
     * @param answer The answer.
     * @throws std::runtime_error if the answer could not be set.
     */
    void SetAnswer(const Peer::PeerID& peerId, const std::string& answer) {
      RTC_LOG(LS_INFO) << "SetAnswer " << peerId;

      Peer* peer = nullptr;

      {
        LockGuard lock(m_mutexPeersMap);
        auto found = m_peerConnections.find(peerId);

        if (found == m_peerConnections.end()) {
          throw std::runtime_error("Client not found");
        }

        peer = &found->second;
      }

      Json::CharReaderBuilder readerBuilder;
      Json::Value answerSdpJson;
      std::string parseErrors;
      std::istringstream jsonStream(answer);

      if (!Json::parseFromStream(readerBuilder, jsonStream, &answerSdpJson,
                                 &parseErrors)) {
        RTC_LOG(LS_WARNING) << "Invalid input SDP: " << parseErrors;
        throw std::runtime_error("Invalid input SDP: " + parseErrors);
      }

      std::string type_str;
      if (!rtc::GetStringFromJsonObject(
              answerSdpJson, kSessionDescriptionTypeName, &type_str)) {
        RTC_LOG(LS_WARNING) << "Invalid input SDP: type";
        throw std::runtime_error("Invalid input SDP: type");
      }

      absl::optional<webrtc::SdpType> type =
          webrtc::SdpTypeFromString(type_str);

      if (!type || *type != webrtc::SdpType::kAnswer) {
        RTC_LOG(LS_WARNING) << "Invalid input SDP: type is not answer";
        throw std::runtime_error("Invalid input SDP: type is not answer");
      }

      std::string sdpAnswerStr;
      if (!rtc::GetStringFromJsonObject(
              answerSdpJson, kSessionDescriptionSdpName, &sdpAnswerStr)) {
        RTC_LOG(LS_WARNING) << "Invalid input SDP: sdp";
        throw std::runtime_error("Invalid input SDP: sdp");
      }

      webrtc::SdpParseError error;
      std::unique_ptr<webrtc::SessionDescriptionInterface> answer_sdp =
          webrtc::CreateSessionDescription(*type, sdpAnswerStr, &error);
      if (!answer_sdp) {
        RTC_LOG(LS_WARNING)
            << "Can't parse received session description message. Error: "
            << error.description;
        throw std::runtime_error(error.description);
      }

      PromiseSDP answer_sdp_promise;
      peer->connection->SetRemoteDescription(
          SetSessionDescriptionObserver::Create(peer->connection,
                                                answer_sdp_promise)
              .get(),
          answer_sdp.release());

      auto start = std::chrono::system_clock::now();

      auto sdp_future = answer_sdp_promise.get_future();

      while (sdp_future.wait_for(std::chrono::milliseconds(200)) !=
                 std::future_status::ready &&
             (std::chrono::system_clock::now() - start) <
                 std::chrono::seconds(/*timeout*/ 5)) {
      }

      if (!sdp_future.valid()) {
        RTC_LOG(LS_ERROR) << "Failed to set answer: Timeout";
        throw std::runtime_error("Failed to set answer");
      }
    }

    void EndConnection(const Peer::PeerID& peerId) {
      // RTC_LOG(LS_INFO) << "EndConnection " << peerId;
      std::cout << "EndConnection " << peerId << std::endl;

      LockGuard lock(m_mutexPeersMap);

      auto found = m_peerConnections.find(peerId);

      if (found == m_peerConnections.end()) {
        return;
      }

      Peer& peer = found->second;

      peer.connection->Close();

      // At this point the sources will check if there are no more clients
      // and stop itself or not.

      m_peerConnections.erase(found);
    }

   protected:
    /**
     * @brief Called when a channel was requested but not found.
     * In this method you should create the channel and add it to the sources
     * map. If it cannot be created, throw an exception.
     * @param sourceId
     */
    virtual void OnChannelNotFound(const std::string& sourceId) = 0;

   private:
    Peer& CreatePeerConnection(
        std::unique_ptr<BasicConnectionObserver> connectionObserver = nullptr) {
      if (connectionObserver == nullptr) {
        connectionObserver = std::make_unique<TerminatorConnectionObserver>();
      }

      webrtc::PeerConnectionInterface::RTCConfiguration config;
      webrtc::PeerConnectionDependencies pcDependencies(
          connectionObserver.get());

      auto errorOrPC = m_peerConnectionFactory->CreatePeerConnectionOrError(
          config, std::move(pcDependencies));

      if (!errorOrPC.ok()) {
        RTC_LOG(LS_ERROR) << "Failed to initialize PeerConnection: "
                          << errorOrPC.error().message();
        throw std::runtime_error(
            std::string("Failed to initialize PeerConnection: ") +
            errorOrPC.error().message());
      }

      LockGuard lock(m_mutexPeersMap);

      const size_t numPeerId = m_peerConnections.size();

      Peer::PeerID peerId = std::to_string(numPeerId);

      rtc::scoped_refptr<webrtc::PeerConnectionInterface> pc =
          std::move(errorOrPC.value());

      ((BasicConnectionObserver*)connectionObserver.get())
          ->SetPeerConnection(pc);

      m_peerConnections[peerId] =
          Peer {.id = peerId,
                .connection = std::move(pc),
                .connObserver = std::move(connectionObserver)};

      return m_peerConnections[peerId];
    }

    std::string CreateOffer(Peer& peer) {
      webrtc::PeerConnectionInterface::RTCOfferAnswerOptions rtcOptions;
      rtcOptions.offer_to_receive_video = 0;
      rtcOptions.offer_to_receive_audio = 0;
      PromiseSDP offer_sdp_promise;
      peer.connection->CreateOffer(CreateSessionDescriptionObserver::Create(
                                       peer.connection, offer_sdp_promise)
                                       .get(),
                                   rtcOptions);

      auto start = std::chrono::system_clock::now();

      auto sdp_future = offer_sdp_promise.get_future();
      while (sdp_future.wait_for(std::chrono::milliseconds(200)) !=
                 std::future_status::ready &&
             (std::chrono::system_clock::now() - start) <
                 std::chrono::seconds(/*timeout*/ 5)) {
      }

      if (!sdp_future.valid()) {
        RTC_LOG(LS_ERROR) << "Failed to create offer: Timeout";
        throw std::runtime_error("Failed to create offer");
      }

      auto offer = sdp_future.get();

      // build the kind of offer that the browser expects
      std::string offerSdpStr;
      offer->ToString(&offerSdpStr);
      Json::Value offerJson;
      offerJson[kSessionDescriptionTypeName] = offer->type();
      offerJson[kSessionDescriptionSdpName] = offerSdpStr;

      static Json::StreamWriterBuilder builder;
      return Json::writeString(builder, offerJson);
    }

    void AddMediaTracks(const std::string& sourceId,
                        rtc::scoped_refptr<BroadcastTrackSource> channel,
                        Peer& peer) {
      if (channel->state() !=
          webrtc::MediaSourceInterface::SourceState::kLive) {
        RTC_LOG(LS_INFO) << "Starting channel " << sourceId;

        if (!channel->Start()) {
          RTC_LOG(LS_ERROR) << "Failed to start channel " << sourceId;
          throw std::runtime_error("Failed to start channel " + sourceId);
        }
      }

      static std::regex onlyAlphaNumeric("[^A-Za-z0-9]");
      const std::string videoLabel =
          "vt_" + std::regex_replace(sourceId, onlyAlphaNumeric, "");

      rtc::scoped_refptr<webrtc::VideoTrackInterface> videoTrack(
          m_peerConnectionFactory->CreateVideoTrack(channel, videoLabel));

      auto resultOrError = peer.connection->AddTrack(videoTrack, {videoLabel});

      if (!resultOrError.ok()) {
        RTC_LOG(LS_ERROR) << "Failed to add video track to PeerConnection: "
                          << resultOrError.error().message();
        throw std::runtime_error(
            std::string("Failed to add video track to PeerConnection: ") +
            resultOrError.error().message());
      }
    }

   protected:
    // Last to destroy should be the threads
    std::unique_ptr<rtc::Thread> m_signalingThread;
    std::unique_ptr<rtc::Thread> m_workerThread;

    typedef std::lock_guard<std::mutex> LockGuard;

    std::mutex m_mutexPeersMap;
    std::unordered_map<std::string, Peer> m_peerConnections;

    std::mutex m_mutexSourcesMap;
    std::unordered_map<std::string, rtc::scoped_refptr<BroadcastTrackSource>>
        m_sources;

    rtc::scoped_refptr<webrtc::PeerConnectionFactoryInterface>
        m_peerConnectionFactory;

    static constexpr char kSessionDescriptionTypeName[] = "type";
    static constexpr char kSessionDescriptionSdpName[] = "sdp";
  };
}  // namespace broadrtc