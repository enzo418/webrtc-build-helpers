// ENSURE webrtc_config IS INCLUDED FIRST
#include "webrtc/webrtc_config.h"
//

#include <atomic>
#include <future>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <thread>

#include "jsoncpp/json/json.h"
#include "webrtc/api/audio_codecs/audio_decoder_factory.h"
#include "webrtc/api/audio_codecs/audio_encoder_factory.h"
#include "webrtc/api/audio_codecs/builtin_audio_decoder_factory.h"
#include "webrtc/api/audio_codecs/builtin_audio_encoder_factory.h"
#include "webrtc/api/create_peerconnection_factory.h"
#include "webrtc/api/peer_connection_interface.h"
#include "webrtc/api/rtp_sender_interface.h"
#include "webrtc/api/scoped_refptr.h"
#include "webrtc/api/task_queue/default_task_queue_factory.h"
#include "webrtc/api/video/i420_buffer.h"
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
#include "webrtc/libyuv/convert.h"
#include "webrtc/libyuv/video_common.h"
#include "webrtc/media/base/video_broadcaster.h"
#include "webrtc/modules/audio_device/include/fake_audio_device.h"
#include "webrtc/modules/video_capture/video_capture_factory.h"
#include "webrtc/pc/video_track_source.h"
#include "webrtc/rtc_base/checks.h"
#include "webrtc/rtc_base/ssl_adapter.h"
#include "webrtc/rtc_base/strings/json.h"
#include "webrtc/rtc_base/thread.h"

int main() {
  // 1. Include at the top webrtc/webrtc_config.h

  // This file showcases how to use this library as a broadcast (but with 1
  // peer only :p) server through webrtc, using opencv as a video source.

  // NOTE 1: webrtc api/peer_connection_interface.h explains the sequence
  // NOTE 2: Debugging is recommended to understand all the observer dances.

  // Starting from here

  rtc::LogMessage::LogToDebug(rtc::LoggingSeverity::LS_WARNING);
  rtc::LogMessage::LogTimestamps();
  rtc::LogMessage::LogThreads();
  RTC_LOG(LS_INFO) << "log level: " << rtc::LogMessage::GetLogToDebug();

  rtc::ThreadManager::Instance()->WrapCurrentThread();
  rtc::InitializeSSL();

  auto signalingThread = rtc::Thread::Create();
  auto workerThread = rtc::Thread::Create();

  workerThread->SetName("worker", NULL);
  workerThread->Start();
  signalingThread->SetName("signaling", NULL);
  signalingThread->Start();

  // auto task_queue_factory = webrtc::CreateDefaultTaskQueueFactory();

  /* ------------------------------------------------------ */
  /*           CREATE THE PEER CONNECTION FACTORY           */
  /* ------------------------------------------------------ */
  RTC_LOG(LS_INFO) << "Create PeerConnectionFactory";

  /**
   * A note about the rtc types:
   * ---------------------------
   * The type "rtc::scoped_refptr<T>" is the equivalent to a
   * "std::shared_ptr<T>".
   * It's a reference counted pointer, so it's safe to pass it around.
   *
   * Also, rtc::make_ref_counted<T>(args) is the equivalent to
   * std::make_shared<T>(args).
   */

  rtc::scoped_refptr<webrtc::AudioDeviceModule> audio_module(
      new webrtc::FakeAudioDeviceModule());

  auto peer_connection_factory_ = webrtc::CreatePeerConnectionFactory(
      nullptr /* network_thread */, workerThread.get() /* worker_thread */,
      signalingThread.get(),

#ifdef WEBRTC_INCLUDE_DEFAULT_AUDIO
      nullptr /* default adm */,
#else
      audio_module /* fake adm */,
#endif

      webrtc::CreateBuiltinAudioEncoderFactory(),
      webrtc::CreateBuiltinAudioDecoderFactory(),
      std::make_unique<webrtc::VideoEncoderFactoryTemplate<
          webrtc::LibvpxVp8EncoderTemplateAdapter,
          webrtc::OpenH264EncoderTemplateAdapter>>(),
      std::make_unique<webrtc::VideoDecoderFactoryTemplate<
          webrtc::LibvpxVp8DecoderTemplateAdapter,
          webrtc::OpenH264DecoderTemplateAdapter>>(),
      nullptr /* audio_mixer */, nullptr /* audio_processing */);

  if (!peer_connection_factory_.get()) {
    RTC_LOG(LS_ERROR) << "Failed to initialize PeerConnectionFactory";
    return -1;
  }

  /* ------------------------------------------------------ */
  /*                CREATE A PEER CONNECTION                */
  /* ------------------------------------------------------ */
  /**
   * A peer connection is that, a connection between two peers.
   * A connection is established by exchanging SDPs (Session Description
   * Protocol) between the peers. In this case us (local) first send the offer
   * so the client (remote) knows what media we have available, which codecs we
   * support, etc. Then the client sends an answer with which media it wants
   * to receive.
   */

  RTC_LOG(LS_INFO) << "Create PeerConnection";

  rtc::scoped_refptr<webrtc::PeerConnectionInterface> peer_connection;

  webrtc::PeerConnectionInterface::RTCConfiguration config;
  config.sdp_semantics = webrtc::SdpSemantics::kUnifiedPlan;

  // Setup ICE servers (STUN and TURN)
  // e.g.
  // webrtc::PeerConnectionInterface::IceServer server;
  // server.uri = "stun:stun.l.google.com:19302";
  // config.servers.push_back(server);

  /**
   * Provides an impl to pure virtual methods of PeerConnectionObserver.
   */
  class MyPeerConnectionObserver : public webrtc::PeerConnectionObserver {
    void OnSignalingChange(
        webrtc::PeerConnectionInterface::SignalingState new_state) override {
      RTC_LOG(LS_INFO) << "OnSignalingChange: " << new_state;
    }

    void OnDataChannel(
        rtc::scoped_refptr<webrtc::DataChannelInterface>) override {
      RTC_LOG(LS_INFO) << "OnDataChannel";
    }

    void OnIceGatheringChange(
        webrtc::PeerConnectionInterface::IceGatheringState new_state) override {
      RTC_LOG(LS_INFO) << "OnIceGatheringChange: " << new_state;
    }

    void OnIceCandidate(
        const webrtc::IceCandidateInterface* candidate) override {
      RTC_LOG(LS_INFO) << "OnIceCandidate: "
                       << candidate->candidate().ToString();
    }
  };

  MyPeerConnectionObserver* observer = new MyPeerConnectionObserver();

  webrtc::PeerConnectionDependencies pc_dependencies(observer);
  // The preferred way to create a new peer connection.
  auto error_or_peer_connection =
      peer_connection_factory_->CreatePeerConnectionOrError(
          config, std::move(pc_dependencies));

  if (!error_or_peer_connection.ok()) {
    RTC_LOG(LS_ERROR) << "Failed to initialize PeerConnection: "
                      << error_or_peer_connection.error().message();
    return -1;
  }

  peer_connection = std::move(error_or_peer_connection.value());

  assert(peer_connection.get() != nullptr);

  /* ------------------------------------------------------ */
  /*                       Add Tracks                       */
  /* ------------------------------------------------------ */

  /**
   * @brief This class will generate the frames and then send it using the
   * broadcaster member. It's a webrtc source and track at the same time,
   * but you can separate it if you have multiple sources.
   */
  class OpenCvSource : public webrtc::VideoTrackSource {
   public:
    /*
     * NOTE 1:
     * OpenCvSource is a abstract class because it has an interface in
     * it's inheritance tree, rtc::RefCountInterface. That interface has two
     * pure virtual methods: AddRef and Release, which are implemented by
     * RefCountedObject by using rtc::make_ref_counted<T>(args).
     * This is the reason why static Create methods are preferred to
     * constructors in this webrtc library.
     */
    static rtc::scoped_refptr<OpenCvSource> Create() {
      return rtc::make_ref_counted<OpenCvSource>();
    }

   private:
    rtc::VideoSourceInterface<webrtc::VideoFrame>* source() override {
      /*
       * NOTE 2:
       * VideoTrackSource overrides AddOrUpdateSink and RemoveSink from
       the
       * video source interface, and forwards the call to
       *    source()->AddOrUpdateSink(sink, wants);
       *    source()->RemoveSink(sink);
       * That's the reason why we need to override source() and return
       * m_broadcaster.
       * Otherwise, if we wouldn't inherit from VideoTrackSource, we would
       need void
       AddOrUpdateSink(rtc::VideoSinkInterface<webrtc::VideoFrame> *sink,
                            const rtc::VideoSinkWants &wants) override {
          m_broadcaster.AddOrUpdateSink(sink, wants);
        }

        void
        RemoveSink(rtc::VideoSinkInterface<webrtc::VideoFrame> *sink)
       override
        {
          m_broadcaster.RemoveSink(sink);
        }
       */

      return &m_broadcaster;
    }

   protected:  // protected because RefCountedObject<T> : T
    OpenCvSource() : VideoTrackSource(/*remote=*/false), m_isRunning(false) {
      RTC_LOG(LS_INFO) << "OpenCvSource:OpenCvSource";
      assert(this->Start());
    }

   public:
    ~OpenCvSource() {
      RTC_LOG(LS_INFO) << "OpenCvSource:~OpenCvSource";
      this->Stop();
    }

    bool GetStats(Stats* stats) override {
      /**
       * This function comes from VideoTrackSourceInterface,
       * @see VideoTrackSourceInterface::GetStats()
       */
      stats->input_height = this->GetHeight();
      stats->input_width = this->GetWidth();

      return true;
    }

    void CaptureThread() {
      int frameNumber {0};
      cv::Mat frame(360, 640, CV_8UC3, cv::Scalar(0, 0, 0));
      while (m_isRunning) {
        frame.setTo(cv::Scalar(0, 0, 0));

        RTC_LOG(LS_INFO) << "OpenCvSource:ReadFrame";

        cv::putText(frame, std::to_string(frameNumber++), cv::Point(320, 180),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255),
                    2);

        cv::imshow("frame", frame);
        cv::waitKey(1);

        // build video frame
        // Ensure the input image has the desired format (e.g., BGR or
        // RGB)
        cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);

        // Define variables for the converted I420 image
        int width = frame.cols;
        int height = frame.rows;
        rtc::scoped_refptr<webrtc::I420Buffer> I420buffer =
            webrtc::I420Buffer::Create(width, height);

        // Convert the input image to I420 format using libyuv
        int res = libyuv::ConvertToI420(
            frame.data, frame.cols * 3,  // Input image data and stride
            I420buffer->MutableDataY(), I420buffer->StrideY(),
            I420buffer->MutableDataU(), I420buffer->StrideU(),
            I420buffer->MutableDataV(), I420buffer->StrideV(), 0,
            0,                   // No cropping
            width, height,       // Input image width and height
            width, height,       // Output I420 image width and height
            libyuv::kRotate0,    // No rotation
            libyuv::FOURCC_RGB3  // Input image format (RGB24)
        );

        // Wrap the converted image in a VideoFrame
        if (res >= 0) {
          webrtc::VideoFrame videoFrame =
              webrtc::VideoFrame::Builder()
                  .set_video_frame_buffer(I420buffer)
                  .set_rotation(webrtc::kVideoRotation_0)
                  .set_timestamp_us(rtc::TimeMicros())
                  //.set_id(ts)
                  .build();
          m_broadcaster.OnFrame(videoFrame);
        } else {
          RTC_LOG(LS_ERROR)
              << "OpenCvSource:OnCaptureResult conversion error:" << res;
        }

        // wait for 1/30th of a second
        std::this_thread::sleep_for(std::chrono::milliseconds(33));
      }
    }

    bool Start() {
      // VideoTrackSource::SetState should be called on the signaling
      // thread.
      VideoTrackSource::SetState(webrtc::MediaSourceInterface::kLive);

      m_isRunning = true;
      m_captureThread = std::thread(&OpenCvSource::CaptureThread, this);
      return true;
    }

    void Stop() {
      m_isRunning = false;
      if (m_captureThread.joinable()) {
        m_captureThread.join();
      }

      // VideoTrackSource::SetState should be called on the signaling
      // thread.
      VideoTrackSource::SetState(webrtc::MediaSourceInterface::kEnded);
    }

    int GetWidth() { return 640; }
    int GetHeight() { return 360; }

   protected:
    rtc::VideoBroadcaster m_broadcaster;

   private:
    std::thread m_captureThread;
    std::atomic_bool m_isRunning;
  };  // class OpenCvSource

  // Video device - See note 1
  auto video_device = OpenCvSource::Create();

  if (!video_device) {
    RTC_LOG(LS_ERROR) << "OpenVideoCaptureDevice failed";
    return -1;
  }

  // 3. Create local MediaStreamTracks using the PeerConnectionFactory ...
  const char video_label[] = "video_label";
  rtc::scoped_refptr<webrtc::VideoTrackInterface> video_track_(
      peer_connection_factory_->CreateVideoTrack(video_device, video_label));

  // ... and add them to PeerConnection by calling AddTrack (or legacy method,
  // AddStream).
  const std::string kStreamId = "stream_id";
  auto result_or_error = peer_connection->AddTrack(video_track_, {kStreamId});
  if (!result_or_error.ok()) {
    RTC_LOG(LS_ERROR) << "Failed to add video track to PeerConnection: "
                      << result_or_error.error().message();
  }

  /* ------------------------------------------------------ */
  /*                     Create an offer                    */
  /* ------------------------------------------------------ */
  typedef std::promise<const webrtc::SessionDescriptionInterface*> PromiseSDP;
  typedef rtc::scoped_refptr<webrtc::PeerConnectionInterface> PC;

  // 4. Create an offer, call SetLocalDescription with it, serialize it, and
  // send it to the remote peer

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
  };  // class SetSessionDescriptionObserver

  // NOTE: chromium webrtc uses observers to handle callbacks, this is because
  // you are not supposed to stall the signaling thread.
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
  };  // class CreateSessionDescriptionObserver

  webrtc::PeerConnectionInterface::RTCOfferAnswerOptions rtc_options;
  rtc_options.offer_to_receive_video = 0;  // broadcast
  rtc_options.offer_to_receive_audio = 0;  // broadcast
  PromiseSDP offer_sdp_promise;
  peer_connection->CreateOffer(CreateSessionDescriptionObserver::Create(
                                   peer_connection, offer_sdp_promise)
                                   .get(),
                               rtc_options);

  auto sdp_future = offer_sdp_promise.get_future();
  while (sdp_future.wait_for(std::chrono::milliseconds(200)) !=
         std::future_status::ready) {
    std::cout << ".\n";
  }

  // write offer to std out
  auto offer = sdp_future.get();

  std::string offer_sdp_str;
  offer->ToString(&offer_sdp_str);

  RTC_LOG(LS_INFO) << "OFFER READY! Paste it in the browser";
  Json::Value offer_json;

  const char kSessionDescriptionTypeName[] = "type";
  const char kSessionDescriptionSdpName[] = "sdp";

  offer_json[kSessionDescriptionTypeName] = offer->type();
  offer_json[kSessionDescriptionSdpName] = offer_sdp_str;
  std::cout << offer_json << std::endl;

  std::cout << "\n\nEnter the ANSWER SDP generated: \n\n";
  std::string answer_sdp_str;
  std::getline(std::cin, answer_sdp_str);

  // Parse the JSON object
  Json::CharReaderBuilder readerBuilder;
  Json::Value answer_sdp_json;
  std::string parseErrors;
  std::istringstream jsonStream(answer_sdp_str);

  if (!Json::parseFromStream(readerBuilder, jsonStream, &answer_sdp_json,
                             &parseErrors)) {
    std::cout << "JSON parsing failed. Error: " << parseErrors << std::endl;
  }

  std::string type_str;
  if (!rtc::GetStringFromJsonObject(answer_sdp_json,
                                    kSessionDescriptionTypeName, &type_str)) {
    RTC_LOG(LS_WARNING) << "Invalid input SDP: type";
    return -1;
  }

  absl::optional<webrtc::SdpType> type = webrtc::SdpTypeFromString(type_str);

  if (!type || *type != webrtc::SdpType::kAnswer) {
    RTC_LOG(LS_WARNING) << "Invalid input SDP: type is not answer";
    return -1;
  }

  if (!rtc::GetStringFromJsonObject(answer_sdp_json, kSessionDescriptionSdpName,
                                    &answer_sdp_str)) {
    RTC_LOG(LS_WARNING) << "Invalid input SDP: sdp";
    return -1;
  }

  webrtc::SdpParseError error;
  std::unique_ptr<webrtc::SessionDescriptionInterface> answer_sdp =
      webrtc::CreateSessionDescription(*type, answer_sdp_str, &error);
  if (!answer_sdp) {
    RTC_LOG(LS_WARNING)
        << "Can't parse received session description message. Error: "
        << error.description;
    return -1;
  }

  // 6. Once an answer is received from the remote peer, call
  // SetRemoteDescription with the remote answer.
  PromiseSDP answer_sdp_promise;
  peer_connection->SetRemoteDescription(
      SetSessionDescriptionObserver::Create(peer_connection, answer_sdp_promise)
          .get(),
      answer_sdp.release());

  auto answer_sdp_future = answer_sdp_promise.get_future();
  while (answer_sdp_future.wait_for(std::chrono::milliseconds(200)) !=
         std::future_status::ready) {
    std::cout << ".\n";
  }

  RTC_LOG(LS_INFO) << "All done, stream should be visible on the client.";

  RTC_LOG(LS_INFO) << "\n\nPress ENTER to exit...\n\n";
  std::cin.get();

  rtc::CleanupSSL();
  return 0;
}