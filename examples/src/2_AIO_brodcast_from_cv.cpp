// ENSURE webrtc_config IS INCLUDED FIRST
#include "webrtc/webrtc_config.h"
//

#include <atomic>
#include <broadrtc/Observers.hpp>
#include <cstdint>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <regex>
#include <set>
#include <thread>

#include "broadrtc/BroadcastWebRTC.hpp"
#include "jsoncpp/json/json.h"

class OpenCvSource : public broadrtc::BroadcastSource {
 public:
  OpenCvSource() {}

  virtual ~OpenCvSource() {
    if (IsRunning()) {
      Stop();
    }
  }

  bool Start() override {
    broadrtc::BroadcastSource::Start();

    m_isRunning = true;
    m_captureThread = std::thread(&OpenCvSource::CaptureLoop, this);
    return true;
  }

  bool Stop() override {
    broadrtc::BroadcastSource::Stop();

    m_isRunning = false;
    if (m_captureThread.joinable()) {
      m_captureThread.join();
    }

    return true;
  }

  bool IsRunning() override { return m_isRunning; }

  int GetWidth() override { return 640; }
  int GetHeight() override { return 360; }

  void OnNoAudience() override {
    std::cout << "Opencv source has no audience." << std::endl;
    this->Stop();
  }

 private:
  void CaptureLoop() {
    RTC_LOG(LS_VERBOSE) << "OpenCvSource::CaptureLoop";

    int frameNumber {0};
    cv::Mat frame(360, 640, CV_8UC3, cv::Scalar(0, 0, 0));
    while (m_isRunning) {
      frame.setTo(cv::Scalar(0, 0, 0));

      cv::putText(frame, std::to_string(frameNumber++), cv::Point(320, 180),
                  cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);

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

        this->OnFrame(videoFrame);
      } else {
        RTC_LOG(LS_ERROR) << "OpenCvSource conversion error:" << res;
      }

      // wait for 1/30th of a second
      std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }
  }

 private:
  std::thread m_captureThread;
  std::atomic_bool m_isRunning;
};

class MultiBroadcastWebRTC : public broadrtc::BroadcastWebRTC {
 public:
  MultiBroadcastWebRTC() = default;

  void OnChannelNotFound(const std::string& sourceId) override {
    RTC_LOG(LS_INFO) << "MultiBroadcastWebRTC::OnChannelNotFound requested "
                     << sourceId;

    rtc::scoped_refptr<broadrtc::BroadcastTrackSource> trackSource =
        broadrtc::BroadcastTrackSource::Create(
            std::make_unique<OpenCvSource>());

    m_mutexSourcesMap.lock();
    m_sources.insert({sourceId, std::move(trackSource)});
    m_mutexSourcesMap.unlock();
  }
};

std::atomic_bool disconnected = false;

class TerminatorWithLoggingConnectionObserver
    : public broadrtc::BasicConnectionObserver {
 public:
  void OnConnectionChange(
      webrtc::PeerConnectionInterface::PeerConnectionState new_state) override {
    // The previous state doesn't apply (it's not closed) and any
    // RTCIceTransports are in the "failed" state.
    if (new_state ==
        webrtc::PeerConnectionInterface::PeerConnectionState::kFailed) {
      if (m_peerConnection.get() != nullptr) {
        std::cout << "The connection will be terminated." << std::endl;
        m_peerConnection->Close();
        disconnected = true;
      } else {
        throw std::runtime_error(
            "The peer connection is null. Make sure to call "
            "SetPeerConnection or use your own "
            "webrtc::PeerConnectionObserver.");
      }
    }
  }
};

int main() {
  broadrtc::BroadcastWebRTC::InitWebRTC(broadrtc::BroadcastWebRTC::LS_WARNING);

  char read = 0;
  {
    MultiBroadcastWebRTC broadcast;

    std::string sourceId = "test";

    while (read != 'q') {
      // try {
      auto [clientId, offer] = broadcast.ConnectToChannel(
          sourceId,
          std::make_unique<TerminatorWithLoggingConnectionObserver>());
      // } catch :)

      std::cout << "clientId: " << clientId << std::endl;
      std::cout << "offer: " << offer << std::endl;

      // read answer from stdin
      std::cout << "Paste the answer here: " << std::endl;

      std::string answer;
      std::getline(std::cin, answer);

      try {
        broadcast.SetAnswer(clientId, answer);
      } catch (std::exception& e) {
        std::cout << "Error while setting answer: " << e.what() << std::endl;
        return -1;
      }

      std::cout << "Press enter to end connection" << std::endl;

      auto future = std::async(std::launch::async,
                               []() -> char { return std::cin.get(); });

      while (!disconnected && future.wait_for(std::chrono::milliseconds(418)) !=
                                  std::future_status::ready) {
      }

      disconnected = false;

      broadcast.EndConnection(clientId);  // OK if disconnected == true
    }
  }

  broadrtc::BroadcastWebRTC::DeinitWebRTC();

  return 0;
}