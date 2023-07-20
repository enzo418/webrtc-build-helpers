// ENSURE webrtc_config IS INCLUDED FIRST
#include "webrtc/webrtc_config.h"
//

#include <atomic>
#include <cstdint>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <optional>
#include <regex>
#include <set>
#include <thread>

#include "broadrtc/BroadcastWebRTC.hpp"
#include "jsoncpp/json/json.h"

class OpenCvSource : public broadrtc::BroadcastSource {
 public:
  OpenCvSource(const std::string& pUri) : uri(pUri) {}

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

  int GetWidth() override { return sourceSize.width; }
  int GetHeight() override { return sourceSize.height; }

  void OnNoAudience() override {
    std::cout << "Opencv source has no audience." << std::endl;
    this->Stop();
  }

 private:
  void CaptureLoop() {
    RTC_LOG(LS_VERBOSE) << "OpenCvSource::CaptureLoop";

    if (!m_capture.open(uri)) {
      RTC_LOG(LS_ERROR) << "OpenCvSource::CaptureLoop failed to open uri: "
                        << uri;
      return;
    }

    sourceSize = cv::Size(m_capture.get(cv::CAP_PROP_FRAME_WIDTH),
                          m_capture.get(cv::CAP_PROP_FRAME_HEIGHT));

    cv::Mat frame;
    while (m_isRunning) {
      // get frame
      if (!m_capture.read(frame)) {
        RTC_LOG(LS_ERROR) << "OpenCvSource::CaptureLoop failed to read frame";
        break;
      }

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

  cv::VideoCapture m_capture;
  std::string uri;
  cv::Size sourceSize;
};

class MultiBroadcastWebRTC : public broadrtc::BroadcastWebRTC {
 public:
  MultiBroadcastWebRTC() = default;

  void OnChannelNotFound(const std::string& sourceId) override {
    RTC_LOG(LS_INFO) << "MultiBroadcastWebRTC::OnChannelNotFound requested "
                     << sourceId;

    rtc::scoped_refptr<broadrtc::BroadcastTrackSource> trackSource =
        broadrtc::BroadcastTrackSource::Create(
            std::make_unique<OpenCvSource>("a.mp4"));

    m_mutexSourcesMap.lock();
    m_sources.insert({sourceId, std::move(trackSource)});
    m_mutexSourcesMap.unlock();
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
      auto [clientId, offer] = broadcast.ConnectToChannel(sourceId);
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

      read = std::cin.get();

      broadcast.EndConnection(clientId);
    }
  }

  broadrtc::BroadcastWebRTC::DeinitWebRTC();

  return 0;
}