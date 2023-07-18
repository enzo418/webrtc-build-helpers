#pragma once

#include "webrtc/webrtc_config.h"
//

#include <memory>
#include <string>
#include <vector>

#include "webrtc/api/peer_connection_interface.h"
namespace broadrtc {
  struct Peer {
    typedef std::string PeerID;
    PeerID id;
    rtc::scoped_refptr<webrtc::PeerConnectionInterface> connection;
    std::unique_ptr<webrtc::PeerConnectionObserver> connObserver;
  };
}  // namespace broadrtc