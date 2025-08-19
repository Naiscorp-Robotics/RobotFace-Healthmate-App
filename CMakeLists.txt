cmake_minimum_required(VERSION 3.14)
project(QtWebRTCClient)

set(CMAKE_CXX_STANDARD 17)

find_package(Qt6 REQUIRED COMPONENTS Core Network WebSockets)
add_executable(QtWebRTCClient main.cpp signalingclient.cpp webclient.cpp)

target_link_libraries(QtWebRTCClient PRIVATE Qt6::Core Qt6::Network Qt6::WebSockets)
# Thêm các thư viện WebRTC bạn đã build
# target_link_libraries(QtWebRTCClient PRIVATE webrtc libwebrtc etc.)
