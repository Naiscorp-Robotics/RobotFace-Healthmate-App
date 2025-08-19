import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtMultimedia
import QtQuick.Window

import "../components"

Item {
    id: root
    property string currentResponse: ""
    property string currentStatus: "Ready to chat with robot..."
    property StackView stackView

    signal responseChanged(string response)
    signal statusChanged(string status)

    Rectangle {
        anchors.fill: parent
        color: "#1a1a1a"

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 5
            spacing: 5

            Text {
                text: "🤖 Robot Face Interface"
                font.pixelSize: 16
                font.bold: true
                color: "#ffffff"
                Layout.alignment: Qt.AlignHCenter
            }

            // Video Player thay cho mắt robot
            Rectangle {
                Layout.fillWidth: true
                Layout.fillHeight: true
                Layout.preferredHeight: 0
                color: "#000000"
                radius: 3
                border.width: 1
                border.color: "#ffffff"

                // Video {
                //     id: robotVideo
                //     anchors.fill: parent
                //     anchors.margins: 5
                //     source: Qt.resolvedUrl("../assets/robot_face.mp4")
                    
                //     // Tự động phát và lặp lại
                //     autoPlay: true
                //     loops: MediaPlayer.Infinite
                    
                //     // Đảm bảo video hiển thị đúng tỷ lệ
                //     fillMode: VideoOutput.PreserveAspectFit
                // }

                Video {
                    id: robotVideo
                    anchors.fill: parent
                    anchors.margins: 2
                    source: "file:///F:/Study-Work/Naiscorp/QT/RobotFace-Healthmate-App/assets/blinking_face.mp4"
                    autoPlay: true
                    loops: MediaPlayer.Infinite
                    fillMode: VideoOutput.PreserveAspectFit
                }


                // Nút điều khiển video (tùy chọn)
                Row {
                    anchors.bottom: parent.bottom
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.margins: 10
                    spacing: 10
                    visible: false // Ẩn nút điều khiển, chỉ hiện khi cần

                    Button {
                        text: robotVideo.playbackState === MediaPlayer.PlayingState ? "⏸️" : "▶️"
                        onClicked: {
                            if (robotVideo.playbackState === MediaPlayer.PlayingState) {
                                robotVideo.pause()
                            } else {
                                robotVideo.play()
                            }
                        }
                    }

                    Button {
                        text: "🔄"
                        onClicked: robotVideo.position = 0
                    }
                }

                // Hiển thị thông báo nếu video không tải được
                Text {
                    anchors.centerIn: parent
                    text: "Video không tải được\nVui lòng kiểm tra đường dẫn file"
                    color: "#ffffff"
                    font.pixelSize: 16
                    horizontalAlignment: Text.AlignHCenter
                    visible: robotVideo.status === MediaPlayer.InvalidMedia
                }
                
                // Debug info (ẩn khi video hoạt động)
                Text {
                    anchors.top: parent.top
                    anchors.left: parent.left
                    anchors.margins: 10
                    text: "Video Status: " + robotVideo.status + 
                          "\nError: " + robotVideo.error + 
                          "\nPlaying: " + (robotVideo.playbackState === MediaPlayer.PlayingState ? "Yes" : "No") +
                          "\nDuration: " + Math.round(robotVideo.duration/1000) + "s" +
                          "\nSource: " + robotVideo.source
                    color: "#ffffff"
                    font.pixelSize: 10
                    visible: false // Ẩn debug text
                }
            }

            // Input box
            InputBox {
                Layout.fillWidth: true
                Layout.preferredHeight: 60
                onSendMessage: function(message) {
                        handleSendMessage(message)
                }
            }

            // Reset button
            Button {
                text: "Reset Memory"
                Layout.fillWidth: true
                Layout.preferredHeight: 30
                background: Rectangle {
                    color: parent.pressed ? "#aa0000" : "#cc0000"
                    radius: 6
                }
                
                onClicked: resetMemory()
            }

            // Open Image button
            Button {
                text: "📷 Open Image"
                Layout.fillWidth: true
                Layout.preferredHeight: 30
                background: Rectangle {
                    color: parent.pressed ? "#555555" : "#333333"
                    radius: 6
                }
                contentItem: Text {
                    text: parent.text
                    color: "#ffffff"
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }
                
                onClicked: {
                    if (root.stackView) {
                        root.stackView.push(Qt.resolvedUrl("ImageScreen.qml"), {
                            "stackView": root.stackView
                        })
                    }
                }
            }

            // Status
            Text {
                text: root.currentStatus
                font.pixelSize: 10
                color: "#888888"
                Layout.alignment: Qt.AlignHCenter
            }
        }
    }

    // Hàm gửi message
    function handleSendMessage(message) {
        root.statusChanged("Sending message...")

        var request = new XMLHttpRequest()
        request.open("POST", "http://192.168.1.122:8989/qa_bot/send_message")
        request.setRequestHeader("Content-Type", "application/json")

        request.onreadystatechange = function() {
            if (request.readyState === XMLHttpRequest.DONE) {
                if (request.status === 200) {
                    try {
                        var response = JSON.parse(request.responseText)
                        var responseText = response.response || response.message || request.responseText
                        root.responseChanged(responseText)
                        root.stackView.push(Qt.resolvedUrl("ResponseScreen.qml"), {
                            "stackView": root.stackView,
                            "response": responseText
                        })
                    } catch (e) {
                        root.responseChanged(request.responseText)
                        root.stackView.push(Qt.resolvedUrl("ResponseScreen.qml"), {
                            "stackView": root.stackView,
                            "response": request.responseText
                        })
                    }
                } else {
                    var errorText = "Error: " + request.status + " - " + request.statusText
                    root.responseChanged(errorText)
                    root.stackView.push(Qt.resolvedUrl("ResponseScreen.qml"), {
                        "stackView": root.stackView,
                        "response": errorText
                    })
                }
            }
        }

        request.onerror = function() {
            var errorText = "Network error occurred"
            root.responseChanged(errorText)
            root.stackView.push(Qt.resolvedUrl("ResponseScreen.qml"), {
                "stackView": root.stackView,
                "response": errorText
            })
        }

        var data = JSON.stringify({ "message": message })
        request.send(data)
    }

    function resetMemory() {
        root.statusChanged("Resetting memory...")

        var request = new XMLHttpRequest()
        request.open("POST", "http://192.168.1.122:8989/qa_bot/reset_memory")
        request.setRequestHeader("Content-Type", "application/json")

        request.onreadystatechange = function() {
            if (request.readyState === XMLHttpRequest.DONE) {
                if (request.status === 200) {
                    root.statusChanged("Memory reset successfully!")
                } else {
                    root.statusChanged("Error resetting memory: " + request.status)
                }
            }
        }

        request.onerror = function() {
            root.statusChanged("Network error while resetting memory")
        }

        request.send()
    }
}
