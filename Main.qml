import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Window
import "./pages"
import "./components"
import Audio 1.0 // Import AudioManager

Window {
    width: Screen.width
    height: Screen.height
    visible: true
    title: qsTr("Robot Face Interface")
    color: "#000000"

    // Thiết lập full screen (tùy chọn)
    // visibility: Window.FullScreen

    // Global states
    property string currentResponse: ""
    property string currentStatus: "Ready to chat with robot..."

    // Audio Manager
    AudioManager {
        id: audioManager

        onErrorOccurred: function(message) {
            console.log("Audio Error:", message)
            currentStatus = "Audio Error: " + message
        }

        onCaptureStarted: {
            console.log("Audio capture started")
            currentStatus = "Recording audio..."
        }

        onCaptureStopped: {
            console.log("Audio capture stopped")
            currentStatus = "Audio recording complete"
        }

        onAudioPlayed: {
            console.log("Audio playback completed")
            currentStatus = "Audio playback finished"
        }

        onRecordingSaved: function(success) {
            if (success) {
                console.log("Recording saved successfully")
                currentStatus = "Recording saved"
            } else {
                console.log("Failed to save recording")
                currentStatus = "Failed to save recording"
            }
        }
    }

    StackView {
        id: stackView
        anchors.fill: parent

        // Smooth fade transition animations
        pushEnter: Transition {
            PropertyAnimation {
                property: "opacity"
                from: 0.0
                to: 1.0
                duration: 200
                easing.type: Easing.OutQuad
            }
        }

        pushExit: Transition {
            PropertyAnimation {
                property: "opacity"
                from: 1.0
                to: 0.0
                duration: 200
                easing.type: Easing.OutQuad
            }
        }

        popEnter: Transition {
            PropertyAnimation {
                property: "opacity"
                from: 0.0
                to: 1.0
                duration: 200
                easing.type: Easing.OutQuad
            }
        }

        popExit: Transition {
            PropertyAnimation {
                property: "opacity"
                from: 1.0
                to: 0.0
                duration: 200
                easing.type: Easing.OutQuad
            }
        }

        initialItem: RobotFaceScreen {
            id: robotFaceScreen
            stackView: stackView
            currentResponse: currentResponse
            currentStatus: currentStatus
            // BỎ DÒNG NÀY: audioManager: audioManager // Gây lỗi read-only property
            onResponseChanged: function(response) {
                currentResponse = response
            }
            onStatusChanged: function(status) {
                currentStatus = status
            }
        }
    }

    // WebSocket Panel (overlay)
    WebSocketPanel {
        anchors {
            top: parent.top
            right: parent.right
            margins: 20
        }
        z: 1000  // Ensure it's on top
    }

    // Audio Control Panel (overlay)
}
