import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Window
import "./pages"
import "./components"


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
            stackView: stackView
            currentResponse: currentResponse
            currentStatus: currentStatus
            onResponseChanged: function(response) {
                currentResponse = response
            }
            onStatusChanged: function(status) {
                currentStatus = status
            }
        }
    }

    // Connect to WebSocket bridge signals
    Connections {
        target: websocketBridge

        function onLogMessage(message) {
            console.log("Main WebSocket Log:", message)
        }

        function onMessageReceived(message) {
            console.log("Main WebSocket Message:", message)
        }

        function onConnectionStatusChanged() {
            console.log("Main WebSocket Status Changed:", websocketBridge.isConnected)
        }
    }
}
