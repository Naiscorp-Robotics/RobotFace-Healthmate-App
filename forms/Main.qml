import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import QtQuick.Window 2.12
import "pages"
import "components"


Window {
    width: 800
    height: 600
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
        // RobotFaceScreen {
        //     anchors.fill: parent
        //     currentResponse: currentResponse
        //     currentStatus: currentStatus
        //     onResponseChanged: function(response) {
        //         currentResponse = response
        //     }
        //     onStatusChanged: function(status) {
        //         currentStatus = status
        //     }
        // }
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
