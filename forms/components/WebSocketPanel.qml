import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12

Rectangle {
    id: root
    width: 300
    height: 200
    color: "#f0f0f0"
    border.color: "#cccccc"
    border.width: 1
    radius: 8

    property bool showDetails: false

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 10
        spacing: 8

        // Header
        RowLayout {
            Layout.fillWidth: true
            spacing: 8

            Text {
                text: "🌐 WebSocket Status"
                font.bold: true
                font.pixelSize: 14
            }

            Rectangle {
                width: 12
                height: 12
                radius: 6
                color: websocketBridge.isConnected ? "#4CAF50" : "#f44336"
            }

            Item { Layout.fillWidth: true }

            Button {
                text: showDetails ? "Hide" : "Show"
                onClicked: showDetails = !showDetails
            }
        }

        // Connection Status
        Text {
            text: "Status: " + (websocketBridge.isConnected ? "Connected" : "Disconnected")
            color: websocketBridge.isConnected ? "#4CAF50" : "#f44336"
            font.pixelSize: 12
        }

        // Controls (only show when details are visible)
        ColumnLayout {
            visible: showDetails
            spacing: 5

            RowLayout {
                spacing: 5

                Button {
                    text: websocketBridge.isConnected ? "Disconnect" : "Connect"
                    onClicked: {
                        if (websocketBridge.isConnected) {
                            websocketBridge.disconnectFromServer()
                        } else {
                            websocketBridge.connectToServer()
                        }
                    }
                }

                TextField {
                    id: messageInput
                    placeholderText: "Enter message..."
                    Layout.fillWidth: true
                    onAccepted: {
                        if (text.length > 0) {
                            websocketBridge.sendMessage(text)
                            text = ""
                        }
                    }
                }

                Button {
                    text: "Send"
                    onClicked: {
                        if (messageInput.text.length > 0) {
                            websocketBridge.sendMessage(messageInput.text)
                            messageInput.text = ""
                        }
                    }
                }
            }

            // Log area
            Rectangle {
                Layout.fillWidth: true
                Layout.preferredHeight: 80
                color: "white"
                border.color: "#cccccc"
                border.width: 1

                ScrollView {
                    anchors.fill: parent
                    anchors.margins: 5

                    TextArea {
                        id: logArea
                        readOnly: true
                        text: "WebSocket log will appear here..."
                        font.family: "Consolas"
                        font.pixelSize: 10
                        wrapMode: TextArea.Wrap
                    }
                }
            }
        }

        Item { Layout.fillHeight: true }
    }

    // Connect to WebSocket bridge signals
    Connections {
        target: websocketBridge ? websocketBridge : null

        function onLogMessage(message) {
            if (logArea && logArea.text !== undefined) {
                logArea.text += "\n" + message
                // Auto-scroll to bottom
                logArea.cursorPosition = logArea.length
            }
        }

        function onConnectionStatusChanged() {
            // Status will be updated automatically via property binding
            console.log("WebSocket connection status changed: " + 
                       (websocketBridge ? websocketBridge.isConnected : "unknown"))
        }

        function onMessageReceived(message) {
            // Handle received messages if needed
            console.log("WebSocket message received: " + message)
        }
    }

    // Auto-connect on startup
    Component.onCompleted: {
        // WebSocketBridge now auto-connects automatically
        // No manual connection needed
    }
}
