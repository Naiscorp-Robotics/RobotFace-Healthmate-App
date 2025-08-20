import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

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
                text: "🌐 TSS Socket Status"
                font.bold: true
                font.pixelSize: 14
            }

            Rectangle {
                width: 12
                height: 12
                radius: 6
                color: tssSocketBridge.isConnected ? "#4CAF50" : "#f44336"
            }

            Item { Layout.fillWidth: true }

            Button {
                text: showDetails ? "Hide" : "Show"
                onClicked: showDetails = !showDetails
            }
        }

        // Connection Status
        Text {
            text: "Status: " + (tssSocketBridge.isConnected ? "Connected" : "Disconnected")
            color: tssSocketBridge.isConnected ? "#4CAF50" : "#f44336"
            font.pixelSize: 12
        }

        // Current Step Data (only show when connected and has data)
        ColumnLayout {
            visible: tssSocketBridge.isConnected && tssSocketBridge.currentStepNumber > 0
            spacing: 2

            Text {
                text: "📋 Current Step: " + tssSocketBridge.currentStepNumber
                font.pixelSize: 11
                color: "#666"
            }

            Text {
                text: tssSocketBridge.currentStepDescription
                font.pixelSize: 10
                color: "#888"
                wrapMode: Text.Wrap
                Layout.fillWidth: true
            }
        }

        // Controls (only show when details are visible)
        ColumnLayout {
            visible: showDetails
            spacing: 5

            RowLayout {
                spacing: 5

                Button {
                    text: tssSocketBridge.isConnected ? "Disconnect" : "Connect"
                    onClicked: {
                        if (tssSocketBridge.isConnected) {
                            tssSocketBridge.disconnectFromServer()
                        } else {
                            tssSocketBridge.connectToServer()
                        }
                    }
                }

                TextField {
                    id: messageInput
                    placeholderText: "Enter message..."
                    Layout.fillWidth: true
                    onAccepted: {
                        if (text.length > 0) {
                            tssSocketBridge.sendMessage(text)
                            text = ""
                        }
                    }
                }

                Button {
                    text: "Send"
                    onClicked: {
                        if (messageInput.text.length > 0) {
                            tssSocketBridge.sendMessage(messageInput.text)
                            messageInput.text = ""
                        }
                    }
                }
            }

            // Step request controls
            RowLayout {
                spacing: 5

                Text {
                    text: "Step:"
                    font.pixelSize: 12
                }

                SpinBox {
                    id: stepNumberInput
                    from: 1
                    to: 10
                    value: 1
                    editable: true
                }

                Button {
                    text: "Request Step"
                    onClicked: {
                        tssSocketBridge.sendStepRequest(stepNumberInput.value, "Step " + stepNumberInput.value)
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
                        text: "TSS Socket log will appear here..."
                        font.family: "Consolas"
                        font.pixelSize: 10
                        wrapMode: TextArea.Wrap
                    }
                }
            }
        }

        Item { Layout.fillHeight: true }
    }

    // Connect to TSS Socket bridge signals
    Connections {
        target: tssSocketBridge

        function onLogMessage(message) {
            logArea.text += "\n" + message
            // Auto-scroll to bottom
            logArea.cursorPosition = logArea.length
            console.log("TSS Log Message:", message)
        }

        function onConnectionStatusChanged() {
            // Status will be updated automatically via property binding
        }

        function onMessageReceived(message) {
            // Handle received messages if needed
            console.log("TSS Message Received:", message)
            logArea.text += "\n📨 Message: " + message
            logArea.cursorPosition = logArea.length
        }

        function onStepDataChanged() {
            // Handle step data changes
            console.log("TSS Step Data Changed:", tssSocketBridge.currentStepNumber, tssSocketBridge.currentStepDescription)
            logArea.text += "\n📋 Step " + tssSocketBridge.currentStepNumber + " received: " + tssSocketBridge.currentStepDescription
            logArea.cursorPosition = logArea.length
        }
    }

    // Auto-connect on startup
    Component.onCompleted: {
        // Auto-connect to TSS WebSocket server
        tssSocketBridge.connectToServer()
        logArea.text += "\n🔄 Auto-connecting to TSS server..."
    }
}
