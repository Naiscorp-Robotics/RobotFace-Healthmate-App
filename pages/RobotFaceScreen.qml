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

    // Video Player full màn hình
    Video {
        id: robotVideo
        anchors.fill: parent
        source: "file:///F:/Study-Work/Naiscorp/QT/RobotFace-Healthmate-App/assets/blinking_face.mp4"
        autoPlay: true
        loops: MediaPlayer.Infinite
        fillMode: VideoOutput.PreserveAspectFit
    }

    // Overlay container cho tất cả controls
    Item {
        id: overlayContainer
        anchors.fill: parent

        // Header với title
        Rectangle {
            id: header
            anchors.top: parent.top
            anchors.left: parent.left
            anchors.right: parent.right
            height: 60
            color: "transparent"

            Text {
                anchors.centerIn: parent
                text: "🤖 Robot Face Interface"
                font.pixelSize: 20
                font.bold: true
                color: "#ffffff"
                style: Text.Outline
                styleColor: "#000000"
            }
        }

        // Bottom controls overlay
        Rectangle {
            id: bottomControls
            anchors.bottom: parent.bottom
            anchors.left: parent.left
            anchors.right: parent.right
            height: 260
            color: "transparent"

            // Semi-transparent background cho controls
            Rectangle {
                anchors.fill: parent
                color: "#80000000"
                radius: 15
            }

            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 20
                spacing: 15

                // Input box
                // InputBox {
                //     Layout.fillWidth: true
                //     Layout.preferredHeight: 70
                //     onSendMessage: function(message) {
                //         handleSendMessage(message)
                //     }
                // }

                // Control buttons row
                RowLayout {
                    Layout.fillWidth: true
                    spacing: 15

                    // Reset button
                    // Button {
                    //     Layout.fillWidth: true
                    //     Layout.preferredHeight: 40
                    //     text: "Reset Memory "
                    //     background: Rectangle {
                    //         color: parent.pressed ? "#aa0000" : "#cc0000"
                    //         radius: 8
                    //     }
                    //     contentItem: Text {
                    //         text: parent.text
                    //         color: "#ffffff"
                    //         horizontalAlignment: Text.AlignHCenter
                    //         verticalAlignment: Text.AlignVCenter
                    //         font.pixelSize: 14
                    //         font.bold: true
                    //     }

                    //     onClicked: resetMemory()
                    // }

                    // Open Image button
                    // Button {
                    //     Layout.fillWidth: true
                    //     Layout.preferredHeight: 40
                    //     text: "📷 Open Image"
                    //     background: Rectangle {
                    //         color: parent.pressed ? "#555555" : "#333333"
                    //         radius: 8
                    //     }
                    //     contentItem: Text {
                    //         text: parent.text
                    //         color: "#ffffff"
                    //         horizontalAlignment: Text.AlignHCenter
                    //         verticalAlignment: Text.AlignVCenter
                    //         font.pixelSize: 14
                    //         font.bold: true
                    //     }

                    //     onClicked: {
                    //         if (root.stackView) {
                    //             root.stackView.push(Qt.resolvedUrl("ImageScreen.qml"), {
                    //                 "stackView": root.stackView
                    //             })
                    //         }
                    //     }
                    // }
                }

                // Care Steps and Map buttons row
                RowLayout {
                    Layout.fillWidth: true
                    spacing: 15

                    // Care Steps button
                    Button {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 40
                        text: "📋 Hướng dẫn chăm sóc"
                        background: Rectangle {
                            color: parent.pressed ? "#8e44ad" : "#9b59b6"
                            radius: 8
                        }
                        contentItem: Text {
                            text: parent.text
                            color: "#ffffff"
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                            font.pixelSize: 13
                            font.bold: true
                        }

                        onClicked: {
                            if (root.stackView) {
                                root.stackView.push(Qt.resolvedUrl("CareStepsScreen.qml"), {
                                    "stackView": root.stackView
                                })
                            }
                        }
                    }

                    // Map button
                    Button {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 40
                        text: "🗺️ Bản đồ"
                        background: Rectangle {
                            color: parent.pressed ? "#27ae60" : "#2ecc71"
                            radius: 8
                        }
                        contentItem: Text {
                            text: parent.text
                            color: "#ffffff"
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                            font.pixelSize: 13
                            font.bold: true
                        }

                        onClicked: {
                            if (root.stackView) {
                                root.stackView.push(Qt.resolvedUrl("MapScreen.qml"), {
                                    "stackView": root.stackView
                                })
                            }
                        }
                    }
                }

                // Status
                Text {
                    text: root.currentStatus
                    font.pixelSize: 12
                    color: "#cccccc"
                    Layout.alignment: Qt.AlignHCenter
                }
            }
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

    // TSS Socket Status Indicator
    Rectangle {
        anchors.top: parent.top
        anchors.right: parent.right
        anchors.margins: 10
        width: 120
        height: 30
        color: tssSocketBridge.isConnected ? "#4CAF50" : "#f44336"
        radius: 15
        opacity: 0.8

        Text {
            anchors.centerIn: parent
            text: tssSocketBridge.isConnected ? "TSS Connected" : "TSS Disconnected"
            color: "white"
            font.pixelSize: 10
            font.bold: true
        }
    }

    // TSS Data Received Indicator
    Rectangle {
        id: tssDataIndicator
        anchors.top: parent.top
        anchors.right: parent.right
        anchors.margins: 10
        anchors.topMargin: 45
        width: 120
        height: 25
        color: "#2196F3"
        radius: 12
        opacity: 0
        visible: false

        Text {
            anchors.centerIn: parent
            text: "📡 TSS Data Received"
            color: "white"
            font.pixelSize: 9
            font.bold: true
        }

        // Navigation notification
        Text {
            anchors.top: parent.bottom
            anchors.topMargin: 5
            anchors.horizontalCenter: parent.horizontalCenter
            text: "Navigating to Care Steps..."
            color: "#ffffff"
            font.pixelSize: 8
            font.bold: true
            style: Text.Outline
            styleColor: "#000000"
            opacity: tssDataIndicator.visible ? 1 : 0
            Behavior on opacity {
                NumberAnimation { duration: 200 }
            }
        }

        // Animation for showing the indicator
        SequentialAnimation on opacity {
            id: showAnimation
            running: false
            NumberAnimation { to: 1.0; duration: 300 }
            PauseAnimation { duration: 2000 }
            NumberAnimation { to: 0.0; duration: 300 }
            onFinished: {
                tssDataIndicator.visible = false
            }
        }
    }

    // Connect to TSS Socket signals
    Connections {
        target: tssSocketBridge

        function onStepDataChanged() {
            console.log("RobotFaceScreen: Received TSS step data, navigating to CareStepsScreen")
            console.log("Step Number:", tssSocketBridge.currentStepNumber)
            console.log("Step Description:", tssSocketBridge.currentStepDescription)
            
            // Show the TSS data received indicator
            tssDataIndicator.visible = true
            showAnimation.start()
            
            // Navigate to CareStepsScreen after a short delay
            navigateTimer.start()
        }

        function onLogMessage(message) {
            console.log("RobotFaceScreen TSS Log:", message)
        }

        function onMessageReceived(message) {
            console.log("RobotFaceScreen TSS Message:", message)
        }
    }

    // Timer for delayed navigation
    Timer {
        id: navigateTimer
        interval: 1500 // 1.5 seconds delay
        repeat: false
        onTriggered: {
            if (root.stackView) {
                console.log("RobotFaceScreen: Navigating to CareStepsScreen")
                root.stackView.push(Qt.resolvedUrl("CareStepsScreen.qml"), {
                    "stackView": root.stackView
                })
            }
        }
    }

    // Auto-connect to TSS server when screen loads
    Component.onCompleted: {
        console.log("RobotFaceScreen: Component completed, connecting to TSS server")
        if (tssSocketBridge && !tssSocketBridge.isConnected) {
            tssSocketBridge.connectToServer()
        }
    }
}
