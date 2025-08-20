import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import QtMultimedia 5.12
import QtQuick.Window 2.12
import AudioController 1.0

import "../components"

Item {
    id: root
    property string currentResponse: ""
    property string currentStatus: "Ready to chat with robot..."
    property StackView stackView
    property alias audioManager: audioManagerRef

    AudioController {
        id: audioManagerRef
    }
    signal responseChanged(string response)
    signal statusChanged(string status)

    // Optimized Video Player with GStreamer backend
    OptimizedVideoPlayer {
        id: robotVideo
        anchors.fill: parent
        source: "qrc:/assets/blinking_face.mp4"
        autoPlay: true
        loops: -1 // infinite loops
        
        Component.onCompleted: {
            console.log("Optimized video player loaded, source: " + source)
            // Short delay before playing to ensure component is fully initialized
            playTimer.start()
        }
        
        onStarted: {
            console.log("Video started")
        }
        
        onStopped: {
            console.log("Video stopped")
        }
        
        onError: function(message) {
            console.error("Video error: " + message)
        }
        
        Timer {
            id: playTimer
            interval: 1000
            repeat: false
            onTriggered: {
                console.log("Starting video playback after delay")
                robotVideo.play()
            }
        }
    }

    // Overlay container cho tất cả controls
    Item {
        id: overlayContainer
        anchors.fill: parent

        // Bottom controls overlay
        Rectangle {
            id: bottomControls
            anchors.bottom: parent.bottom
            anchors.left: parent.left
            anchors.right: parent.right
            height: 100
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

                // Audio Test button row
                RowLayout {
                    Layout.fillWidth: true
                    spacing: 15

                    Button {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 40
                        text: "🎤 Audio Recorder"
                        background: Rectangle {
                            color: parent.pressed ? "#f39c12" : "#e67e22"
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
                                root.stackView.push(Qt.resolvedUrl("../components/AudioManager.qml"), {
                                    "stackView": root.stackView
                                })
                            }
                        }
                    }
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
        visible: false // We'll handle errors via the OptimizedVideoPlayer's error signal
    }

    // Debug info (ẩn khi video hoạt động)
    Text {
        anchors.top: parent.top
        anchors.left: parent.left
        anchors.margins: 10
        text: "Video Source: " + robotVideo.source +
              "\nPlaying: " + (robotVideo.playing ? "Yes" : "No")
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

    // Connect to WebSocket signals
    Connections {
        target: websocketBridge

        function onLogMessage(message) {
            console.log("RobotFaceScreen WebSocket Log:", message)
        }

        function onMessageReceived(message) {
            console.log("RobotFaceScreen WebSocket Message:", message)
        }

        function onConnectionStatusChanged() {
            console.log("RobotFaceScreen WebSocket Status Changed:", websocketBridge.isConnected)
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

    // Audio Control Panel (overlay)
    Rectangle {
        id: audioControlPanel
        anchors.top: parent.top
        anchors.right: parent.right
        anchors.margins: 20
        width: 200
        height: 150
        color: "#80000000"
        radius: 10
        visible: true

        Column {
            anchors.fill: parent
            anchors.margins: 10
            spacing: 8

            Text {
                text: "🎤 Audio Controls"
                color: "white"
                font.bold: true
                font.pixelSize: 12
                anchors.horizontalCenter: parent.horizontalCenter
            }

            Row {
                anchors.horizontalCenter: parent.horizontalCenter
                spacing: 10

                Button {
                    id: recordBtn
                    text: audioManagerRef.isCapturing ? "⏹️ Stop" : "🔴 Record"
                    width: 80
                    height: 30
                    background: Rectangle {
                        color: parent.pressed ? "#e74c3c" : "#c0392b"
                        radius: 5
                    }
                    contentItem: Text {
                        text: parent.text
                        color: "white"
                        font.pixelSize: 10
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                    onClicked: {
                        if (audioManagerRef.isCapturing) {
                            audioManagerRef.stopCapture()
                        } else {
                            audioManagerRef.startCapture()
                        }
                    }
                }

                Button {
                    text: "▶️ Play"
                    width: 80
                    height: 30
                    enabled: audioManagerRef.hasRecordedData
                    background: Rectangle {
                        color: parent.enabled ? (parent.pressed ? "#27ae60" : "#2ecc71") : "#7f8c8d"
                        radius: 5
                    }
                    contentItem: Text {
                        text: parent.text
                        color: "white"
                        font.pixelSize: 10
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                    onClicked: {
                        audioManagerRef.playAudio()
                    }
                }
            }

            Row {
                anchors.horizontalCenter: parent.horizontalCenter
                spacing: 5

                Button {
                    text: "📄 Base64"
                    width: 70
                    height: 25
                    enabled: audioManagerRef.hasRecordedData
                    background: Rectangle {
                        color: parent.enabled ? (parent.pressed ? "#3498db" : "#2980b9") : "#7f8c8d"
                        radius: 3
                    }
                    contentItem: Text {
                        text: parent.text
                        color: "white"
                        font.pixelSize: 8
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                    onClicked: {
                        var base64Data = audioManagerRef.getWavAsBase64()
                        if (base64Data.length > 0) {
                            audioStatus.text = "📄 Base64: " + base64Data.substring(0, 20) + "..."
                            console.log("=== AUDIO BASE64 WAV OUTPUT ===")
                            console.log("Data Length:", base64Data.length, "characters")
                            console.log("Sample Rate: 44100 Hz, Channels: 1, Format: 16-bit PCM")
                            console.log("Base64 WAV Data:")
                            console.log(base64Data)
                            console.log("=== END BASE64 WAV OUTPUT ===")
                        }
                    }
                }

                Button {
                    text: "📋 Copy"
                    width: 50
                    height: 25
                    enabled: audioManagerRef.hasRecordedData
                    background: Rectangle {
                        color: parent.enabled ? (parent.pressed ? "#9b59b6" : "#8e44ad") : "#7f8c8d"
                        radius: 3
                    }
                    contentItem: Text {
                        text: parent.text
                        color: "white"
                        font.pixelSize: 8
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                    onClicked: {
                        var base64Data = audioManagerRef.getWavAsBase64()
                        var rawBase64 = audioManagerRef.getAudioAsBase64()
                        
                        console.log("=== AUDIO BASE64 COPY OUTPUT ===")
                        console.log("🎵 WAV FORMAT (with header):")
                        console.log("Length:", base64Data.length, "characters")
                        console.log("Data:", base64Data)
                        console.log("")
                        console.log("🎵 RAW PCM FORMAT (audio only):")
                        console.log("Length:", rawBase64.length, "characters") 
                        console.log("Data:", rawBase64)
                        console.log("=== END BASE64 COPY OUTPUT ===")
                        
                        audioStatus.text = "📋 Base64 logged to console"
                    }
                }
            }

            Text {
                id: audioStatus
                text: audioManagerRef.isCapturing ? "🔴 Recording..." : 
                      audioManagerRef.hasRecordedData ? "✅ Ready to play" : "⭕ No recording"
                color: audioManagerRef.isCapturing ? "#e74c3c" : 
                       audioManagerRef.hasRecordedData ? "#2ecc71" : "#f39c12"
                font.pixelSize: 10
                anchors.horizontalCenter: parent.horizontalCenter
                wrapMode: Text.Wrap
                width: parent.width - 20
                horizontalAlignment: Text.AlignHCenter
            }
        }

        // Audio status connections
        Connections {
            target: audioManagerRef
            
            function onErrorOccurred(message) {
                audioStatus.text = "❌ Error: " + message
                audioStatus.color = "#e74c3c"
            }
            
            function onCaptureStarted() {
                audioStatus.text = "🔴 Recording..."
                audioStatus.color = "#e74c3c"
            }
            
            function onCaptureStopped() {
                audioStatus.text = "✅ Recording complete"
                audioStatus.color = "#2ecc71"
                
                // Auto-log base64 when recording stops
                if (audioManagerRef.hasRecordedData) {
                    var base64Data = audioManagerRef.getWavAsBase64()
                    var rawBase64 = audioManagerRef.getAudioAsBase64()
                    
                    console.log("=== AUTO BASE64 LOG - RECORDING STOPPED ===")
                    console.log("🎵 Recording completed - Base64 data available")
                    console.log("WAV Format Length:", base64Data.length, "characters")
                    console.log("Raw PCM Length:", rawBase64.length, "characters")
                    console.log("📄 WAV Base64 (with header):", base64Data)
                    console.log("📄 Raw PCM Base64 (audio only):", rawBase64)
                    console.log("=== END AUTO BASE64 LOG ===")
                }
            }
            
            function onAudioPlayed() {
                audioStatus.text = "🔊 Playback finished"
                audioStatus.color = "#3498db"
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
