import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import QtMultimedia 5.12
import QtQuick.Window 2.12
import AudioController 1.0
import QtGraphicalEffects 1.15

import "../components"

Item {
    id: root
    property string currentResponse: ""
    property string currentStatus: "Ready to chat with robot..."
    property StackView stackView
    property alias audioManager: audioManagerRef
    property var globalAllSteps: []

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

                    // Placeholder for layout balance
                    Item {
                        Layout.fillWidth: true
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

    // Minimap hình tròn ở góc trái trên
    Rectangle {
    id: minimapContainer
    anchors.top: parent.top
    anchors.left: parent.left
    anchors.margins: 15
    width: 80; height: 80
    radius: width / 2
    color: "#2c3e50"
    border.color: "#34495e"
    border.width: 2

    // Đặt transform origin để scale không bị "trôi"
    transformOrigin: Item.Center
    layer.enabled: true
    layer.smooth: true

    // Ảnh nguồn (ẩn), chừa 4px để không che viền
    Image {
        id: minimapSrc
        anchors.fill: parent
        anchors.margins: 4
        source: "qrc:/assets/map.png"
        fillMode: Image.PreserveAspectCrop
        smooth: true
        visible: false

        // Debug: báo nếu ảnh không load được
        onStatusChanged: {
            if (status === Image.Error) {
                console.warn("Minimap image failed to load:", source);
            }
        }
    }

    // Mặt nạ tròn (ẩn)
    Rectangle {
        id: minimapMask
        anchors.fill: minimapSrc
        radius: width / 2
        color: "white"      // chỉ dùng alpha làm mask
        visible: false
    }

    // Kết quả: ảnh cắt theo mặt nạ
    OpacityMask {
        anchors.fill: minimapSrc
        source: minimapSrc
        maskSource: minimapMask
    }

    MouseArea {
        id: hoverArea
        anchors.fill: parent
        hoverEnabled: true
        cursorShape: Qt.PointingHandCursor
        onEntered: {
            minimapContainer.scale = 1.1
            minimapContainer.border.color = "#3498db"
        }
        onExited: {
            minimapContainer.scale = 1.0
            minimapContainer.border.color = "#34495e"
        }
        onClicked: {
            if (root.stackView) {
                root.stackView.push(Qt.resolvedUrl("MapScreen.qml"), { "stackView": root.stackView })
            }
        }
    }

    Behavior on scale {
        NumberAnimation { duration: 150; easing.type: Easing.OutQuad }
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
            console.log("RobotFaceScreen: Received step data change signal")
            console.log("Step Number:", tssSocketBridge.currentStepNumber)
            console.log("Step Description:", tssSocketBridge.currentStepDescription)
            console.log("Image Base64 Length:", tssSocketBridge.currentImageBase64 ? tssSocketBridge.currentImageBase64.length : 0)
            console.log("Audio Base64 Length:", tssSocketBridge.currentAudioBase64 ? tssSocketBridge.currentAudioBase64.length : 0)
            console.log("Audio Base64 Preview:", tssSocketBridge.currentAudioBase64 ? tssSocketBridge.currentAudioBase64.substring(0, 100) + "..." : "null")
            
            // Show the TSS data received indicator
            tssDataIndicator.visible = true
            showAnimation.start()
            
            // Store step data globally
            console.log("RobotFaceScreen: About to create stepData:")
            console.log("  - tssSocketBridge.currentAudioBase64:", tssSocketBridge.currentAudioBase64)
            console.log("  - tssSocketBridge.currentAudioBase64 type:", typeof tssSocketBridge.currentAudioBase64)
            console.log("  - tssSocketBridge.currentAudioBase64 is null:", tssSocketBridge.currentAudioBase64 === null)
            console.log("  - tssSocketBridge.currentAudioBase64 is undefined:", tssSocketBridge.currentAudioBase64 === undefined)
            console.log("  - tssSocketBridge.currentAudioBase64 length:", tssSocketBridge.currentAudioBase64 ? tssSocketBridge.currentAudioBase64.length : 0)
            
            let stepData = {
                stepNumber: tssSocketBridge.currentStepNumber,
                stepDescription: tssSocketBridge.currentStepDescription,
                imageBase64: tssSocketBridge.currentImageBase64 || "",
                voiceBase64: tssSocketBridge.currentAudioBase64 || "",
                timestamp: Date.now()
            }
            
            console.log("RobotFaceScreen: Created stepData:")
            console.log("  - stepNumber:", stepData.stepNumber)
            console.log("  - imageBase64 length:", stepData.imageBase64.length)
            console.log("  - voiceBase64 length:", stepData.voiceBase64.length)
            console.log("  - voiceBase64 preview:", stepData.voiceBase64.substring(0, 100) + "...")
            console.log("  - voiceBase64 is null:", stepData.voiceBase64 === null)
            console.log("  - voiceBase64 is undefined:", stepData.voiceBase64 === undefined)
            console.log("  - voiceBase64 type:", typeof stepData.voiceBase64)
            
            // Add to global steps array
            if (!root.globalAllSteps) {
                root.globalAllSteps = []
            }
            
            // Check if step already exists
            let existingIndex = -1
            for (let i = 0; i < root.globalAllSteps.length; i++) {
                if (root.globalAllSteps[i].stepNumber === tssSocketBridge.currentStepNumber) {
                    existingIndex = i
                    break
                }
            }
            
            if (existingIndex >= 0) {
                root.globalAllSteps[existingIndex] = stepData
                console.log("RobotFaceScreen: Updated existing step", tssSocketBridge.currentStepNumber, "in global steps")
                console.log("RobotFaceScreen: Updated step voiceBase64 length:", root.globalAllSteps[existingIndex].voiceBase64.length)
            } else {
                root.globalAllSteps.push(stepData)
                console.log("RobotFaceScreen: Added new step", tssSocketBridge.currentStepNumber, "to global steps")
                console.log("RobotFaceScreen: Added step voiceBase64 length:", root.globalAllSteps[root.globalAllSteps.length - 1].voiceBase64.length)
            }
            
            console.log("RobotFaceScreen: Total steps in global:", root.globalAllSteps.length)
            console.log("RobotFaceScreen: Global step numbers:", JSON.stringify(root.globalAllSteps.map(s => s.stepNumber)))
            
            // Navigate only when step number is 1
            if (tssSocketBridge.currentStepNumber === 1) {
                console.log("RobotFaceScreen: Step 1 received, navigating to CareStepsScreen")
                navigateTimer.start()
            } else {
                console.log("RobotFaceScreen: Step", tssSocketBridge.currentStepNumber, "received, not navigating")
            }
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
            // console.log("RobotFaceScreen WebSocket Log:", message)
        }

        function onMessageReceived(message) {
            // console.log("RobotFaceScreen WebSocket Message:", message)
        }

        function onConnectionStatusChanged() {
            // console.log("RobotFaceScreen WebSocket Status Changed:", websocketBridge.isConnected)
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

            Row {
                anchors.horizontalCenter: parent.horizontalCenter
                spacing: 5

                Button {
                    text: "📡 Send"
                    width: 50
                    height: 25
                    enabled: audioManagerRef.hasRecordedData && websocketBridge.isConnected
                    background: Rectangle {
                        color: parent.enabled ? (parent.pressed ? "#e67e22" : "#f39c12") : "#7f8c8d"
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
                            websocketBridge.sendMessage(base64Data)
                            audioStatus.text = "📡 Sent via WebSocket"
                            console.log("=== SENDING AUDIO VIA WEBSOCKET ===")
                            console.log("WebSocket connected:", websocketBridge.isConnected)
                            console.log("Base64 data length:", base64Data.length, "characters")
                            console.log("Data preview:", base64Data.substring(0, 50) + "...")
                            console.log("=== END WEBSOCKET SEND ===")
                        }
                    }
                }

                Button {
                    text: "💾 Save"
                    width: 50
                    height: 25
                    enabled: audioManagerRef.hasRecordedData
                    background: Rectangle {
                        color: parent.enabled ? (parent.pressed ? "#27ae60" : "#2ecc71") : "#7f8c8d"
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
                        audioManagerRef.autoSaveAll()
                        audioStatus.text = "💾 Auto-saved files"
                        console.log("=== AUTO SAVE TRIGGERED ===")
                        console.log("Current directory:", Qt.resolvedUrl("."))
                        console.log("=== END AUTO SAVE ===")
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
                
                // Auto-save files when recording stops
                if (audioManagerRef.hasRecordedData) {
                    var audioFile = audioManagerRef.autoSaveAudio()
                    var base64File = audioManagerRef.autoSaveBase64()
                    
                    console.log("=== AUTO SAVE ON RECORDING STOP ===")
                    console.log("🎵 Recording completed - Files auto-saved")
                    if (audioFile.length > 0) {
                        console.log("📁 Audio file saved:", audioFile)
                    }
                    if (base64File.length > 0) {
                        console.log("📄 Base64 file saved:", base64File)
                    }
                    console.log("=== END AUTO SAVE ===")
                    
                    // Update status with file locations
                    if (audioFile.length > 0 && base64File.length > 0) {
                        audioStatus.text = "✅ Saved: " + audioFile.split('/').pop() + ", " + base64File.split('/').pop()
                    }
                    
                    // Auto-log base64 when recording stops
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
            
            function onAutoSaveCompleted(audioFile, base64File) {
                console.log("=== AUTO SAVE COMPLETED SIGNAL ===")
                console.log("📁 Audio file:", audioFile)
                console.log("📄 Base64 file:", base64File)
                console.log("=== END AUTO SAVE COMPLETED ===")
                
                audioStatus.text = "💾 Saved: " + audioFile.split('/').pop() + ", " + base64File.split('/').pop()
                audioStatus.color = "#27ae60"
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
