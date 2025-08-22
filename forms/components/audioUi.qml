import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

ApplicationWindow {
    id: mainWindow
    width: 500
    height: 400
    title: "Professional Audio Recorder"
    visible: true
    color: "#f0f0f0"

    // AudioManager giả định
    Item {
        id: audioManager
        property bool isCapturing: false
        property bool isCapturing: false // Sửa lỗi chính tả

        signal captureStarted()
        signal captureStopped()
        signal audioPlayed()
        signal errorOccurred(string errorMessage)

        function startCapture() { isCapturing = true; captureStarted() }
        function stopCapture() { isCapturing = false; captureStopped() }
        function playAudio() { audioPlayed() }
    }

    ColumnLayout {
        anchors.centerIn: parent
        spacing: 30
        width: parent.width * 0.8

        // Header
        Text {
            text: "Audio Recorder"
            font.pixelSize: 24
            font.bold: true
            color: "#333"
            Layout.alignment: Qt.AlignHCenter
        }

        // Status indicator - Thay thế bằng component đơn giản
        Rectangle {
            id: statusIndicator
            width: 120
            height: 40
            radius: 20
            color: "#fff"
            border.color: "#ddd"
            Layout.alignment: Qt.AlignHCenter

            property string status: "ready"

            Text {
                anchors.centerIn: parent
                text: {
                    switch(parent.status) {
                    case "ready": return "READY"
                    case "recording": return "RECORDING"
                    case "playback_complete": return "PLAYED"
                    default: return "READY"
                    }
                }
                color: {
                    switch(parent.status) {
                    case "ready": return "green"
                    case "recording": return "red"
                    case "playback_complete": return "blue"
                    default: return "green"
                    }
                }
                font.bold: true
            }
        }

        // Record button - Thay thế bằng component đơn giản
        Rectangle {
            id: recordButton
            width: 80
            height: 80
            radius: 40
            color: recording ? "#ff4444" : "#44ff44"
            border.color: recording ? "#cc0000" : "#00cc00"
            border.width: 3
            Layout.alignment: Qt.AlignHCenter

            property bool recording: false

            Text {
                anchors.centerIn: parent
                text: recording ? "⏹" : "⏺"
                font.pixelSize: 30
                color: "white"
            }

            MouseArea {
                anchors.fill: parent
                onClicked: {
                    if (audioManager.isCapturing) {
                        audioManager.stopCapture()
                    } else {
                        audioManager.startCapture()
                    }
                }
            }
        }

        // Play button - Thay thế bằng component đơn giản
        Rectangle {
            id: playButton
            width: 80
            height: 80
            radius: 40
            color: enabled ? "#4444ff" : "#cccccc"
            border.color: enabled ? "#0000cc" : "#999999"
            border.width: 3
            Layout.alignment: Qt.AlignHCenter
            enabled: !audioManager.isCapturing

            Text {
                anchors.centerIn: parent
                text: "▶"
                font.pixelSize: 30
                color: "white"
            }

            MouseArea {
                anchors.fill: parent
                enabled: parent.enabled
                onClicked: audioManager.playAudio()
            }
        }

        // Audio level meter - Thay thế bằng component đơn giản
        Rectangle {
            id: levelMeter
            Layout.fillWidth: true
            Layout.preferredHeight: 30
            color: "#e0e0e0"
            radius: 5
            border.color: "#ccc"

            property bool recording: audioManager.isCapturing

            Rectangle {
                id: meterFill
                width: parent.width * (parent.recording ? (0.3 + Math.random() * 0.7) : 0)
                height: parent.height
                radius: parent.radius
                color: parent.recording ? "red" : "green"
                Behavior on width { NumberAnimation { duration: 100 } }
            }

            Timer {
                interval: 50
                running: true
                repeat: true
                onTriggered: {
                    if (parent.recording) {
                        meterFill.width = parent.width * (0.2 + Math.random() * 0.8)
                    }
                }
            }
        }
    }

    // Error Popup - Thay thế bằng Popup đơn giản
    Popup {
        id: errorPopup
        width: 300
        height: 150
        anchors.centerIn: parent
        modal: true
        closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutside

        function show(message) {
            errorText.text = message
            open()
        }

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 20

            Text {
                text: "Error"
                font.bold: true
                font.pixelSize: 18
                color: "red"
            }

            Text {
                id: errorText
                Layout.fillWidth: true
                Layout.fillHeight: true
                wrapMode: Text.Wrap
                color: "#333"
            }

            Button {
                text: "OK"
                Layout.alignment: Qt.AlignRight
                onClicked: errorPopup.close()
            }
        }
    }

    // Kết nối các signal của audioManager
    Connections {
        target: audioManager
        function onCaptureStarted() {
            recordButton.recording = true
            statusIndicator.status = "recording"
        }
        function onCaptureStopped() {
            recordButton.recording = false
            statusIndicator.status = "ready"
        }
        function onAudioPlayed() {
            statusIndicator.status = "playback_complete"
        }
        function onErrorOccurred(errorMessage) {
            errorPopup.show(errorMessage)
        }
    }
}
