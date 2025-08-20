import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.Dialogs
import Audio 1.0

ApplicationWindow {
    id: mainWindow
    title: "Advanced Audio Recorder"
    width: 300
    height: 250
    visible: true

    AudioManager {
        id: audioManager

        onErrorOccurred: function(message) {
            statusText.text = "Lỗi: " + message
            statusText.color = "red"
        }

        onCaptureStarted: {
            statusText.text = "Đang ghi âm..."
            statusText.color = "green"
            recordButton.text = "Dừng ghi"
            recordingTimer.start()
        }

        onCaptureStopped: {
            statusText.text = "Đã dừng ghi âm"
            statusText.color = "black"
            recordButton.text = "Bắt đầu ghi"
            playButton.enabled = audioManager.hasRecordedData  // BỎ ()
            saveButton.enabled = audioManager.hasRecordedData  // BỎ ()
            recordingTimer.stop()
            recordingTime.text = "00:00"
            recordingTimer.repeatCount = 0
        }

        onAudioPlayed: {
            statusText.text = "Đã phát âm thanh"
            statusText.color = "blue"
        }

        onRecordingSaved: function(success) {
            if (success) {
                statusText.text = "Đã lưu bản ghi âm"
                statusText.color = "darkgreen"
            } else {
                statusText.text = "Lỗi khi lưu bản ghi"
                statusText.color = "red"
            }
        }
    }

    Timer {
        id: recordingTimer
        interval: 1000
        repeat: true
        onTriggered: {
            var seconds = Math.floor(recordingTimer.repeatCount % 60)
            var minutes = Math.floor(recordingTimer.repeatCount / 60)
            recordingTime.text = (minutes < 10 ? "0" + minutes : minutes) + ":" +
                                (seconds < 10 ? "0" + seconds : seconds)
            recordingTimer.repeatCount++
        }
        property int repeatCount: 0
    }

    FileDialog {
        id: saveDialog
        title: "Lưu bản ghi âm"
        fileMode: FileDialog.SaveFile
        nameFilters: ["Wave files (*.wav)"]
        onAccepted: {
            var filePath = saveDialog.selectedFile.toString().replace("file://", "")
            if (audioManager.saveToFile(filePath)) {
                statusText.text = "Đã lưu: " + filePath
            }
        }
    }

    FileDialog {
        id: loadDialog
        title: "Mở bản ghi âm"
        fileMode: FileDialog.OpenFile
        nameFilters: ["Wave files (*.wav)"]
        onAccepted: {
            var filePath = loadDialog.selectedFile.toString().replace("file://", "")
            if (audioManager.loadFromFile(filePath)) {
                statusText.text = "Đã tải: " + filePath
                playButton.enabled = true
                saveButton.enabled = true
            }
        }
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 20
        spacing: 15

        Text {
            id: title
            text: "Máy Ghi Âm"
            font.pixelSize: 24
            font.bold: true
            Layout.alignment: Qt.AlignHCenter
        }

        Text {
            id: statusText
            text: "Sẵn sàng"
            font.pixelSize: 16
            Layout.alignment: Qt.AlignHCenter
        }

        Text {
            id: recordingTime
            text: "00:00"
            font.pixelSize: 32
            font.bold: true
            color: "red"
            Layout.alignment: Qt.AlignHCenter
            visible: audioManager.isCapturing  // BỎ ()
        }

        RowLayout {
            Layout.alignment: Qt.AlignHCenter
            spacing: 15

            Button {
                id: recordButton
                text: "Bắt đầu ghi"
                onClicked: {
                    if (audioManager.isCapturing) {  // BỎ ()
                        audioManager.stopCapture()
                    } else {
                        if (audioManager.startCapture()) {
                            playButton.enabled = false
                            saveButton.enabled = false
                        }
                    }
                }
            }

            Button {
                id: playButton
                text: "Phát lại"
                enabled: audioManager.hasRecordedData  // BỎ ()
                onClicked: {
                    audioManager.playAudio()
                }
            }
        }

        RowLayout {
            Layout.alignment: Qt.AlignHCenter
            spacing: 15

            Button {
                id: saveButton
                text: "Lưu file"
                enabled: audioManager.hasRecordedData  // BỎ ()
                onClicked: {
                    var timestamp = new Date().toISOString().replace(/[:.]/g, "-")
                    var defaultFileName = "recording-" + timestamp + ".wav"
                    saveDialog.currentFile = defaultFileName
                    saveDialog.open()
                }
            }

            Button {
                id: loadButton
                text: "Mở file"
                onClicked: {
                    loadDialog.open()
                }
            }
        }

        // Text {
        //     text: "Thông tin âm thanh:"
        //     font.bold: true
        //     Layout.alignment: Qt.AlignLeft
        // }

        // GridLayout {
        //     columns: 2
        //     Layout.fillWidth: true

        //     Text { text: "Tần số mẫu:" }
        //     Text { text: audioManager.sampleRate() + " Hz" }  // GIỮ NGUYÊN () vì đây là function

        //     Text { text: "Số kênh:" }
        //     Text { text: audioManager.channelCount() }  // GIỮ NGUYÊN () vì đây là function

        //     Text { text: "Độ sâu bit:" }
        //     Text { text: audioManager.sampleSize() + " bit" }  // GIỮ NGUYÊN () vì đây là function
        // }

        Item { Layout.fillHeight: true } // Spacer
    }
}
