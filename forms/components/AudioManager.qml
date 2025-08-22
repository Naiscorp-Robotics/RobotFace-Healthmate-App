import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.Dialogs 1.3
import AudioController 1.0

ApplicationWindow {
    id: mainWindow
    title: "Advanced Audio Recorder"
    width: 300
    height: 250
    visible: true

    AudioController {
        id: audioManager

        onErrorOccurred: function(message) {
            console.log("Audio Error:", message)
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
                console.log("Recording saved successfully")
                statusText.text = "Đã lưu bản ghi âm"
                statusText.color = "darkgreen"
            } else {
                console.log("Failed to save recording")
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
            console.log("Saving to file:", filePath)
            console.log("File path type:", typeof filePath)
            console.log("File path length:", filePath.length)
            
            // Kiểm tra xem có dữ liệu audio không
            if (!audioManager.hasRecordedData) {
                console.log("No audio data to save!")
                statusText.text = "Không có dữ liệu audio để lưu"
                statusText.color = "red"
                return
            }
            
            if (audioManager.saveToFile(filePath)) {
                statusText.text = "Đã lưu: " + filePath
                statusText.color = "green"
                console.log("File saved successfully")
                
                // Kiểm tra file có tồn tại không
                var file = Qt.createQmlObject('import QtQuick 2.15; QtObject {}', parent)
                var fileInfo = Qt.createQmlObject('import QtQuick 2.15; QtObject {}', parent)
                // Thử đọc file để kiểm tra
                console.log("Checking if file exists...")
            } else {
                console.log("Failed to save file")
                statusText.color = "red"
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

            Button {
                id: testButton
                text: "Kiểm tra Audio"
                onClicked: {
                    if (audioManager.checkAudioDevice()) {
                        statusText.text = "Audio device OK"
                        statusText.color = "green"
                    } else {
                        statusText.text = "Audio device error"
                        statusText.color = "red"
                    }
                }
            }

            Button {
                id: debugButton
                text: "Debug Info"
                onClicked: {
                    var info = audioManager.getAudioBufferInfo()
                    console.log("Audio Buffer Info:", info)
                    statusText.text = "Buffer: " + audioManager.getAudioBufferSize() + " bytes"
                    statusText.color = "blue"
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
                    console.log("Save button clicked")
                    console.log("Has recorded data:", audioManager.hasRecordedData)
                    console.log("Audio buffer size:", audioManager.getAudioAsBase64().length)
                    
                    var timestamp = new Date().toISOString().replace(/[:.]/g, "-")
                    var defaultFileName = "recording-" + timestamp + ".wav"
                    console.log("Default filename:", defaultFileName)
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

            Button {
                id: quickSaveButton
                text: "Lưu nhanh"
                enabled: audioManager.hasRecordedData
                onClicked: {
                    if (!audioManager.hasRecordedData) {
                        statusText.text = "Không có dữ liệu audio"
                        statusText.color = "red"
                        return
                    }
                    
                    var timestamp = new Date().toISOString().replace(/[:.]/g, "-")
                    var fileName = "/tmp/recording-" + timestamp + ".wav"
                    console.log("Quick saving to:", fileName)
                    
                    if (audioManager.saveToFile(fileName)) {
                        statusText.text = "Đã lưu nhanh: " + fileName
                        statusText.color = "green"
                        console.log("Quick save successful")
                    } else {
                        statusText.text = "Lỗi lưu nhanh"
                        statusText.color = "red"
                        console.log("Quick save failed")
                    }
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
