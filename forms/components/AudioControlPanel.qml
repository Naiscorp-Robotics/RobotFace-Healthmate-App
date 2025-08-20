// // components/AudioControlPanel.qml
// import QtQuick
// import QtQuick.Controls
// import QtQuick.Layouts

// Rectangle {
//     id: audioPanel
//     width: 200
//     height: 120
//     color: "#1a1a1a"
//     radius: 10
//     border.color: "#333333"
//     border.width: 1
//     opacity: 0.9

//     property alias audioManager: audioManagerRef
//     property bool isRecording: false

//     AudioManager {
//         id: audioManagerRef
//     }

//     ColumnLayout {
//         anchors.fill: parent
//         anchors.margins: 10
//         spacing: 5

//         Text {
//             text: "Audio Controls"
//             color: "white"
//             font.bold: true
//             Layout.alignment: Qt.AlignHCenter
//         }

//         Button {
//             id: recordButton
//             text: audioManager.isCapturing ? "Stop Recording" : "Start Recording"
//             Layout.fillWidth: true
//             onClicked: {
//                 if (audioManager.isCapturing) {
//                     audioManager.stopCapture()
//                 } else {
//                     audioManager.startCapture()
//                 }
//             }
//         }

//         Button {
//             id: playButton
//             text: "Play Audio"
//             enabled: audioManager.hasRecordedData
//             Layout.fillWidth: true
//             onClicked: {
//                 audioManager.playAudio()
//             }
//         }

//         Button {
//             id: saveButton
//             text: "Save Recording"
//             enabled: audioManager.hasRecordedData
//             Layout.fillWidth: true
//             onClicked: {
//                 // Generate filename with timestamp
//                 var timestamp = new Date().toISOString().replace(/[:.]/g, "-")
//                 var filename = "recording-" + timestamp + ".wav"
//                 audioManager.saveToFile(filename)
//             }
//         }
//     }
// }
