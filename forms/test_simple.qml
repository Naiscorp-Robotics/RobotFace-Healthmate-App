import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

ApplicationWindow {
    id: window
    width: 400
    height: 200
    visible: true
    title: "Simple Test"

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 20
        spacing: 15

        Text {
            text: "🔍 Simple Resource Test"
            font.pixelSize: 18
            font.bold: true
            Layout.alignment: Qt.AlignHCenter
        }

        Text {
            id: statusText
            text: "Testing..."
            color: "#3498db"
            font.pixelSize: 14
            Layout.alignment: Qt.AlignHCenter
        }

        Button {
            text: "🔍 Test Image"
            Layout.fillWidth: true
            Layout.preferredHeight: 40
            
            background: Rectangle {
                color: parent.pressed ? "#2980b9" : "#3498db"
                radius: 8
            }
            
            contentItem: Text {
                text: parent.text
                color: "#ffffff"
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                font.pixelSize: 14
                font.bold: true
            }

            onClicked: {
                console.log("Testing image resource...")
                statusText.text = "🔍 Testing image..."
                
                // Test if we can load an image from resources
                testImage.source = "qrc:/assets/j97.jpg"
            }
        }

        Image {
            id: testImage
            Layout.fillWidth: true
            Layout.preferredHeight: 100
            fillMode: Image.PreserveAspectFit
            
            onStatusChanged: {
                if (status === Image.Ready) {
                    statusText.text = "✅ Image loaded successfully"
                    console.log("Image loaded successfully")
                } else if (status === Image.Error) {
                    statusText.text = "❌ Failed to load image"
                    console.log("Failed to load image")
                }
            }
        }
    }
}
