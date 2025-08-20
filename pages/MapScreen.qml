import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Item {
    id: root
    property StackView stackView

    // Background
    Rectangle {
        anchors.fill: parent
        color: "#1a1a2e"
    }

    // Header
    Rectangle {
        id: header
        anchors.top: parent.top
        anchors.left: parent.left
        anchors.right: parent.right
        height: 60
        color: "#0f3460"

        RowLayout {
            anchors.fill: parent
            anchors.margins: 15

            // Back button
            Button {
                Layout.preferredWidth: 40
                Layout.preferredHeight: 40
                text: "←"
                background: Rectangle {
                    color: parent.pressed ? "#555555" : "#333333"
                    radius: 8
                }
                contentItem: Text {
                    text: parent.text
                    color: "#ffffff"
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                    font.pixelSize: 18
                    font.bold: true
                }

                onClicked: {
                    if (root.stackView) {
                        root.stackView.pop()
                    }
                }
            }

            // Title
            Text {
                Layout.fillWidth: true
                text: "🗺️ Bản đồ định vị"
                font.pixelSize: 20
                font.bold: true
                color: "#ffffff"
                horizontalAlignment: Text.AlignHCenter
            }

            // Spacer
            Item {
                Layout.preferredWidth: 40
            }
        }
    }

    // Map image
    Image {
        anchors.top: header.bottom
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        anchors.margins: 20
        source: "qrc:/assets/map.png"
        fillMode: Image.PreserveAspectFit
    }
}
