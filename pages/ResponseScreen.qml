import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Item {
    property StackView stackView
    property string response: ""

    Rectangle {
        anchors.fill: parent
        color: "#1a1a1a"

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 20
            spacing: 20

            RowLayout {
                Layout.fillWidth: true
                Button {
                    text: "← Back"
                    Layout.preferredWidth: 80
                    Layout.preferredHeight: 40
                    background: Rectangle {
                        color: parent.pressed ? "#555555" : "#3a3a3a"
                        radius: 5
                    }
                    onClicked: {
                        if (stackView) {
                            stackView.pop()
                        }
                    }
                }
                Text {
                    text: "Robot Response"
                    font.pixelSize: 24
                    font.bold: true
                    color: "#ffffff"
                    Layout.fillWidth: true
                    horizontalAlignment: Text.AlignHCenter
                }
            }

            ScrollView {
                Layout.fillWidth: true
                Layout.fillHeight: true
                
                TextArea {
                    readOnly: true
                    wrapMode: TextArea.Wrap
                    color: "#ffffff"
                    font.pixelSize: 14
                    text: response
                    background: Rectangle {
                        color: "#2a2a2a"
                        radius: 5
                    }
                }
            }
        }
    }
}
