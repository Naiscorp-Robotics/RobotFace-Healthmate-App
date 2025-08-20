import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Item {
    id: root
    property StackView stackView

    Rectangle {
        anchors.fill: parent
        color: "#1a1a1a"

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 10
            spacing: 10

            // Header với nút back
            RowLayout {
                Layout.fillWidth: true
                Layout.preferredHeight: 40

                Button {
                    text: "← Back"
                    Layout.preferredWidth: 80
                    Layout.preferredHeight: 30
                    background: Rectangle {
                        color: parent.pressed ? "#555555" : "#333333"
                        radius: 6
                    }
                    contentItem: Text {
                        text: parent.text
                        color: "#ffffff"
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                    onClicked: {
                        if (root.stackView) {
                            root.stackView.pop()
                        }
                    }
                }

                Text {
                    text: "📷 Image Viewer"
                    font.pixelSize: 18
                    font.bold: true
                    color: "#ffffff"
                    Layout.fillWidth: true
                    horizontalAlignment: Text.AlignHCenter
                }

                // Spacer để cân bằng layout
                Item {
                    Layout.preferredWidth: 80
                }
            }

            // Image container
            Rectangle {
                Layout.fillWidth: true
                Layout.fillHeight: true
                color: "#000000"
                radius: 8
                border.width: 2
                border.color: "#ffffff"

                Image {
                    id: imageView
                    anchors.fill: parent
                    anchors.margins: 10
                    source: "file:///F:/Study-Work/Naiscorp/QT/RobotFace-Healthmate-App/assets/j97.jpg"
                    fillMode: Image.PreserveAspectFit
                    smooth: true
                    antialiasing: true

                    // Hiển thị thông báo nếu ảnh không tải được
                    Text {
                        anchors.centerIn: parent
                        text: "Không thể tải ảnh\nVui lòng kiểm tra đường dẫn file"
                        color: "#ffffff"
                        font.pixelSize: 16
                        horizontalAlignment: Text.AlignHCenter
                        visible: imageView.status === Image.Error
                    }

                    // Loading indicator
                    BusyIndicator {
                        anchors.centerIn: parent
                        running: imageView.status === Image.Loading
                        visible: imageView.status === Image.Loading
                    }
                }

                // Debug info (ẩn khi ảnh hoạt động)
                Text {
                    anchors.top: parent.top
                    anchors.left: parent.left
                    anchors.margins: 10
                    text: "Image Status: " + imageView.status + 
                          "\nSource: " + imageView.source +
                          "\nWidth: " + imageView.sourceSize.width +
                          "\nHeight: " + imageView.sourceSize.height
                    color: "#ffffff"
                    font.pixelSize: 10
                    visible: false // Ẩn debug text
                }
            }

            // Image info
            Text {
                text: "File: j97.jpg"
                font.pixelSize: 12
                color: "#888888"
                Layout.alignment: Qt.AlignHCenter
            }
        }
    }
}
