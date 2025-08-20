import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Item {
    id: root
    property StackView stackView
    property int currentStepIndex: 0
    
    // Dummy data for health care steps
    property var careSteps: [
        {
            title: "Bước 1: Kiểm tra sức khỏe tổng quát",
            description: "Bắt đầu bằng việc kiểm tra các chỉ số cơ bản như huyết áp, nhịp tim, nhiệt độ cơ thể. Đảm bảo bạn đang trong trạng thái ổn định trước khi thực hiện các bước tiếp theo.",
            image: "qrc:/assets/j97.jpg"
        },
        {
            title: "Bước 2: Thực hiện bài tập thở",
            description: "Hít thở sâu và đều đặn trong 5-10 phút. Điều này giúp giảm căng thẳng, cải thiện tuần hoàn máu và tăng cường oxy cho não bộ.",
            image: "qrc:/assets/j97.jpg"
        },
        {
            title: "Bước 3: Uống nước đầy đủ",
            description: "Uống ít nhất 8 ly nước mỗi ngày. Nước giúp thanh lọc cơ thể, duy trì nhiệt độ cơ thể và hỗ trợ các chức năng trao đổi chất.",
            image: "qrc:/assets/j97.jpg"
        },
        {
            title: "Bước 4: Thực hiện bài tập thể dục nhẹ",
            description: "Thực hiện các động tác kéo giãn cơ bản trong 15-20 phút. Điều này giúp tăng cường sự linh hoạt và giảm đau nhức cơ bắp.",
            image: "qrc:/assets/j97.jpg"
        },
        {
            title: "Bước 5: Nghỉ ngơi và thư giãn",
            description: "Dành thời gian nghỉ ngơi, có thể nghe nhạc nhẹ nhàng hoặc thiền định. Điều này giúp tâm trí được thư giãn và chuẩn bị cho giấc ngủ ngon.",
            image: "qrc:/assets/j97.jpg"
        }
    ]

    Rectangle {
        anchors.fill: parent
        color: "#f0f0f0"
    }

    // Header
    Rectangle {
        id: header
        anchors.top: parent.top
        anchors.left: parent.left
        anchors.right: parent.right
        height: 60
        color: "#2c3e50"

        Text {
            anchors.centerIn: parent
            text: "📋 Hướng dẫn chăm sóc sức khỏe"
            font.pixelSize: 18
            font.bold: true
            color: "#ffffff"
        }

        // Step indicator
        Text {
            anchors.right: parent.right
            anchors.rightMargin: 20
            anchors.verticalCenter: parent.verticalCenter
            text: "Bước " + (currentStepIndex + 1) + "/" + careSteps.length
            font.pixelSize: 14
            color: "#ecf0f1"
        }
    }

    // Main content area
    RowLayout {
        anchors.top: header.bottom
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.bottom: bottomControls.top
        anchors.margins: 20
        spacing: 20

        // Left side - Image (2/3 width)
        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredWidth: parent.width * 0.6
            color: "#ffffff"
            radius: 10
            border.color: "#bdc3c7"
            border.width: 1

            Image {
                anchors.fill: parent
                anchors.margins: 10
                source: careSteps[currentStepIndex].image
                fillMode: Image.PreserveAspectFit
                smooth: true
                antialiasing: true
            }
        }

        // Right side - Description (1/3 width)
        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredWidth: parent.width * 0.4
            color: "#ffffff"
            radius: 10
            border.color: "#bdc3c7"
            border.width: 1

            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 20
                spacing: 15

                // Title
                Text {
                    text: careSteps[currentStepIndex].title
                    font.pixelSize: 18
                    font.bold: true
                    color: "#2c3e50"
                    wrapMode: Text.WordWrap
                    Layout.fillWidth: true
                }

                // Description
                Text {
                    text: careSteps[currentStepIndex].description
                    font.pixelSize: 14
                    color: "#34495e"
                    wrapMode: Text.WordWrap
                    lineHeight: 1.4
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                }
            }
        }
    }

    // Bottom controls
    Rectangle {
        id: bottomControls
        anchors.bottom: parent.bottom
        anchors.left: parent.left
        anchors.right: parent.right
        height: 80
        color: "#ecf0f1"
        border.color: "#bdc3c7"
        border.width: 1

        RowLayout {
            anchors.fill: parent
            anchors.margins: 20
            spacing: 15

            // Back button
            Button {
                Layout.fillWidth: true
                Layout.preferredHeight: 40
                text: "⬅️ Quay lại"
                enabled: currentStepIndex > 0
                
                background: Rectangle {
                    color: parent.enabled ? (parent.pressed ? "#95a5a6" : "#7f8c8d") : "#bdc3c7"
                    radius: 8
                }
                
                contentItem: Text {
                    text: parent.text
                    color: parent.enabled ? "#ffffff" : "#7f8c8d"
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                    font.pixelSize: 14
                    font.bold: true
                }

                onClicked: {
                    if (currentStepIndex > 0) {
                        currentStepIndex--
                    }
                }
            }

            // Next button
            Button {
                Layout.fillWidth: true
                Layout.preferredHeight: 40
                text: currentStepIndex < careSteps.length - 1 ? "Tiếp theo ➡️" : "Hoàn thành ✅"
                
                background: Rectangle {
                    color: parent.pressed ? "#27ae60" : "#2ecc71"
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
                    if (currentStepIndex < careSteps.length - 1) {
                        currentStepIndex++
                    } else {
                        // Completed all steps
                        if (stackView) {
                            stackView.pop()
                        }
                    }
                }
            }

            // Home button
            Button {
                Layout.fillWidth: true
                Layout.preferredHeight: 40
                text: "🏠 Trang chủ"
                
                background: Rectangle {
                    color: parent.pressed ? "#e67e22" : "#f39c12"
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
                    if (stackView) {
                        // Go back to robot face screen
                        stackView.pop(stackView.get(0))
                    }
                }
            }
        }
    }

    // Progress bar
    Rectangle {
        anchors.top: header.bottom
        anchors.left: parent.left
        anchors.right: parent.right
        height: 4
        color: "#ecf0f1"

        Rectangle {
            width: parent.width * ((currentStepIndex + 1) / careSteps.length)
            height: parent.height
            color: "#3498db"
            Behavior on width {
                NumberAnimation { duration: 300 }
            }
        }
    }
}
