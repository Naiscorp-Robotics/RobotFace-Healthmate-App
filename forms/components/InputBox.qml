import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Item {
    signal sendMessage(string message)

    RowLayout {
        anchors.fill: parent
        spacing: 10

        TextField {
            id: messageInput
            Layout.fillWidth: true
            placeholderText: "Type your message here..."
            color: "#ffffff"
            background: Rectangle {
                color: "#3a3a3a"
                radius: 5
            }
            font.pixelSize: 14
            onAccepted: sendButton.clicked()
        }

        Button {
            id: sendButton
            text: "Send"
            Layout.preferredWidth: 80
            Layout.preferredHeight: 40
            onClicked: {
                if (messageInput.text.trim() !== "") {
                    sendMessage(messageInput.text.trim())
                    messageInput.text = ""
                }
            }
        }
    }
}
