import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import "./pages"


Window {
    width: 800
    height: 600
    visible: true
    title: qsTr("Robot Face Interface")
    color: "#1a1a1a"

    // Global states
    property string currentResponse: ""
    property string currentStatus: "Ready to chat with robot..."

    StackView {
        id: stackView
        anchors.fill: parent
        initialItem: RobotFaceScreen {
            stackView: stackView
            currentResponse: currentResponse
            currentStatus: currentStatus
            onResponseChanged: function(response) {
                currentResponse = response
            }
            onStatusChanged: function(status) {
                currentStatus = status
            }
        }
    }
}
