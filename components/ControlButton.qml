import QtQuick
import QtQuick.Controls

Button {
    property color colorNormal: "#0077cc"
    property color colorPressed: "#0055aa"

    background: Rectangle {
        color: control.pressed ? control.colorPressed : control.colorNormal
        radius: 5
    }

    contentItem: Text {
        text: control.text
        color: "#ffffff"
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
    }
}
