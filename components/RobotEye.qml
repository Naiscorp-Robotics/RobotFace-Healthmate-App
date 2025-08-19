import QtQuick

Rectangle {
    width: 120
    height: 120
    radius: 60
    color: "#00ff00"
    border.width: 3
    border.color: "#ffffff"

    SequentialAnimation on scale {
        running: true
        loops: Animation.Infinite
        NumberAnimation { to: 1.1; duration: 1000 }
        NumberAnimation { to: 1.0; duration: 1000 }
    }
}
