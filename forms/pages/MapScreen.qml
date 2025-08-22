import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12

Item {
    id: root
    property StackView stackView
    property var mapBridge: null
    
    // Control mode state
    property int currentMode: 0 // 0: Manual, 1: Navigation
    property bool goalModeActive: false
    property bool hasGoal: false
    
    // Speed settings
    property real linearSpeed: 0.3
    property real angularSpeed: 0.3
    
    // Focus for keyboard events
    focus: true

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
                text: "🗺️ Bản đồ định vị & Điều khiển"
                font.pixelSize: 20
                font.bold: true
                color: "#ffffff"
                horizontalAlignment: Text.AlignHCenter
            }

            // Connection status indicator
            Rectangle {
                Layout.preferredWidth: 40
                Layout.preferredHeight: 40
                radius: 20
                color: mapBridge && mapBridge.isConnected ? "#00FF00" : "#FF0000"
                border.color: "#ffffff"
                border.width: 2

                Text {
                    anchors.centerIn: parent
                    text: mapBridge && mapBridge.isConnected ? "✓" : "✗"
                    color: "#000000"
                    font.pixelSize: 16
                    font.bold: true
                }
            }
        }
    }

    // Status bar
    Rectangle {
        id: statusBar
        anchors.bottom: parent.bottom
        anchors.left: parent.left
        anchors.right: parent.right
        height: 40
        color: "#0f3460"

        Text {
            anchors.centerIn: parent
            text: {
                if (currentMode === 0) {
                    return "Manual Control Mode - Sử dụng nút hoặc phím WASD để điều khiển"
                } else {
                    if (goalModeActive) {
                        return "Navigation Mode - Click vào bản đồ để đặt mục tiêu"
                    } else {
                        return "Navigation Mode - Click 'Set Goal Mode' để đặt mục tiêu"
                    }
                }
            }
            color: "#ffffff"
            font.pixelSize: 14
            horizontalAlignment: Text.AlignHCenter
        }
    }

    // Main content area
    Rectangle {
        id: mainContent
        anchors.top: header.bottom
        anchors.left: parent.left
        anchors.right: controlPanel.left
        anchors.bottom: statusBar.top
        anchors.margins: 10
        color: "#2a2a3e"
        border.color: "#0f3460"
        border.width: 2
        radius: 10

        // Dynamic map image from ROS2
        Image {
            id: mapImage
            anchors.fill: parent
            anchors.margins: 10
            source: mapBridge && mapBridge.mapImage ? mapBridge.mapImage : (mapBridge && mapBridge.mapImageUrl ? mapBridge.mapImageUrl : "qrc:/assets/map.png")
            fillMode: Image.PreserveAspectFit
            cache: false

            // Robot position marker
            Rectangle {
                id: robotMarker
                width: 20
                height: 20
                radius: 10
                color: "#00FF00"
                border.color: "#004400"
                border.width: 2
                visible: mapBridge && mapBridge.isConnected
                
                x: {
                    if (mapBridge && mapImage.paintedWidth > 0) {
                        var pos = mapBridge.mapToScreen(mapBridge.robotX, mapBridge.robotY)
                        return mapImage.x + (pos.x / mapImage.sourceSize.width) * mapImage.paintedWidth - width/2
                    }
                    return 0
                }
                
                y: {
                    if (mapBridge && mapImage.paintedHeight > 0) {
                        var pos = mapBridge.mapToScreen(mapBridge.robotX, mapBridge.robotY)
                        return mapImage.y + (pos.y / mapImage.sourceSize.height) * mapImage.paintedHeight - height/2
                    }
                    return 0
                }

                // Direction indicator
                Rectangle {
                    width: 2
                    height: 10
                    color: "#00FF00"
                    anchors.centerIn: parent
                    rotation: mapBridge ? mapBridge.robotTheta * 180 / Math.PI : 0
                    transformOrigin: Item.Center
                }
            }

            // Click handler for setting navigation goals
            MouseArea {
                anchors.fill: parent
                enabled: mapBridge && mapBridge.isConnected && currentMode === 1 && goalModeActive
                
                onClicked: {
                    if (mapBridge && mapImage.paintedWidth > 0 && mapImage.paintedHeight > 0) {
                        // Convert click position to map coordinates
                        var relX = (mouse.x - mapImage.x) / mapImage.paintedWidth
                        var relY = (mouse.y - mapImage.y) / mapImage.paintedHeight
                        var mapPos = mapBridge.screenToMap(
                            relX * mapImage.sourceSize.width,
                            relY * mapImage.sourceSize.height
                        )
                        mapBridge.setGoalPose(mapPos.x, mapPos.y)
                        
                        // Visual feedback for goal
                        goalMarker.x = mouse.x - goalMarker.width/2
                        goalMarker.y = mouse.y - goalMarker.height/2
                        goalMarker.visible = true
                        
                        // Update state
                        hasGoal = true
                        goalModeActive = false
                        setGoalBtn.checked = false
                        setGoalBtn.text = "Set Goal Mode"
                        cancelGoalBtn.enabled = true
                        goalStatusLabel.text = "Mục tiêu đặt tại (" + mapPos.x.toFixed(2) + ", " + mapPos.y.toFixed(2) + ")"
                    }
                }
            }

            // Goal marker
            Rectangle {
                id: goalMarker
                width: 20
                height: 20
                radius: 10
                color: "#FF0000"
                border.color: "#440000"
                border.width: 2
                visible: false
                opacity: 0.8
            }
        }

        // Loading indicator
        BusyIndicator {
            anchors.centerIn: parent
            running: mapBridge && mapBridge.isConnected && !mapImage.source
            visible: running
        }
    }

    // Control panel
    Rectangle {
        id: controlPanel
        anchors.top: header.bottom
        anchors.right: parent.right
        anchors.bottom: statusBar.top
        width: 300
        anchors.margins: 10
        color: "#2a2a3e"
        border.color: "#0f3460"
        border.width: 2
        radius: 10

        ScrollView {
            anchors.fill: parent
            anchors.margins: 10
            
            ColumnLayout {
                width: controlPanel.width - 20
                spacing: 15

                // Mode selection
                Rectangle {
                    Layout.fillWidth: true
                    height: 60
                    color: "#1a1a2e"
                    radius: 8
                    border.color: "#0f3460"
                    border.width: 1

                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 10

                        Text {
                            text: "Chế độ điều khiển"
                            font.pixelSize: 14
                            font.bold: true
                            color: "#ffffff"
                        }

                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 10

                            RadioButton {
                                id: manualModeRadio
                                text: "Manual Control"
                                checked: currentMode === 0
                                onCheckedChanged: {
                                    if (checked) {
                                        currentMode = 0
                                        goalModeActive = false
                                        setGoalBtn.checked = false
                                        setGoalBtn.text = "Set Goal Mode"
                                        // Clear any navigation goals when switching to manual mode
                                        if (mapBridge && mapBridge.isConnected) {
                                            mapBridge.cancelGoal()
                                        }
                                    }
                                }
                            }

                            RadioButton {
                                id: navigationModeRadio
                                text: "Navigation"
                                checked: currentMode === 1
                                onCheckedChanged: {
                                    if (checked) {
                                        currentMode = 1
                                        // Stop manual control when switching to navigation mode
                                        if (mapBridge && mapBridge.isConnected) {
                                            mapBridge.stopVelocity()
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                // Manual Control Group
                Rectangle {
                    id: manualGroup
                    Layout.fillWidth: true
                    height: 200
                    color: "#1a1a2e"
                    radius: 8
                    border.color: "#0f3460"
                    border.width: 1
                    visible: currentMode === 0

                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 10

                        Text {
                            text: "Điều khiển thủ công"
                            font.pixelSize: 14
                            font.bold: true
                            color: "#ffffff"
                        }

                        GridLayout {
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            columns: 3
                            rowSpacing: 10
                            columnSpacing: 10

                            // Forward button
                            Button {
                                Layout.column: 1
                                Layout.row: 0
                                Layout.preferredWidth: 60
                                Layout.preferredHeight: 60
                                text: "↑\nW"
                                background: Rectangle {
                                    color: parent.pressed ? "#27ae60" : "#2ecc71"
                                    radius: 8
                                }
                                contentItem: Text {
                                    text: parent.text
                                    color: "#ffffff"
                                    horizontalAlignment: Text.AlignHCenter
                                    verticalAlignment: Text.AlignVCenter
                                    font.pixelSize: 12
                                    font.bold: true
                                }
                                onPressed: {
                                    if (mapBridge && mapBridge.isConnected) {
                                        mapBridge.sendVelocity(linearSpeed, 0)
                                    }
                                }
                                onReleased: {
                                    if (mapBridge && mapBridge.isConnected) {
                                        mapBridge.sendVelocity(0, 0)
                                    }
                                }
                            }

                            // Left button
                            Button {
                                Layout.column: 0
                                Layout.row: 1
                                Layout.preferredWidth: 60
                                Layout.preferredHeight: 60
                                text: "←\nA"
                                background: Rectangle {
                                    color: parent.pressed ? "#e74c3c" : "#e67e22"
                                    radius: 8
                                }
                                contentItem: Text {
                                    text: parent.text
                                    color: "#ffffff"
                                    horizontalAlignment: Text.AlignHCenter
                                    verticalAlignment: Text.AlignVCenter
                                    font.pixelSize: 12
                                    font.bold: true
                                }
                                onPressed: {
                                    if (mapBridge && mapBridge.isConnected) {
                                        mapBridge.sendVelocity(0, angularSpeed)
                                    }
                                }
                                onReleased: {
                                    if (mapBridge && mapBridge.isConnected) {
                                        mapBridge.sendVelocity(0, 0)
                                    }
                                }
                            }

                            // Stop button
                            Button {
                                Layout.column: 1
                                Layout.row: 1
                                Layout.preferredWidth: 60
                                Layout.preferredHeight: 60
                                text: "⬜\nSpace"
                                background: Rectangle {
                                    color: parent.pressed ? "#c0392b" : "#e74c3c"
                                    radius: 8
                                }
                                contentItem: Text {
                                    text: parent.text
                                    color: "#ffffff"
                                    horizontalAlignment: Text.AlignHCenter
                                    verticalAlignment: Text.AlignVCenter
                                    font.pixelSize: 10
                                    font.bold: true
                                }
                                onClicked: {
                                    if (mapBridge && mapBridge.isConnected) {
                                        mapBridge.sendVelocity(0, 0)
                                    }
                                }
                            }

                            // Right button
                            Button {
                                Layout.column: 2
                                Layout.row: 1
                                Layout.preferredWidth: 60
                                Layout.preferredHeight: 60
                                text: "→\nD"
                                background: Rectangle {
                                    color: parent.pressed ? "#e74c3c" : "#e67e22"
                                    radius: 8
                                }
                                contentItem: Text {
                                    text: parent.text
                                    color: "#ffffff"
                                    horizontalAlignment: Text.AlignHCenter
                                    verticalAlignment: Text.AlignVCenter
                                    font.pixelSize: 12
                                    font.bold: true
                                }
                                onPressed: {
                                    if (mapBridge && mapBridge.isConnected) {
                                        mapBridge.sendVelocity(0, -angularSpeed)
                                    }
                                }
                                onReleased: {
                                    if (mapBridge && mapBridge.isConnected) {
                                        mapBridge.sendVelocity(0, 0)
                                    }
                                }
                            }

                            // Backward button
                            Button {
                                Layout.column: 1
                                Layout.row: 2
                                Layout.preferredWidth: 60
                                Layout.preferredHeight: 60
                                text: "↓\nS"
                                background: Rectangle {
                                    color: parent.pressed ? "#27ae60" : "#2ecc71"
                                    radius: 8
                                }
                                contentItem: Text {
                                    text: parent.text
                                    color: "#ffffff"
                                    horizontalAlignment: Text.AlignHCenter
                                    verticalAlignment: Text.AlignVCenter
                                    font.pixelSize: 12
                                    font.bold: true
                                }
                                onPressed: {
                                    if (mapBridge && mapBridge.isConnected) {
                                        mapBridge.sendVelocity(-linearSpeed, 0)
                                    }
                                }
                                onReleased: {
                                    if (mapBridge && mapBridge.isConnected) {
                                        mapBridge.sendVelocity(0, 0)
                                    }
                                }
                            }
                        }
                    }
                }

                // Speed Control Group
                Rectangle {
                    Layout.fillWidth: true
                    height: 120
                    color: "#1a1a2e"
                    radius: 8
                    border.color: "#0f3460"
                    border.width: 1

                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 10

                        Text {
                            text: "Điều chỉnh tốc độ"
                            font.pixelSize: 14
                            font.bold: true
                            color: "#ffffff"
                        }

                        // Linear speed
                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 10

                            Text {
                                text: "Tốc độ tuyến tính:"
                                color: "#ffffff"
                                font.pixelSize: 12
                            }

                            Slider {
                                id: linearSpeedSlider
                                Layout.fillWidth: true
                                from: 0
                                to: 100
                                value: 30
                                onValueChanged: {
                                    linearSpeed = value / 100.0
                                    linearSpeedLabel.text = linearSpeed.toFixed(2) + " m/s"
                                }
                            }

                            Text {
                                id: linearSpeedLabel
                                text: "0.30 m/s"
                                color: "#ffffff"
                                font.pixelSize: 10
                                Layout.preferredWidth: 50
                            }
                        }

                        // Angular speed
                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 10

                            Text {
                                text: "Tốc độ góc:"
                                color: "#ffffff"
                                font.pixelSize: 12
                            }

                            Slider {
                                id: angularSpeedSlider
                                Layout.fillWidth: true
                                from: 0
                                to: 100
                                value: 30
                                onValueChanged: {
                                    angularSpeed = value / 100.0
                                    angularSpeedLabel.text = angularSpeed.toFixed(2) + " rad/s"
                                }
                            }

                            Text {
                                id: angularSpeedLabel
                                text: "0.30 rad/s"
                                color: "#ffffff"
                                font.pixelSize: 10
                                Layout.preferredWidth: 50
                            }
                        }
                    }
                }

                // Navigation Control Group
                Rectangle {
                    id: navigationGroup
                    Layout.fillWidth: true
                    height: 150
                    color: "#1a1a2e"
                    radius: 8
                    border.color: "#0f3460"
                    border.width: 1
                    visible: currentMode === 1

                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 10

                        Text {
                            text: "Điều khiển điều hướng"
                            font.pixelSize: 14
                            font.bold: true
                            color: "#ffffff"
                        }

                        Text {
                            text: "Click vào bản đồ để đặt mục tiêu"
                            color: "#cccccc"
                            font.pixelSize: 11
                            wrapMode: Text.WordWrap
                        }

                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 10

                            Button {
                                id: setGoalBtn
                                Layout.fillWidth: true
                                text: "Set Goal Mode"
                                checkable: true
                                background: Rectangle {
                                    color: parent.checked ? "#44ff44" : (parent.pressed ? "#555555" : "#333333")
                                    radius: 8
                                }
                                contentItem: Text {
                                    text: parent.text
                                    color: parent.checked ? "#000000" : "#ffffff"
                                    horizontalAlignment: Text.AlignHCenter
                                    verticalAlignment: Text.AlignVCenter
                                    font.pixelSize: 12
                                    font.bold: parent.checked
                                }
                                onCheckedChanged: {
                                    goalModeActive = checked
                                    if (checked) {
                                        text = "Click vào bản đồ"
                                    } else {
                                        text = "Set Goal Mode"
                                    }
                                }
                            }

                            Button {
                                id: cancelGoalBtn
                                Layout.fillWidth: true
                                text: "Hủy mục tiêu"
                                enabled: hasGoal
                                background: Rectangle {
                                    color: parent.enabled ? (parent.pressed ? "#c0392b" : "#e74c3c") : "#555555"
                                    radius: 8
                                }
                                contentItem: Text {
                                    text: parent.text
                                    color: "#ffffff"
                                    horizontalAlignment: Text.AlignHCenter
                                    verticalAlignment: Text.AlignVCenter
                                    font.pixelSize: 12
                                }
                                onClicked: {
                                    if (mapBridge && mapBridge.isConnected) {
                                        mapBridge.cancelGoal()
                                    }
                                    hasGoal = false
                                    goalMarker.visible = false
                                    cancelGoalBtn.enabled = false
                                    goalStatusLabel.text = "Không có mục tiêu"
                                }
                            }
                        }

                        Text {
                            id: goalStatusLabel
                            text: "Không có mục tiêu"
                            color: "#cccccc"
                            font.pixelSize: 11
                            wrapMode: Text.WordWrap
                        }
                    }
                }

                // Connection Control
                Rectangle {
                    Layout.fillWidth: true
                    height: 80
                    color: "#1a1a2e"
                    radius: 8
                    border.color: "#0f3460"
                    border.width: 1

                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 10

                        Text {
                            text: "Kết nối"
                            font.pixelSize: 14
                            font.bold: true
                            color: "#ffffff"
                        }

                        Button {
                            Layout.fillWidth: true
                            text: mapBridge && mapBridge.isConnected ? "Ngắt kết nối ROS2" : "Kết nối ROS2"
                            
                            background: Rectangle {
                                color: parent.pressed ? "#555555" : "#333333"
                                radius: 8
                            }
                            
                            contentItem: Text {
                                text: parent.text
                                color: "#ffffff"
                                horizontalAlignment: Text.AlignHCenter
                                verticalAlignment: Text.AlignVCenter
                                font.pixelSize: 12
                            }
                            
                            onClicked: {
                                if (mapBridge) {
                                    if (mapBridge.isConnected) {
                                        mapBridge.disconnectFromRos2()
                                    } else {
                                        mapBridge.connectToRos2()
                                    }
                                }
                            }
                        }
                    }
                }

                // Keyboard shortcuts info
                Rectangle {
                    Layout.fillWidth: true
                    height: 120
                    color: "#1a1a2e"
                    radius: 8
                    border.color: "#0f3460"
                    border.width: 1

                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 10

                        Text {
                            text: "Phím tắt"
                            font.pixelSize: 14
                            font.bold: true
                            color: "#ffffff"
                        }

                        Text {
                            text: "W/↑ - Tiến lên\nS/↓ - Lùi lại\nA/← - Rẽ trái\nD/→ - Rẽ phải\nSpace - Dừng\nClick - Đặt mục tiêu (Navigation mode)"
                            color: "#cccccc"
                            font.pixelSize: 10
                            lineHeight: 1.2
                        }
                    }
                }
            }
        }
    }

    // Keyboard event handling - Only active in Manual Control mode
    Keys.onPressed: {
        if (event.isAutoRepeat) return
        
        if (currentMode === 0 && mapBridge && mapBridge.isConnected) {
            switch(event.key) {
                case Qt.Key_W:
                case Qt.Key_Up:
                    mapBridge.sendVelocity(linearSpeed, 0)
                    break
                case Qt.Key_S:
                case Qt.Key_Down:
                    mapBridge.sendVelocity(-linearSpeed, 0)
                    break
                case Qt.Key_A:
                case Qt.Key_Left:
                    mapBridge.sendVelocity(0, angularSpeed)
                    break
                case Qt.Key_D:
                case Qt.Key_Right:
                    mapBridge.sendVelocity(0, -angularSpeed)
                    break
                case Qt.Key_Space:
                    mapBridge.sendVelocity(0, 0)
                    break
            }
        }
    }

    Keys.onReleased: {
        if (event.isAutoRepeat) return
        
        if (currentMode === 0 && mapBridge && mapBridge.isConnected) {
            switch(event.key) {
                case Qt.Key_W:
                case Qt.Key_Up:
                case Qt.Key_S:
                case Qt.Key_Down:
                case Qt.Key_A:
                case Qt.Key_Left:
                case Qt.Key_D:
                case Qt.Key_Right:
                    mapBridge.sendVelocity(0, 0)
                    break
            }
        }
    }
}
