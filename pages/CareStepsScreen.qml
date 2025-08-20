import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Item {
    id: root
    property StackView stackView
    property int currentStepIndex: 0
    
    // Dynamic care steps that can be updated from TSS Socket
    property var careSteps: [
        {
            title: "Bước 1: Kiểm tra sức khỏe tổng quát",
            description: "Bắt đầu bằng việc kiểm tra các chỉ số cơ bản như huyết áp, nhịp tim, nhiệt độ cơ thể. Đảm bảo bạn đang trong trạng thái ổn định trước khi thực hiện các bước tiếp theo.",
            image: "file:///F:/Study-Work/Naiscorp/QT/RobotFace-Healthmate-App/assets/j97.jpg"
        },
        {
            title: "Bước 2: Thực hiện bài tập thở",
            description: "Hít thở sâu và đều đặn trong 5-10 phút. Điều này giúp giảm căng thẳng, cải thiện tuần hoàn máu và tăng cường oxy cho não bộ.",
            image: "file:///F:/Study-Work/Naiscorp/QT/RobotFace-Healthmate-App/assets/j97.jpg"
        },
        {
            title: "Bước 3: Uống nước đầy đủ",
            description: "Uống ít nhất 8 ly nước mỗi ngày. Nước giúp thanh lọc cơ thể, duy trì nhiệt độ cơ thể và hỗ trợ các chức năng trao đổi chất.",
            image: "file:///F:/Study-Work/Naiscorp/QT/RobotFace-Healthmate-App/assets/j97.jpg"
        },
        {
            title: "Bước 4: Thực hiện bài tập thể dục nhẹ",
            description: "Thực hiện các động tác kéo giãn cơ bản trong 15-20 phút. Điều này giúp tăng cường sự linh hoạt và giảm đau nhức cơ bắp.",
            image: "file:///F:/Study-Work/Naiscorp/QT/RobotFace-Healthmate-App/assets/j97.jpg"
        },
        {
            title: "Bước 5: Nghỉ ngơi và thư giãn",
            description: "Dành thời gian nghỉ ngơi, có thể nghe nhạc nhẹ nhàng hoặc thiền định. Điều này giúp tâm trí được thư giãn và chuẩn bị cho giấc ngủ ngon.",
            image: "file:///F:/Study-Work/Naiscorp/QT/RobotFace-Healthmate-App/assets/j97.jpg"
        }
    ]

    // Queue to store received TSS data
    property var tssDataQueue: []
    property bool hasQueuedData: false

    // Function to add TSS data to queue
    function addToTSSQueue(stepNumber, stepDescription, imageBase64) {
        console.log("CareStepsScreen: Adding to TSS queue - Step", stepNumber, ":", stepDescription)
        
        let stepData = {
            stepNumber: stepNumber,
            stepDescription: stepDescription,
            imageBase64: imageBase64,
            timestamp: Date.now()
        }
        
        tssDataQueue.push(stepData)
        hasQueuedData = tssDataQueue.length > 0
        
        console.log("CareStepsScreen: Queue now has", tssDataQueue.length, "items")
        console.log("CareStepsScreen: Queue contents:", JSON.stringify(tssDataQueue))
    }

    // Function to apply next queued step data
    function applyNextQueuedStep() {
        if (tssDataQueue.length > 0) {
            let stepData = tssDataQueue.shift() // Remove and get first item
            console.log("CareStepsScreen: Applying queued step data:", stepData)
            
            // Convert step number to 0-based index
            let stepIndex = stepData.stepNumber - 1
            
            // Ensure the step exists in the array
            if (stepIndex >= 0 && stepIndex < careSteps.length) {
                // Update the step data
                careSteps[stepIndex].title = "Bước " + stepData.stepNumber + ": " + stepData.stepDescription
                careSteps[stepIndex].description = stepData.stepDescription
                
                // If we have base64 image data, convert it to a data URL
                if (stepData.imageBase64 && stepData.imageBase64.length > 0) {
                    careSteps[stepIndex].image = "data:image/png;base64," + stepData.imageBase64
                    console.log("CareStepsScreen: Updated image for step", stepData.stepNumber)
                }
                
                // Force UI update by triggering property change
                careSteps = careSteps
                
                console.log("CareStepsScreen: Applied queued step", stepData.stepNumber, "successfully")
            } else {
                console.log("CareStepsScreen: Queued step index out of range:", stepIndex)
            }
            
            // Update queue status
            hasQueuedData = tssDataQueue.length > 0
            console.log("CareStepsScreen: Remaining queue items:", tssDataQueue.length)
        } else {
            console.log("CareStepsScreen: No queued data to apply")
        }
    }

    // Function to navigate to a specific step
    function navigateToStep(stepNumber) {
        let stepIndex = stepNumber - 1
        if (stepIndex >= 0 && stepIndex < careSteps.length) {
            currentStepIndex = stepIndex
            console.log("CareStepsScreen: Navigated to step", stepNumber)
        }
    }

    // Function to clear the queue
    function clearTSSQueue() {
        tssDataQueue = []
        hasQueuedData = false
        console.log("CareStepsScreen: TSS queue cleared")
    }

    // Function to directly apply TSS data to a specific step
    function applyTSSDataDirectly(stepNumber, stepDescription, imageBase64) {
        console.log("CareStepsScreen: Applying TSS data directly to step", stepNumber)
        
        // Convert step number to 0-based index
        let stepIndex = stepNumber - 1
        
        // Ensure the step exists in the array
        if (stepIndex >= 0 && stepIndex < careSteps.length) {
            // Update the step data
            careSteps[stepIndex].title = "Bước " + stepNumber + ": " + stepDescription
            careSteps[stepIndex].description = stepDescription
            
            // If we have base64 image data, convert it to a data URL
            if (imageBase64 && imageBase64.length > 0) {
                careSteps[stepIndex].image = "data:image/png;base64," + imageBase64
                console.log("CareStepsScreen: Updated image for step", stepNumber)
            }
            
            // Force UI update by triggering property change
            careSteps = careSteps
            
            console.log("CareStepsScreen: Applied TSS data directly to step", stepNumber, "successfully")
        } else {
            console.log("CareStepsScreen: Step index out of range:", stepIndex)
        }
    }

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
        height: 120
        color: "#ecf0f1"
        border.color: "#bdc3c7"
        border.width: 1

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 20
            spacing: 10

            // Top row - Navigation buttons
            RowLayout {
                Layout.fillWidth: true
                spacing: 10

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
                        // First, apply any queued TSS data
                        if (hasQueuedData) {
                            applyNextQueuedStep()
                        }
                        
                        // Then navigate to next step
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

            // Bottom row - TSS buttons
            RowLayout {
                Layout.fillWidth: true
                spacing: 10

                // TSS Request button
                Button {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 40
                    text: hasQueuedData ? "📡 Request TSS Step (" + tssDataQueue.length + ")" : "📡 Request TSS Step"
                    
                    background: Rectangle {
                        color: parent.pressed ? "#9b59b6" : (hasQueuedData ? "#e74c3c" : "#8e44ad")
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
                        if (tssSocketBridge && tssSocketBridge.isConnected) {
                            // Request the current step + 1 (or step 1 if at the end)
                            let nextStep = (currentStepIndex + 1) % careSteps.length + 1
                            tssSocketBridge.sendStepRequest(nextStep, "Step " + nextStep + " from CareStepsScreen")
                            console.log("CareStepsScreen: Requested TSS step", nextStep)
                        } else {
                            console.log("CareStepsScreen: TSS Socket not connected")
                        }
                    }
                }

                // Clear Queue button
                Button {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 40
                    text: "🗑️ Clear Queue"
                    enabled: hasQueuedData
                    
                    background: Rectangle {
                        color: parent.enabled ? (parent.pressed ? "#c0392b" : "#e74c3c") : "#bdc3c7"
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
                        if (hasQueuedData) {
                            clearTSSQueue()
                        }
                    }
                }

                // Test Navigation button
                Button {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 40
                    text: "🧪 Test Step 3"
                    
                    background: Rectangle {
                        color: parent.pressed ? "#16a085" : "#1abc9c"
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
                        // Simulate receiving TSS data for step 3
                        applyTSSDataDirectly(3, "Test Step 3 - Exercise Routine", "")
                        currentStepIndex = 2 // Navigate to step 3 (index 2)
                        console.log("CareStepsScreen: Test navigation to step 3")
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

    // TSS Data Received Indicator
    Rectangle {
        id: tssDataIndicator
        anchors.top: parent.top
        anchors.right: parent.right
        anchors.margins: 10
        anchors.topMargin: 45
        width: 120
        height: 25
        color: "#2196F3"
        radius: 12
        opacity: 0
        visible: false

        Text {
            anchors.centerIn: parent
            text: "📡 TSS Data Received"
            color: "white"
            font.pixelSize: 9
            font.bold: true
        }

        // Animation for showing the indicator
        SequentialAnimation on opacity {
            id: showAnimation
            running: false
            NumberAnimation { to: 1.0; duration: 300 }
            PauseAnimation { duration: 2000 }
            NumberAnimation { to: 0.0; duration: 300 }
            onFinished: {
                tssDataIndicator.visible = false
            }
        }
    }

    // TSS Queue Status Indicator
    Rectangle {
        anchors.top: parent.top
        anchors.right: parent.right
        anchors.margins: 10
        anchors.topMargin: 75
        width: 120
        height: 25
        color: hasQueuedData ? "#FF9800" : "#9E9E9E"
        radius: 12
        opacity: 0.8

        Text {
            anchors.centerIn: parent
            text: hasQueuedData ? "📋 Queue: " + tssDataQueue.length : "📋 Queue: Empty"
            color: "white"
            font.pixelSize: 9
            font.bold: true
        }
    }

    // Current Step with TSS Data Indicator
    Rectangle {
        anchors.top: parent.top
        anchors.right: parent.right
        anchors.margins: 10
        anchors.topMargin: 105
        width: 120
        height: 25
        color: "#4CAF50"
        radius: 12
        opacity: 0.8

        Text {
            anchors.centerIn: parent
            text: "📍 Step " + (currentStepIndex + 1) + " (TSS)"
            color: "white"
            font.pixelSize: 9
            font.bold: true
        }
    }

    // Connect to TSS Socket signals
    Connections {
        target: tssSocketBridge

        function onStepDataChanged() {
            console.log("CareStepsScreen: Received step data change signal")
            console.log("Step Number:", tssSocketBridge.currentStepNumber)
            console.log("Step Description:", tssSocketBridge.currentStepDescription)
            console.log("Has Image:", tssSocketBridge.currentImageBase64.length > 0)
            
            // Add the received data to the queue for later use
            addToTSSQueue(
                tssSocketBridge.currentStepNumber,
                tssSocketBridge.currentStepDescription,
                tssSocketBridge.currentImageBase64
            )
            
            // Show the TSS data received indicator
            tssDataIndicator.visible = true
            showAnimation.start()
            
            // Apply the data immediately to display it right away
            applyTSSDataDirectly(
                tssSocketBridge.currentStepNumber,
                tssSocketBridge.currentStepDescription,
                tssSocketBridge.currentImageBase64
            )
            
            // Navigate to the received step
            let receivedStepIndex = tssSocketBridge.currentStepNumber - 1
            if (receivedStepIndex >= 0 && receivedStepIndex < careSteps.length) {
                console.log("CareStepsScreen: Navigating to received step", tssSocketBridge.currentStepNumber, "(index", receivedStepIndex, ")")
                currentStepIndex = receivedStepIndex
            } else {
                console.log("CareStepsScreen: Received step number out of range:", tssSocketBridge.currentStepNumber)
            }
        }

        function onLogMessage(message) {
            console.log("CareStepsScreen TSS Log:", message)
        }

        function onMessageReceived(message) {
            console.log("CareStepsScreen TSS Message:", message)
        }
    }

    // Auto-connect to TSS server when screen loads
    Component.onCompleted: {
        console.log("CareStepsScreen: Component completed, connecting to TSS server")
        if (tssSocketBridge && !tssSocketBridge.isConnected) {
            tssSocketBridge.connectToServer()
        }
        
        // Check if there's already TSS data available
        if (tssSocketBridge && tssSocketBridge.currentStepNumber > 0) {
            console.log("CareStepsScreen: Found existing TSS data, applying it")
            applyTSSDataDirectly(
                tssSocketBridge.currentStepNumber,
                tssSocketBridge.currentStepDescription,
                tssSocketBridge.currentImageBase64
            )
            
            // Navigate to the step that has data
            let stepIndex = tssSocketBridge.currentStepNumber - 1
            if (stepIndex >= 0 && stepIndex < careSteps.length) {
                console.log("CareStepsScreen: Navigating to step with existing data:", tssSocketBridge.currentStepNumber)
                currentStepIndex = stepIndex
            }
        }
        
        // Check if there's queued data and apply it immediately
        if (hasQueuedData && tssDataQueue.length > 0) {
            console.log("CareStepsScreen: Found queued data on load, applying first step")
            applyNextQueuedStep()
            // Navigate to first step
            currentStepIndex = 0
        }
    }
}
