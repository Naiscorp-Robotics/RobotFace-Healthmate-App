import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Item {
    id: root
    property StackView stackView
    property int currentStepIndex: 0
    
    // Dynamic care steps that can be updated from TSS Socket
    property var careSteps: []

    // Queue to store received TSS data
    property var tssDataQueue: []
    property bool hasQueuedData: false
    
    // Buffer to collect steps within 3 seconds
    property var stepBuffer: []
    property var stepBufferTimer: null
    
    // Flag to track if we've received step 1
    property bool hasReceivedStep1: false
    
    // Array to store all received steps from TSS
    property var allReceivedSteps: []
    
    // Global variable to store all steps across instances
    property var globalAllSteps: []

    // Function to add step to buffer and start timer
    function addStepToBuffer(stepNumber, stepDescription, imageBase64) {
        console.log("CareStepsScreen: Adding step to buffer - Step", stepNumber, ":", stepDescription.substring(0, 50) + "...")
        
        let stepData = {
            stepNumber: stepNumber,
            stepDescription: stepDescription,
            imageBase64: imageBase64,
            timestamp: Date.now()
        }
        
        // If this is step 1, clear the buffer first
        if (stepNumber === 1) {
            console.log("CareStepsScreen: Step 1 received, clearing buffer")
            stepBuffer = []
            hasReceivedStep1 = true
        }
        
        // Add to buffer
        stepBuffer.push(stepData)
        console.log("CareStepsScreen: Buffer now has", stepBuffer.length, "items")
        console.log("CareStepsScreen: Buffer step numbers:", JSON.stringify(stepBuffer.map(s => s.stepNumber)))
        
        // Start or restart timer
        if (stepBufferTimer) {
            stepBufferTimer.stop()
        }
        
        stepBufferTimer = Qt.createQmlObject('import QtQuick 2.15; Timer { interval: 3000; repeat: false; }', root)
        stepBufferTimer.triggered.connect(function() {
            processStepBuffer()
        })
        stepBufferTimer.start()
        
        console.log("CareStepsScreen: Started 3-second timer for step buffer")
    }
    
    // Function to process all steps in buffer
    function processStepBuffer() {
        console.log("CareStepsScreen: Processing step buffer with", stepBuffer.length, "items")
        
        if (stepBuffer.length === 0) {
            console.log("CareStepsScreen: Buffer is empty, nothing to process")
            return
        }
        
        // Sort steps by step number
        stepBuffer.sort(function(a, b) {
            return a.stepNumber - b.stepNumber
        })
        
        console.log("CareStepsScreen: Sorted step numbers:", JSON.stringify(stepBuffer.map(s => s.stepNumber)))
        
        // Apply all steps from buffer
        for (let i = 0; i < stepBuffer.length; i++) {
            let stepData = stepBuffer[i]
            console.log("CareStepsScreen: Processing step", stepData.stepNumber, "from buffer")
            addOrUpdateStep(stepData.stepNumber, stepData.stepDescription, stepData.imageBase64)
        }
        
        // Clear buffer
        stepBuffer = []
        console.log("CareStepsScreen: Buffer cleared after processing")
        
        // Always set current step to step 1 when processing buffer
        if (careSteps.length > 0) {
            // Find step 1 index
            let step1Index = -1
            for (let i = 0; i < careSteps.length; i++) {
                if (careSteps[i].stepNumber === 1) {
                    step1Index = i
                    break
                }
            }
            
            if (step1Index >= 0) {
                currentStepIndex = step1Index
                console.log("CareStepsScreen: Set current step to step 1 (index", step1Index, ")")
            } else {
                // If no step 1, set to first step
                currentStepIndex = 0
                console.log("CareStepsScreen: No step 1 found, set current step to first step (index 0)")
            }
        }
        
        // Debug final state
        debugAllSteps()
    }

    // Function to apply next queued step data
    function applyNextQueuedStep() {
        if (tssDataQueue.length > 0) {
            let stepData = tssDataQueue.shift() // Remove and get first item
            console.log("CareStepsScreen: Applying queued step data:", stepData)
            
            // Add or update step in the dynamic list
            addOrUpdateStep(stepData.stepNumber, stepData.stepDescription, stepData.imageBase64)
            
            // Update queue status
            hasQueuedData = tssDataQueue.length > 0
            console.log("CareStepsScreen: Remaining queue items:", tssDataQueue.length)
        } else {
            console.log("CareStepsScreen: No queued data to apply")
        }
    }

    // Function to add or update a step in the dynamic list
    function addOrUpdateStep(stepNumber, stepDescription, imageBase64) {
        console.log("CareStepsScreen: Adding/updating step", stepNumber, "with description:", stepDescription.substring(0, 50) + "...")
        
        // Find if step already exists
        let existingIndex = -1
        for (let i = 0; i < careSteps.length; i++) {
            if (careSteps[i].stepNumber === stepNumber) {
                existingIndex = i
                break
            }
        }
        
        let stepData = {
            stepNumber: stepNumber,
            title: "Bước " + stepNumber + ": " + stepDescription,
            description: stepDescription,
            image: imageBase64 && imageBase64.length > 0 ? "data:image/png;base64," + imageBase64 : "qrc:/assets/j97.jpg"
        }
        
        if (existingIndex >= 0) {
            // Update existing step
            careSteps[existingIndex] = stepData
            console.log("CareStepsScreen: Updated existing step", stepNumber, "at index", existingIndex)
        } else {
            // Add new step
            careSteps.push(stepData)
            console.log("CareStepsScreen: Added new step", stepNumber, "at index", careSteps.length - 1)
        }
        
        // Sort steps by step number
        careSteps.sort(function(a, b) {
            return a.stepNumber - b.stepNumber
        })
        
        // Force UI update by creating a new array
        let newCareSteps = []
        for (let i = 0; i < careSteps.length; i++) {
            newCareSteps.push(careSteps[i])
        }
        careSteps = newCareSteps
        
        // Only navigate to the step if this is the first step added
        if (careSteps.length === 1) {
            currentStepIndex = 0
        }
        
        console.log("CareStepsScreen: Total steps now:", careSteps.length)
        console.log("CareStepsScreen: Current step index:", currentStepIndex)
        console.log("CareStepsScreen: All step numbers:", JSON.stringify(careSteps.map(s => s.stepNumber)))
        console.log("CareStepsScreen: All step titles:", JSON.stringify(careSteps.map(s => s.title.substring(0, 30) + "...")))
    }

    // Function to navigate to a specific step
    function navigateToStep(stepNumber) {
        for (let i = 0; i < careSteps.length; i++) {
            if (careSteps[i].stepNumber === stepNumber) {
                currentStepIndex = i
                console.log("CareStepsScreen: Navigated to step", stepNumber)
                return
            }
        }
        console.log("CareStepsScreen: Step", stepNumber, "not found")
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
        addOrUpdateStep(stepNumber, stepDescription, imageBase64)
    }

    // Function to debug and show all current steps
    function debugAllSteps() {
        console.log("=== DEBUG ALL STEPS ===")
        console.log("Total steps:", careSteps.length)
        console.log("Current step index:", currentStepIndex)
        for (let i = 0; i < careSteps.length; i++) {
            console.log("Step", i, ":", careSteps[i].stepNumber, "-", careSteps[i].title.substring(0, 50) + "...")
        }
        console.log("=======================")
    }
    
    // Function to process all received steps
    function processAllReceivedSteps() {
        console.log("CareStepsScreen: Processing all received steps:", allReceivedSteps.length)
        
        if (allReceivedSteps.length === 0) {
            console.log("CareStepsScreen: No received steps to process")
            return
        }
        
        // Sort steps by step number
        allReceivedSteps.sort(function(a, b) {
            return a.stepNumber - b.stepNumber
        })
        
        console.log("CareStepsScreen: Sorted allReceivedSteps step numbers:", JSON.stringify(allReceivedSteps.map(s => s.stepNumber)))
        
        // Apply all steps from allReceivedSteps
        for (let i = 0; i < allReceivedSteps.length; i++) {
            let stepData = allReceivedSteps[i]
            console.log("CareStepsScreen: Processing step", stepData.stepNumber, "from allReceivedSteps")
            addOrUpdateStep(stepData.stepNumber, stepData.stepDescription, stepData.imageBase64)
        }
        
        // Set current step to step 1
        if (careSteps.length > 0) {
            // Find step 1 index
            let step1Index = -1
            for (let i = 0; i < careSteps.length; i++) {
                if (careSteps[i].stepNumber === 1) {
                    step1Index = i
                    break
                }
            }
            
            if (step1Index >= 0) {
                currentStepIndex = step1Index
                console.log("CareStepsScreen: Set current step to step 1 (index", step1Index, ")")
            } else {
                // If no step 1, set to first step
                currentStepIndex = 0
                console.log("CareStepsScreen: No step 1 found, set current step to first step (index 0)")
            }
        }
        
        // Debug final state
        debugAllSteps()
    }
    
    // Function to check for missing steps and request them
    function checkForMissingSteps() {
        console.log("CareStepsScreen: Checking for missing steps...")
        
        // If we have current step number, check if we're missing earlier steps
        if (tssSocketBridge && tssSocketBridge.currentStepNumber > 1) {
            console.log("CareStepsScreen: Current step is", tssSocketBridge.currentStepNumber, ", checking for missing steps 1 to", tssSocketBridge.currentStepNumber - 1)
            
            // Request missing steps from server
            for (let i = 1; i < tssSocketBridge.currentStepNumber; i++) {
                console.log("CareStepsScreen: Requesting step", i, "from server")
                // You might need to implement a method to request specific steps
                // For now, we'll add a placeholder step
                addStepToBuffer(i, "Step " + i + " (placeholder - waiting for server data)", "")
            }
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
            text: careSteps.length > 0 ? "Bước " + (currentStepIndex + 1) + "/" + careSteps.length : "Chưa có bước nào"
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

        // Left side - Steps List (1/3 width)
        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredWidth: parent.width * 0.3
            color: "#ffffff"
            radius: 10
            border.color: "#bdc3c7"
            border.width: 1

            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 10
                spacing: 5

                // Steps List Header
                Text {
                    text: "📝 Danh sách các bước"
                    font.pixelSize: 16
                    font.bold: true
                    color: "#2c3e50"
                    Layout.fillWidth: true
                }

                // Steps List
                ScrollView {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    clip: true

                    ListView {
                        id: stepsListView
                        model: careSteps
                        spacing: 5

                        delegate: Rectangle {
                            width: stepsListView.width
                            height: 60
                            color: index === currentStepIndex ? "#3498db" : "#ecf0f1"
                            radius: 8
                            border.color: "#bdc3c7"
                            border.width: 1

                            RowLayout {
                                anchors.fill: parent
                                anchors.margins: 10
                                spacing: 10

                                // Step number
                                Rectangle {
                                    width: 30
                                    height: 30
                                    radius: 15
                                    color: index === currentStepIndex ? "#ffffff" : "#3498db"

                                    Text {
                                        anchors.centerIn: parent
                                        text: modelData.stepNumber
                                        font.pixelSize: 12
                                        font.bold: true
                                        color: index === currentStepIndex ? "#3498db" : "#ffffff"
                                    }
                                }

                                // Step description
                                Text {
                                    text: modelData.description.length > 30 ? modelData.description.substring(0, 30) + "..." : modelData.description
                                    font.pixelSize: 12
                                    color: index === currentStepIndex ? "#ffffff" : "#2c3e50"
                                    wrapMode: Text.WordWrap
                                    Layout.fillWidth: true
                                    Layout.fillHeight: true
                                    verticalAlignment: Text.AlignVCenter
                                }
                            }

                            MouseArea {
                                anchors.fill: parent
                                onClicked: {
                                    currentStepIndex = index
                                }
                            }
                        }
                    }
                }
            }
        }

        // Right side - Step Details (2/3 width)
        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredWidth: parent.width * 0.7
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
                    text: careSteps.length > 0 ? careSteps[currentStepIndex].title : "Chưa có bước nào"
                    font.pixelSize: 18
                    font.bold: true
                    color: "#2c3e50"
                    wrapMode: Text.WordWrap
                    Layout.fillWidth: true
                }

                // Image
                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 200
                    color: "#f8f9fa"
                    radius: 8
                    border.color: "#dee2e6"
                    border.width: 1

                    Image {
                        anchors.fill: parent
                        anchors.margins: 10
                        source: careSteps.length > 0 ? careSteps[currentStepIndex].image : ""
                        fillMode: Image.PreserveAspectFit
                        smooth: true
                        antialiasing: true
                    }
                }

                // Description
                Text {
                    text: careSteps.length > 0 ? careSteps[currentStepIndex].description : "Vui lòng chờ dữ liệu từ hệ thống..."
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
                    enabled: currentStepIndex > 0 && careSteps.length > 0
                    
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
                    text: careSteps.length > 0 && currentStepIndex < careSteps.length - 1 ? "Tiếp theo ➡️" : "Hoàn thành ✅"
                    
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
                        if (careSteps.length > 0 && currentStepIndex < careSteps.length - 1) {
                            currentStepIndex++
                        } else {
                            // Completed all steps
                            if (stackView) {
                                stackView.pop()
                            }
                        }
                    }
                }

                // Debug button
                // Button for debugging: shows all current care steps in the console
                /*
                Button {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 40
                    text: "🐛 Debug"
                    
                    background: Rectangle {
                        color: parent.pressed ? "#9b59b6" : "#8e44ad"
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
                        debugAllSteps()
                    }
                }
                */

                // Force Process Buffer button
                /*
                Button {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 40
                    text: "⚡ Process Buffer (" + stepBuffer.length + ")"
                    enabled: stepBuffer.length > 0
                    
                    background: Rectangle {
                        color: parent.enabled ? (parent.pressed ? "#e74c3c" : "#c0392b") : "#bdc3c7"
                        radius: 8
                    }
                    
                    contentItem: Text {
                        text: parent.text
                        color: parent.enabled ? "#ffffff" : "#7f8c8d"
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                        font.pixelSize: 12
                        font.bold: true
                    }

                    onClicked: {
                        if (stepBuffer.length > 0) {
                            console.log("CareStepsScreen: Force processing buffer with", stepBuffer.length, "items")
                            if (stepBufferTimer) {
                                stepBufferTimer.stop()
                            }
                            processStepBuffer()
                        }
                    }
                }
                */

                // Check Missing Steps button
                /*
                Button {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 40
                    text: "🔍 Check Missing Steps"
                    
                    background: Rectangle {
                        color: parent.pressed ? "#16a085" : "#1abc9c"
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

                    onClicked: {
                        checkForMissingSteps()
                    }
                }
                */

                // Process All Received Steps button
                /*
                Button {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 40
                    text: "📋 Process All Steps (" + allReceivedSteps.length + ")"
                    enabled: allReceivedSteps.length > 0
                    
                    background: Rectangle {
                        color: parent.enabled ? (parent.pressed ? "#e67e22" : "#f39c12") : "#bdc3c7"
                        radius: 8
                    }
                    
                    contentItem: Text {
                        text: parent.text
                        color: parent.enabled ? "#ffffff" : "#7f8c8d"
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                        font.pixelSize: 12
                        font.bold: true
                    }

                    onClicked: {
                        if (allReceivedSteps.length > 0) {
                            console.log("CareStepsScreen: Force processing all received steps")
                            processAllReceivedSteps()
                        }
                    }
                }
                */

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
    }

    // Progress bar
    Rectangle {
        anchors.top: header.bottom
        anchors.left: parent.left
        anchors.right: parent.right
        height: 4
        color: "#ecf0f1"

        Rectangle {
            width: careSteps.length > 0 ? parent.width * ((currentStepIndex + 1) / careSteps.length) : 0
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

    // Connect to TSS Socket signals
    Connections {
        target: tssSocketBridge

        function onStepDataChanged() {
            console.log("CareStepsScreen: Received step data change signal")
            console.log("Step Number:", tssSocketBridge.currentStepNumber)
            console.log("Step Description:", tssSocketBridge.currentStepDescription)
            console.log("Has Image:", tssSocketBridge.currentImageBase64.length > 0)
            
            // Show the TSS data received indicator
            tssDataIndicator.visible = true
            showAnimation.start()
            
            // Store this step in allReceivedSteps array
            let stepData = {
                stepNumber: tssSocketBridge.currentStepNumber,
                stepDescription: tssSocketBridge.currentStepDescription,
                imageBase64: tssSocketBridge.currentImageBase64,
                timestamp: Date.now()
            }
            
            // Check if step already exists in allReceivedSteps
            let existingIndex = -1
            for (let i = 0; i < allReceivedSteps.length; i++) {
                if (allReceivedSteps[i].stepNumber === tssSocketBridge.currentStepNumber) {
                    existingIndex = i
                    break
                }
            }
            
            if (existingIndex >= 0) {
                // Update existing step
                allReceivedSteps[existingIndex] = stepData
                console.log("CareStepsScreen: Updated existing step", tssSocketBridge.currentStepNumber, "in allReceivedSteps")
            } else {
                // Add new step
                allReceivedSteps.push(stepData)
                console.log("CareStepsScreen: Added new step", tssSocketBridge.currentStepNumber, "to allReceivedSteps")
            }
            
            console.log("CareStepsScreen: Total steps in allReceivedSteps:", allReceivedSteps.length)
            console.log("CareStepsScreen: AllReceivedSteps step numbers:", JSON.stringify(allReceivedSteps.map(s => s.stepNumber)))
            
            // Add step to buffer
            addStepToBuffer(
                tssSocketBridge.currentStepNumber,
                tssSocketBridge.currentStepDescription,
                tssSocketBridge.currentImageBase64
            )
            
            // If this is step 1, process all received steps immediately
            if (tssSocketBridge.currentStepNumber === 1) {
                console.log("CareStepsScreen: Step 1 received, processing all received steps immediately")
                if (stepBufferTimer) {
                    stepBufferTimer.stop()
                }
                processAllReceivedSteps()
            } else {
                console.log("CareStepsScreen: Step", tssSocketBridge.currentStepNumber, "added to buffer, waiting for more steps...")
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
        
        // Check if there are global steps available
        if (stackView && stackView.get(0) && stackView.get(0).globalAllSteps && stackView.get(0).globalAllSteps.length > 0) {
            console.log("CareStepsScreen: Found global steps, copying to allReceivedSteps")
            allReceivedSteps = stackView.get(0).globalAllSteps.slice() // Copy array
            console.log("CareStepsScreen: Copied", allReceivedSteps.length, "steps from global")
            console.log("CareStepsScreen: Global step numbers:", JSON.stringify(allReceivedSteps.map(s => s.stepNumber)))
            
            // Process all global steps immediately
            processAllReceivedSteps()
        } else if (tssSocketBridge && tssSocketBridge.currentStepNumber > 0) {
            console.log("CareStepsScreen: Found existing TSS data, adding to allReceivedSteps")
            
            // Add current step to allReceivedSteps
            let stepData = {
                stepNumber: tssSocketBridge.currentStepNumber,
                stepDescription: tssSocketBridge.currentStepDescription,
                imageBase64: tssSocketBridge.currentImageBase64,
                timestamp: Date.now()
            }
            allReceivedSteps.push(stepData)
            
            // If this is step 1, process all received steps immediately
            if (tssSocketBridge.currentStepNumber === 1) {
                console.log("CareStepsScreen: Found step 1, processing all received steps immediately")
                processAllReceivedSteps()
            } else {
                // If we don't have step 1, create a placeholder and wait for more steps
                console.log("CareStepsScreen: No step 1 found, creating placeholder and waiting for more steps")
                addOrUpdateStep(1, "Đang chờ dữ liệu cho bước 1...", "")
                currentStepIndex = 0
            }
        } else {
            // No TSS data available, create placeholder for step 1
            console.log("CareStepsScreen: No TSS data available, creating placeholder for step 1")
            addOrUpdateStep(1, "Đang chờ dữ liệu cho bước 1...", "")
            currentStepIndex = 0
        }
    }
}
