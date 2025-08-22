import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import AudioController 1.0

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

    // Audio Controller for voice playback
    AudioController {
        id: voicePlayer
        
        onErrorOccurred: function(message) {
            console.log("Voice Player Error:", message)
            voiceStatus.text = "❌ Lỗi: " + message
            voiceStatus.color = "#e74c3c"
        }
        
        onAudioPlayed: {
            console.log("Voice playback completed")
            voiceStatus.text = "✅ Phát âm thanh hoàn thành"
            voiceStatus.color = "#27ae60"
        }
    }
    
    // Voice status indicator
    Text {
        id: voiceStatus
        anchors.top: header.bottom
        anchors.left: parent.left
        anchors.margins: 10
        anchors.topMargin: 15
        text: "🔊 Sẵn sàng phát âm thanh"
        color: "#3498db"
        font.pixelSize: 12
        font.bold: true
        z: 10
    }

    // Function to decode base64 and load audio directly
    function playVoiceFromBase64Optimized(voiceBase64) {
        console.log("CareStepsScreen: Attempting to play voice from base64 (optimized)")
        console.log("CareStepsScreen: Original base64 length:", voiceBase64 ? voiceBase64.length : 0)
        
        if (voiceBase64 && voiceBase64.length > 0) {
            // Use base64 data directly from server without cleaning
            console.log("CareStepsScreen: Using base64 data directly from server (optimized)")
            console.log("CareStepsScreen: Base64 length:", voiceBase64.length)
            
            if (voiceBase64.length < 100) {
                console.log("CareStepsScreen: Base64 too short, likely invalid")
                playDefaultVoice()
                return
            }
            
            voiceStatus.text = "🔄 Đang phát âm thanh từ server..."
            voiceStatus.color = "#f39c12"
            
            // Try to load and play
            if (voicePlayer.loadFromBase64(voiceBase64)) {
                console.log("CareStepsScreen: Successfully loaded server audio, starting playback...")
                voicePlayer.playAudio()
            } else {
                console.log("CareStepsScreen: Failed to load voice from base64, falling back to default")
                playDefaultVoice()
            }
        } else {
            console.log("CareStepsScreen: No voice data from server, playing default voice")
            voiceStatus.text = "🔄 Đang phát âm thanh mặc định..."
            voiceStatus.color = "#f39c12"
            playDefaultVoice()
        }
    }

    // Function to play voice from base64
    function playVoiceFromBase64(voiceBase64) {
        console.log("CareStepsScreen: Attempting to play voice from base64")
        console.log("CareStepsScreen: Original base64 length:", voiceBase64 ? voiceBase64.length : 0)
        console.log("CareStepsScreen: Original base64 preview:", voiceBase64 ? voiceBase64.substring(0, 100) + "..." : "null")
        
        if (voiceBase64 && voiceBase64.length > 0) {
            // Use base64 data directly from server without cleaning
            console.log("CareStepsScreen: Using base64 data directly from server")
            console.log("CareStepsScreen: Base64 length:", voiceBase64.length)
            console.log("CareStepsScreen: Base64 preview:", voiceBase64.substring(0, 100) + "...")
            
            // Validate base64 length
            if (voiceBase64.length < 1000) {
                console.log("CareStepsScreen: Base64 too short, likely invalid")
                voiceStatus.text = "❌ Dữ liệu âm thanh không hợp lệ"
                voiceStatus.color = "#e74c3c"
                playDefaultVoice()
                return
            }
            
            // Server returned voice data, play it
            console.log("CareStepsScreen: Playing voice from server data")
            voiceStatus.text = "🔄 Đang phát âm thanh từ server..."
            voiceStatus.color = "#f39c12"
            
            // Try to load and play
            if (voicePlayer.loadFromBase64(voiceBase64)) {
                console.log("CareStepsScreen: Successfully loaded server audio, starting playback...")
                voicePlayer.playAudio()
            } else {
                console.log("CareStepsScreen: Failed to load voice from base64, falling back to default")
                console.log("CareStepsScreen: Base64 was invalid or corrupted")
                voiceStatus.text = "❌ Không thể phát âm thanh từ server"
                voiceStatus.color = "#e74c3c"
                playDefaultVoice()
            }
        } else {
            // No voice data from server, play default voice
            console.log("CareStepsScreen: No voice data from server, playing default voice")
            voiceStatus.text = "🔄 Đang phát âm thanh mặc định..."
            voiceStatus.color = "#f39c12"
            playDefaultVoice()
        }
    }
    
    // Function to play default voice from assets/voice_base.txt
    function playDefaultVoice() {
        console.log("CareStepsScreen: Loading default voice from assets/voice_base.txt")
        
        // Try multiple approaches to load the file
        loadVoiceFileWithRetry()
    }
    
    // Function to try different methods to load voice file
    function loadVoiceFileWithRetry() {
        console.log("CareStepsScreen: Trying to load voice file with multiple methods...")
        
        // Method 1: Try dedicated voice file reader
        if (fileHelper) {
            console.log("CareStepsScreen: Method 1 - Using dedicated voice file reader")
            var base64Data = fileHelper.readVoiceBaseFile()
            console.log("CareStepsScreen: Dedicated reader result length:", base64Data.length)
            
            if (base64Data && base64Data.length > 1000) { // Check if we got substantial data
                console.log("CareStepsScreen: Successfully loaded voice from dedicated reader")
                if (voicePlayer.loadFromBase64(base64Data)) {
                    voicePlayer.playAudio()
                    return
                }
            }
        }
        
        // Method 2: Try FileHelper with resource path
        if (fileHelper && fileHelper.resourceFileExists("qrc:/assets/voice_base.txt")) {
            console.log("CareStepsScreen: Method 2 - Using FileHelper with resource path")
            var base64Data = fileHelper.readResourceFile("qrc:/assets/voice_base.txt")
            console.log("CareStepsScreen: FileHelper result length:", base64Data.length)
            
            if (base64Data && base64Data.length > 1000) { // Check if we got substantial data
                console.log("CareStepsScreen: Successfully loaded voice from FileHelper")
                if (voicePlayer.loadFromBase64(base64Data)) {
                    voicePlayer.playAudio()
                    return
                }
            }
        }
        
        // Method 3: Try XMLHttpRequest
        console.log("CareStepsScreen: Method 3 - Using XMLHttpRequest")
        loadVoiceWithXMLHttpRequest()
    }
    
    // Function to load voice using XMLHttpRequest
    function loadVoiceWithXMLHttpRequest() {
        var xhr = new XMLHttpRequest()
        xhr.open("GET", "qrc:/assets/voice_base.txt")
        xhr.onreadystatechange = function() {
            if (xhr.readyState === XMLHttpRequest.DONE) {
                if (xhr.status === 200) {
                    var base64Data = xhr.responseText.trim()
                    console.log("CareStepsScreen: XMLHttpRequest loaded voice, length:", base64Data.length)
                    
                    if (base64Data && base64Data.length > 1000) {
                        console.log("CareStepsScreen: Successfully loaded voice from XMLHttpRequest")
                        if (voicePlayer.loadFromBase64(base64Data)) {
                            voicePlayer.playAudio()
                            return
                        }
                    }
                }
                
                // If XMLHttpRequest failed, try fallback
                console.log("CareStepsScreen: XMLHttpRequest failed, using fallback")
                playDefaultVoiceFallback()
            }
        }
        xhr.send()
    }
    
    // Fallback function with hardcoded base64 audio (short beep)
    function playDefaultVoiceFallback() {
        console.log("CareStepsScreen: Using fallback voice (short beep)")
        voiceStatus.text = "🔄 Đang phát âm thanh mặc định (fallback)..."
        voiceStatus.color = "#f39c12"
        
        // Short beep sound in base64 (WAV format)
        var fallbackBase64 = "UklGRnoGAABXQVZFZm10IBAAAAABAAEAQB8AAEAfAAABAAgAZGF0YQoGAACBhYqFbF1fdJivrJBhNjVgodDbq2EcBj+a2/LDciUFLIHO8tiJNwgZaLvt559NEAxQp+PwtmMcBjiR1/LMeSwFJHfH8N2QQAoUXrTp66hVFApGn+DyvmwhBSuBzvLZiTYIG2m98OScTgwOUarm7blmGgU7k9n1unEiBC13yO/eizEIHWq+8+OWT"
        
        if (voicePlayer.loadFromBase64(fallbackBase64)) {
            voicePlayer.playAudio()
        } else {
            console.log("CareStepsScreen: Failed to load fallback voice")
            voiceStatus.text = "❌ Không thể phát âm thanh mặc định"
            voiceStatus.color = "#e74c3c"
        }
    }

    // Function to add step to buffer and start timer
    function addStepToBuffer(stepNumber, stepDescription, imageBase64, voiceBase64) {
        console.log("CareStepsScreen: Adding step to buffer - Step", stepNumber, ":", stepDescription ? stepDescription.substring(0, 50) + "..." : "null")
        console.log("CareStepsScreen: Buffer input voiceBase64 length:", voiceBase64 ? voiceBase64.length : 0)
        console.log("CareStepsScreen: Buffer input voiceBase64 preview:", voiceBase64 ? voiceBase64.substring(0, 100) + "..." : "null")
        console.log("CareStepsScreen: Buffer input voiceBase64 is null:", voiceBase64 === null)
        console.log("CareStepsScreen: Buffer input voiceBase64 is undefined:", voiceBase64 === undefined)
        console.log("CareStepsScreen: Buffer input voiceBase64 type:", typeof voiceBase64)
        
        let stepData = {
            stepNumber: stepNumber,
            stepDescription: stepDescription,
            imageBase64: imageBase64,
            voiceBase64: voiceBase64 || "",
            timestamp: Date.now()
        }
        
        console.log("CareStepsScreen: Created buffer stepData with voiceBase64 length:", stepData.voiceBase64 ? stepData.voiceBase64.length : 0)
        console.log("CareStepsScreen: Created buffer stepData voiceBase64 preview:", stepData.voiceBase64 ? stepData.voiceBase64.substring(0, 100) + "..." : "null")
        
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
        console.log("CareStepsScreen: Last added step voiceBase64 length:", stepBuffer[stepBuffer.length - 1].voiceBase64.length)
        console.log("CareStepsScreen: Last added step voiceBase64 preview:", stepBuffer[stepBuffer.length - 1].voiceBase64.substring(0, 100) + "...")
        
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
            console.log("CareStepsScreen: Buffer step voiceBase64 length:", stepData.voiceBase64 ? stepData.voiceBase64.length : 0)
            console.log("CareStepsScreen: Buffer step voiceBase64 preview:", stepData.voiceBase64 ? stepData.voiceBase64.substring(0, 100) + "..." : "null")
            addOrUpdateStep(stepData.stepNumber, stepData.stepDescription, stepData.imageBase64, stepData.voiceBase64)
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
            addOrUpdateStep(stepData.stepNumber, stepData.stepDescription, stepData.imageBase64, stepData.voiceBase64)
            
            // Update queue status
            hasQueuedData = tssDataQueue.length > 0
            console.log("CareStepsScreen: Remaining queue items:", tssDataQueue.length)
        } else {
            console.log("CareStepsScreen: No queued data to apply")
        }
    }

    // Function to add or update a step in the dynamic list
    function addOrUpdateStep(stepNumber, stepDescription, imageBase64, voiceBase64) {
        console.log("CareStepsScreen: addOrUpdateStep called with:")
        console.log("  - stepNumber:", stepNumber)
        console.log("  - stepDescription:", stepDescription.substring(0, 50) + "...")
        console.log("  - imageBase64 length:", imageBase64 ? imageBase64.length : 0)
        console.log("  - voiceBase64 length:", voiceBase64 ? voiceBase64.length : 0)
        console.log("  - voiceBase64 preview:", voiceBase64 ? voiceBase64.substring(0, 100) + "..." : "null")
        console.log("  - voiceBase64 is null:", voiceBase64 === null)
        console.log("  - voiceBase64 is undefined:", voiceBase64 === undefined)
        console.log("  - voiceBase64 type:", typeof voiceBase64)
        
        console.log("CareStepsScreen: Adding/updating step", stepNumber, "with description:", stepDescription.substring(0, 50) + "...")
        console.log("CareStepsScreen: Step", stepNumber, "voiceBase64 length:", voiceBase64 ? voiceBase64.length : 0)
        console.log("CareStepsScreen: Step", stepNumber, "voiceBase64 preview:", voiceBase64 ? voiceBase64.substring(0, 100) + "..." : "null")
        console.log("CareStepsScreen: Step", stepNumber, "voiceBase64 is null:", voiceBase64 === null)
        console.log("CareStepsScreen: Step", stepNumber, "voiceBase64 is undefined:", voiceBase64 === undefined)
        console.log("CareStepsScreen: Step", stepNumber, "voiceBase64 type:", typeof voiceBase64)
        
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
            image: imageBase64 && imageBase64.length > 0 ? "data:image/png;base64," + imageBase64 : "qrc:/assets/j97.jpg",
            imageBase64: imageBase64 || "",
            voiceBase64: voiceBase64 || ""
        }
        
        console.log("CareStepsScreen: Created stepData object:")
        console.log("  - stepNumber:", stepData.stepNumber)
        console.log("  - imageBase64 length:", stepData.imageBase64.length)
        console.log("  - voiceBase64 length:", stepData.voiceBase64.length)
        console.log("  - voiceBase64 preview:", stepData.voiceBase64.substring(0, 100) + "...")
        console.log("  - voiceBase64 is null:", stepData.voiceBase64 === null)
        console.log("  - voiceBase64 is undefined:", stepData.voiceBase64 === undefined)
        console.log("  - voiceBase64 type:", typeof stepData.voiceBase64)
        
        if (existingIndex >= 0) {
            // Update existing step
            careSteps[existingIndex] = stepData
            console.log("CareStepsScreen: Updated existing step", stepNumber, "at index", existingIndex)
            console.log("CareStepsScreen: Updated step voiceBase64 length:", careSteps[existingIndex].voiceBase64 ? careSteps[existingIndex].voiceBase64.length : 0)
            console.log("CareStepsScreen: Updated step voiceBase64 preview:", careSteps[existingIndex].voiceBase64 ? careSteps[existingIndex].voiceBase64.substring(0, 100) + "..." : "null")
        } else {
            // Add new step
            careSteps.push(stepData)
            console.log("CareStepsScreen: Added new step", stepNumber, "at index", careSteps.length - 1)
            console.log("CareStepsScreen: New step voiceBase64 length:", careSteps[careSteps.length - 1].voiceBase64 ? careSteps[careSteps.length - 1].voiceBase64.length : 0)
            console.log("CareStepsScreen: New step voiceBase64 preview:", careSteps[careSteps.length - 1].voiceBase64 ? careSteps[careSteps.length - 1].voiceBase64.substring(0, 100) + "..." : "null")
        }
        
        // Sort steps by step number
        careSteps.sort(function(a, b) {
            return a.stepNumber - b.stepNumber
        })
        
        // Force UI update by creating a new array
        let newCareSteps = []
        for (let i = 0; i < careSteps.length; i++) {
            let newStep = {
                stepNumber: careSteps[i].stepNumber,
                title: careSteps[i].title,
                description: careSteps[i].description,
                image: careSteps[i].image,
                imageBase64: careSteps[i].imageBase64 || "",
                voiceBase64: careSteps[i].voiceBase64 || ""
            }
            console.log("CareStepsScreen: Copying step", i, "to newCareSteps:")
            console.log("  - Original voiceBase64 length:", careSteps[i].voiceBase64 ? careSteps[i].voiceBase64.length : 0)
            console.log("  - Original voiceBase64 preview:", careSteps[i].voiceBase64 ? careSteps[i].voiceBase64.substring(0, 100) + "..." : "null")
            console.log("  - Copied voiceBase64 length:", newStep.voiceBase64.length)
            console.log("  - Copied voiceBase64 preview:", newStep.voiceBase64.substring(0, 100) + "...")
            newCareSteps.push(newStep)
        }
        careSteps = newCareSteps
        
        // Only navigate to the step if this is the first step added
        if (careSteps.length === 1) {
            currentStepIndex = 0
        }
        
        console.log("CareStepsScreen: Total steps now:", careSteps.length)
        console.log("CareStepsScreen: Current step index:", currentStepIndex)
        console.log("CareStepsScreen: All step numbers:", JSON.stringify(careSteps.map(s => s.stepNumber)))
        console.log("CareStepsScreen: All step voiceBase64 lengths:", JSON.stringify(careSteps.map(s => s.voiceBase64 ? s.voiceBase64.length : 0)))
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
    function applyTSSDataDirectly(stepNumber, stepDescription, imageBase64, voiceBase64) {
        console.log("CareStepsScreen: Applying TSS data directly to step", stepNumber)
        addOrUpdateStep(stepNumber, stepDescription, imageBase64, voiceBase64)
    }

    // Function to debug and show all current steps
    function debugAllSteps() {
        console.log("=== DEBUG ALL STEPS ===")
        console.log("Total steps:", careSteps.length)
        console.log("Current step index:", currentStepIndex)
        for (let i = 0; i < careSteps.length; i++) {
            console.log("Step", i, ":", careSteps[i].stepNumber, "-", careSteps[i].title.substring(0, 50) + "...")
            console.log("  - imageBase64 length:", careSteps[i].imageBase64 ? careSteps[i].imageBase64.length : 0)
            console.log("  - voiceBase64 length:", careSteps[i].voiceBase64 ? careSteps[i].voiceBase64.length : 0)
            console.log("  - voiceBase64 preview:", careSteps[i].voiceBase64 ? careSteps[i].voiceBase64.substring(0, 100) + "..." : "null")
            console.log("  - Has valid audio:", careSteps[i].voiceBase64 && careSteps[i].voiceBase64.length > 1000)
            console.log("  - Has valid image:", careSteps[i].imageBase64 && careSteps[i].imageBase64.length > 1000)
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
            console.log("CareStepsScreen: AllReceivedSteps step voiceBase64 length:", stepData.voiceBase64 ? stepData.voiceBase64.length : 0)
            console.log("CareStepsScreen: AllReceivedSteps step voiceBase64 preview:", stepData.voiceBase64 ? stepData.voiceBase64.substring(0, 100) + "..." : "null")
            console.log("CareStepsScreen: About to call addOrUpdateStep with voiceBase64 length:", stepData.voiceBase64 ? stepData.voiceBase64.length : 0)
            addOrUpdateStep(stepData.stepNumber, stepData.stepDescription, stepData.imageBase64, stepData.voiceBase64)
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
                                    // Navigate to the selected step (auto-play will be handled by onCurrentStepIndexChanged)
                                    currentStepIndex = index
                                    console.log("CareStepsScreen: Step clicked - Navigated to Index:", index, "Step Number:", modelData.stepNumber)
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
                            // Navigate to previous step (auto-play will be handled by onCurrentStepIndexChanged)
                            currentStepIndex--
                            console.log("CareStepsScreen: Back button - Navigated to step", currentStepIndex + 1)
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
                            // Navigate to next step (auto-play will be handled by onCurrentStepIndexChanged)
                            currentStepIndex++
                            console.log("CareStepsScreen: Next button - Navigated to step", currentStepIndex + 1)
                        } else {
                            // Completed all steps
                            if (stackView) {
                                stackView.pop()
                            }
                        }
                    }
                }

                // Play Voice button
                Button {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 40
                    text: "🔊 Phát âm thanh"
                    enabled: careSteps.length > 0
                    
                    background: Rectangle {
                        color: parent.enabled ? (parent.pressed ? "#9b59b6" : "#8e44ad") : "#bdc3c7"
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
                        if (careSteps.length > 0) {
                            // Try to get voice data from current step (if available)
                            var currentStep = careSteps[currentStepIndex]
                            var voiceBase64 = currentStep.voiceBase64 || ""
                            playVoiceFromBase64(voiceBase64)
                        }
                    }
                }

                // Debug button
                // Button for debugging: shows all current care steps in the console
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

                // Force Process All Steps button
                Button {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 40
                    text: "🔄 Force Process All Steps"
                    
                    background: Rectangle {
                        color: parent.pressed ? "#8e44ad" : "#9b59b6"
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
                        console.log("CareStepsScreen: Force processing all steps")
                        console.log("CareStepsScreen: allReceivedSteps length:", allReceivedSteps.length)
                        for (let i = 0; i < allReceivedSteps.length; i++) {
                            console.log("CareStepsScreen: allReceivedSteps[", i, "] voiceBase64 length:", allReceivedSteps[i].voiceBase64 ? allReceivedSteps[i].voiceBase64.length : 0)
                        }
                        processAllReceivedSteps()
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

                // Debug TSS Data button
                Button {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 40
                    text: "🔍 Debug TSS Data"
                    
                    background: Rectangle {
                        color: parent.pressed ? "#e67e22" : "#d35400"
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
                        console.log("=== DEBUG TSS SOCKET BRIDGE DATA ===")
                        console.log("Is Connected:", tssSocketBridge.isConnected)
                        console.log("Current Step Number:", tssSocketBridge.currentStepNumber)
                        console.log("Current Step Description:", tssSocketBridge.currentStepDescription)
                        console.log("Image Base64 Length:", tssSocketBridge.currentImageBase64 ? tssSocketBridge.currentImageBase64.length : 0)
                        console.log("Audio Base64 Length:", tssSocketBridge.currentAudioBase64 ? tssSocketBridge.currentAudioBase64.length : 0)
                        console.log("Audio Base64 Preview:", tssSocketBridge.currentAudioBase64 ? tssSocketBridge.currentAudioBase64.substring(0, 100) + "..." : "null")
                        console.log("Audio Base64 is null:", tssSocketBridge.currentAudioBase64 === null)
                        console.log("Audio Base64 is undefined:", tssSocketBridge.currentAudioBase64 === undefined)
                        console.log("Audio Base64 type:", typeof tssSocketBridge.currentAudioBase64)
                        console.log("=====================================")
                    }
                }

                // Test Request Step button
                Button {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 40
                    text: "🔄 Test Request Step 1"
                    
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
                        console.log("CareStepsScreen: Testing request for step 1")
                        if (tssSocketBridge) {
                            tssSocketBridge.sendStepRequest(1, "Test step description")
                        }
                    }
                }

                // Test Direct Audio button
                Button {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 40
                    text: "🔊 Test Direct Audio"
                    
                    background: Rectangle {
                        color: parent.pressed ? "#e74c3c" : "#c0392b"
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
                        console.log("CareStepsScreen: Testing direct audio from TSS Socket Bridge")
                        if (tssSocketBridge && tssSocketBridge.currentAudioBase64) {
                            console.log("CareStepsScreen: Direct audio test - length:", tssSocketBridge.currentAudioBase64.length)
                            console.log("CareStepsScreen: Direct audio test - preview:", tssSocketBridge.currentAudioBase64.substring(0, 100) + "...")
                            playVoiceFromBase64(tssSocketBridge.currentAudioBase64)
                        } else {
                            console.log("CareStepsScreen: No audio data available in TSS Socket Bridge")
                        }
                    }
                }

                // Test Create StepData button
                Button {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 40
                    text: "🧪 Test Create StepData"
                    
                    background: Rectangle {
                        color: parent.pressed ? "#f39c12" : "#e67e22"
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
                        console.log("CareStepsScreen: Testing create stepData")
                        if (tssSocketBridge) {
                            console.log("CareStepsScreen: TSS Socket Bridge data:")
                            console.log("  - currentAudioBase64:", tssSocketBridge.currentAudioBase64)
                            console.log("  - currentAudioBase64 type:", typeof tssSocketBridge.currentAudioBase64)
                            console.log("  - currentAudioBase64 is null:", tssSocketBridge.currentAudioBase64 === null)
                            console.log("  - currentAudioBase64 is undefined:", tssSocketBridge.currentAudioBase64 === undefined)
                            console.log("  - currentAudioBase64 length:", tssSocketBridge.currentAudioBase64 ? tssSocketBridge.currentAudioBase64.length : 0)
                            
                            let testStepData = {
                                stepNumber: tssSocketBridge.currentStepNumber,
                                stepDescription: tssSocketBridge.currentStepDescription,
                                imageBase64: tssSocketBridge.currentImageBase64 || "",
                                voiceBase64: tssSocketBridge.currentAudioBase64 || "",
                                timestamp: Date.now()
                            }
                            
                            console.log("CareStepsScreen: Test stepData created:")
                            console.log("  - voiceBase64 length:", testStepData.voiceBase64.length)
                            console.log("  - voiceBase64 preview:", testStepData.voiceBase64.substring(0, 100) + "...")
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
            
            // Add delay to ensure TSS Socket Bridge data is updated
            var processTimer = Qt.createQmlObject('import QtQuick 2.15; Timer { interval: 100; repeat: false; }', root)
            processTimer.triggered.connect(function() {
                console.log("CareStepsScreen: Processing step data after delay")
                console.log("Step Number:", tssSocketBridge.currentStepNumber)
                console.log("Step Description:", tssSocketBridge.currentStepDescription)
                console.log("Has Image:", tssSocketBridge.currentImageBase64 && tssSocketBridge.currentImageBase64.length > 0)
                console.log("Audio Base64 Length:", tssSocketBridge.currentAudioBase64 ? tssSocketBridge.currentAudioBase64.length : 0)
                console.log("Audio Base64 Preview:", tssSocketBridge.currentAudioBase64 ? tssSocketBridge.currentAudioBase64.substring(0, 100) + "..." : "null")
                console.log("Audio Base64 is null:", tssSocketBridge.currentAudioBase64 === null)
                console.log("Audio Base64 is undefined:", tssSocketBridge.currentAudioBase64 === undefined)
                console.log("Audio Base64 type:", typeof tssSocketBridge.currentAudioBase64)
                
                // Show the TSS data received indicator
                tssDataIndicator.visible = true
                showAnimation.start()
                
                // Store this step in allReceivedSteps array
                console.log("CareStepsScreen: About to create stepData:")
                console.log("  - tssSocketBridge.currentAudioBase64:", tssSocketBridge.currentAudioBase64)
                console.log("  - tssSocketBridge.currentAudioBase64 type:", typeof tssSocketBridge.currentAudioBase64)
                console.log("  - tssSocketBridge.currentAudioBase64 is null:", tssSocketBridge.currentAudioBase64 === null)
                console.log("  - tssSocketBridge.currentAudioBase64 is undefined:", tssSocketBridge.currentAudioBase64 === undefined)
                console.log("  - tssSocketBridge.currentAudioBase64 length:", tssSocketBridge.currentAudioBase64 ? tssSocketBridge.currentAudioBase64.length : 0)
                
                let stepData = {
                    stepNumber: tssSocketBridge.currentStepNumber,
                    stepDescription: tssSocketBridge.currentStepDescription,
                    imageBase64: tssSocketBridge.currentImageBase64 || "",
                    voiceBase64: tssSocketBridge.currentAudioBase64 || "",
                    timestamp: Date.now()
                }
                
                console.log("CareStepsScreen: Created stepData in onStepDataChanged:")
                console.log("  - stepNumber:", stepData.stepNumber)
                console.log("  - imageBase64 length:", stepData.imageBase64.length)
                console.log("  - voiceBase64 length:", stepData.voiceBase64.length)
                console.log("  - voiceBase64 preview:", stepData.voiceBase64.substring(0, 100) + "...")
                console.log("  - voiceBase64 is null:", stepData.voiceBase64 === null)
                console.log("  - voiceBase64 is undefined:", stepData.voiceBase64 === undefined)
                console.log("  - voiceBase64 type:", typeof stepData.voiceBase64)
                
                console.log("CareStepsScreen: Created stepData with voiceBase64 length:", stepData.voiceBase64 ? stepData.voiceBase64.length : 0)
                console.log("CareStepsScreen: Created stepData voiceBase64 preview:", stepData.voiceBase64 ? stepData.voiceBase64.substring(0, 100) + "..." : "null")
                
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
                    console.log("CareStepsScreen: Updated step voiceBase64 length:", allReceivedSteps[existingIndex].voiceBase64.length)
                } else {
                    // Add new step
                    allReceivedSteps.push(stepData)
                    console.log("CareStepsScreen: Added new step", tssSocketBridge.currentStepNumber, "to allReceivedSteps")
                    console.log("CareStepsScreen: Added step voiceBase64 length:", allReceivedSteps[allReceivedSteps.length - 1].voiceBase64.length)
                }
                
                console.log("CareStepsScreen: Total steps in allReceivedSteps:", allReceivedSteps.length)
                console.log("CareStepsScreen: AllReceivedSteps step numbers:", JSON.stringify(allReceivedSteps.map(s => s.stepNumber)))
                
                // Add step to buffer
                console.log("CareStepsScreen: About to add step to buffer:")
                console.log("  - Step number:", tssSocketBridge.currentStepNumber)
                console.log("  - Image base64 length:", tssSocketBridge.currentImageBase64 ? tssSocketBridge.currentImageBase64.length : 0)
                console.log("  - Audio base64 length:", tssSocketBridge.currentAudioBase64 ? tssSocketBridge.currentAudioBase64.length : 0)
                console.log("  - Audio base64 preview:", tssSocketBridge.currentAudioBase64 ? tssSocketBridge.currentAudioBase64.substring(0, 100) + "..." : "null")
                
                addStepToBuffer(
                    tssSocketBridge.currentStepNumber,
                    tssSocketBridge.currentStepDescription,
                    tssSocketBridge.currentImageBase64 || "",
                    tssSocketBridge.currentAudioBase64 || ""
                )
                
                // If this is step 1, process all received steps immediately
                if (tssSocketBridge.currentStepNumber === 1) {
                    console.log("CareStepsScreen: Step 1 received, processing all received steps immediately")
                    console.log("CareStepsScreen: Step 1 audio base64 length:", tssSocketBridge.currentAudioBase64 ? tssSocketBridge.currentAudioBase64.length : 0)
                    console.log("CareStepsScreen: Step 1 audio base64 preview:", tssSocketBridge.currentAudioBase64 ? tssSocketBridge.currentAudioBase64.substring(0, 100) + "..." : "null")
                    
                    if (stepBufferTimer) {
                        stepBufferTimer.stop()
                    }
                    processAllReceivedSteps()
                    
                    // Auto-play voice for step 1 with delay to ensure UI is updated
                    var step1AutoPlayTimer = Qt.createQmlObject('import QtQuick 2.15; Timer { interval: 500; repeat: false; }', root)
                    step1AutoPlayTimer.triggered.connect(function() {
                        if (tssSocketBridge.currentAudioBase64 && tssSocketBridge.currentAudioBase64.length > 0) {
                            console.log("CareStepsScreen: Auto-playing voice for step 1")
                            console.log("CareStepsScreen: Step 1 voice data length:", tssSocketBridge.currentAudioBase64.length)
                            playVoiceFromBase64(tssSocketBridge.currentAudioBase64)
                        } else {
                            console.log("CareStepsScreen: Step 1 has no audio data, playing default voice")
                            playDefaultVoice()
                        }
                        step1AutoPlayTimer.destroy()
                    })
                    step1AutoPlayTimer.start()
                } else {
                    console.log("CareStepsScreen: Step", tssSocketBridge.currentStepNumber, "added to buffer, waiting for more steps...")
                }
                
                processTimer.destroy()
            })
            processTimer.start()
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
            console.log("CareStepsScreen: Global steps length:", stackView.get(0).globalAllSteps.length)
            
            // Copy global steps with deep copy to ensure all properties are copied
            allReceivedSteps = []
            for (let i = 0; i < stackView.get(0).globalAllSteps.length; i++) {
                let globalStep = stackView.get(0).globalAllSteps[i]
                console.log("CareStepsScreen: Copying global step", i, ":", globalStep.stepNumber)
                console.log("CareStepsScreen: Global step voiceBase64 length:", globalStep.voiceBase64 ? globalStep.voiceBase64.length : 0)
                console.log("CareStepsScreen: Global step voiceBase64 preview:", globalStep.voiceBase64 ? globalStep.voiceBase64.substring(0, 100) + "..." : "null")
                
                let copiedStep = {
                    stepNumber: globalStep.stepNumber,
                    stepDescription: globalStep.stepDescription,
                    imageBase64: globalStep.imageBase64 || "",
                    voiceBase64: globalStep.voiceBase64 || "",
                    timestamp: globalStep.timestamp || Date.now()
                }
                
                console.log("CareStepsScreen: Copied step voiceBase64 length:", copiedStep.voiceBase64.length)
                console.log("CareStepsScreen: Copied step voiceBase64 preview:", copiedStep.voiceBase64.substring(0, 100) + "...")
                
                allReceivedSteps.push(copiedStep)
            }
            
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
                imageBase64: tssSocketBridge.currentImageBase64 || "",
                voiceBase64: tssSocketBridge.currentAudioBase64 || "",
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

    // Auto-play voice when current step changes
    onCurrentStepIndexChanged: {
        console.log("CareStepsScreen: Current step index changed to:", currentStepIndex)
        
        if (careSteps.length > 0 && currentStepIndex >= 0 && currentStepIndex < careSteps.length) {
            var currentStep = careSteps[currentStepIndex]
            var voiceBase64 = currentStep.voiceBase64 || ""
            console.log("CareStepsScreen: Auto-playing voice for step", currentStep.stepNumber)
            console.log("CareStepsScreen: Step voiceBase64 length:", voiceBase64 ? voiceBase64.length : 0)
            console.log("CareStepsScreen: Step voiceBase64 is null:", voiceBase64 === null)
            console.log("CareStepsScreen: Step voiceBase64 is undefined:", voiceBase64 === undefined)
            console.log("CareStepsScreen: Step voiceBase64 type:", typeof voiceBase64)
            console.log("CareStepsScreen: Step voiceBase64 preview:", voiceBase64 ? voiceBase64.substring(0, 100) + "..." : "null")
            
            // Auto-play voice with delay to ensure UI is updated
            var autoPlayTimer = Qt.createQmlObject('import QtQuick 2.15; Timer { interval: 200; repeat: false; }', root)
            autoPlayTimer.triggered.connect(function() {
                if (voiceBase64 && voiceBase64.length > 0) {
                    console.log("CareStepsScreen: Playing voice from server data for step", currentStep.stepNumber)
                    playVoiceFromBase64(voiceBase64)
                } else {
                    console.log("CareStepsScreen: No voice data for step", currentStep.stepNumber, "- playing default voice")
                    playDefaultVoice()
                }
                autoPlayTimer.destroy()
            })
            autoPlayTimer.start()
        }
    }
}
