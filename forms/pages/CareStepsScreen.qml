import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import AudioController 1.0

Item {
    id: root
    property StackView stackView
    property int currentStepIndex: 0
    
    // Dynamic care steps
    property var careSteps: []

    // TSS queue/buffer
    property var tssDataQueue: []
    property bool hasQueuedData: false
    
    property var stepBuffer: []
    property var stepBufferTimer: null
    property bool hasReceivedStep1: false
    
    property var allReceivedSteps: []
    property var globalAllSteps: []

    // ---- NEW: chỉ phát khi trang đã Active ----
    property bool willAutoplayWhenActive: false

    function playForCurrentStep() {
        if (careSteps.length === 0 || currentStepIndex < 0 || currentStepIndex >= careSteps.length)
            return
        var v = careSteps[currentStepIndex].voiceBase64 || ""
        if (v && v.length > 0) playVoiceFromBase64(v)
        else playDefaultVoice()
    }

    function scheduleAutoplay(delayMs) {
        var d = (delayMs === undefined ? 120 : delayMs)
        if (StackView.status !== StackView.Active) {
            // Trang chưa Active: đợi Active rồi mới phát
            willAutoplayWhenActive = true
            return
        }
        // Trang đã Active: chờ 1 nhịp ngắn cho UI render rồi phát
        var t = Qt.createQmlObject('import QtQuick 2.15; Timer { interval: ' + d + '; repeat: false; }', root)
        t.triggered.connect(function() {
            playForCurrentStep()
            t.destroy()
        })
        t.start()
    }

    // Dùng thay cho gán trực tiếp currentStepIndex
    function setCurrentStepAndScheduleAutoplay(newIndex) {
        var prev = currentStepIndex
        currentStepIndex = newIndex
        // Nếu index không đổi (0->0), onCurrentStepIndexChanged sẽ không bắn
        // → vẫn lên lịch phát (sau khi trang Active)
        if (prev === newIndex) scheduleAutoplay()
    }

    // Khi trang thay đổi trạng thái trong StackView
    StackView.onStatusChanged: {
        if (StackView.status === StackView.Active && willAutoplayWhenActive) {
            willAutoplayWhenActive = false
            scheduleAutoplay()
        }
    }
    // -------------------------------------------

    // Audio Controller
    AudioController {
        id: voicePlayer
        
    }
    
    // ======= Audio helpers (giữ nguyên logic của bạn) =======
    function playVoiceFromBase64Optimized(voiceBase64) {
        console.log("CareStepsScreen: Attempting to play voice from base64 (optimized)")
        console.log("CareStepsScreen: Original base64 length:", voiceBase64 ? voiceBase64.length : 0)
        if (voiceBase64 && voiceBase64.length > 0) {
            console.log("CareStepsScreen: Using base64 data directly from server (optimized)")
            console.log("CareStepsScreen: Base64 length:", voiceBase64.length)
            if (voiceBase64.length < 100) { playDefaultVoice(); return }
            if (voicePlayer.loadFromBase64(voiceBase64)) voicePlayer.playAudio()
            else playDefaultVoice()
        } else {
            playDefaultVoice()
        }
    }

    function playVoiceFromBase64(voiceBase64) {
        console.log("CareStepsScreen: Attempting to play voice from base64")
        console.log("CareStepsScreen: Original base64 length:", voiceBase64 ? voiceBase64.length : 0)
        console.log("CareStepsScreen: Original base64 preview:", voiceBase64 ? voiceBase64.substring(0, 100) + "..." : "null")
        if (voiceBase64 && voiceBase64.length > 0) {
            console.log("CareStepsScreen: Base64 length:", voiceBase64.length)
            if (voiceBase64.length < 1000) { // điều chỉnh nếu TTS ngắn
                voiceStatus.text = "❌ Dữ liệu âm thanh không hợp lệ"
                voiceStatus.color = "#e74c3c"
                playDefaultVoice()
                return
            }
            if (voicePlayer.loadFromBase64(voiceBase64)) voicePlayer.playAudio()
            else { voiceStatus.text = "❌ Không thể phát âm thanh từ server"; voiceStatus.color = "#e74c3c"; playDefaultVoice() }
        } else {
            playDefaultVoice()
        }
    }
    
    function playDefaultVoice() {
        console.log("CareStepsScreen: Loading default voice from assets/voice_base.txt")
        loadVoiceFileWithRetry()
    }
    function loadVoiceFileWithRetry() {
        console.log("CareStepsScreen: Trying to load voice file with multiple methods...")
        if (fileHelper) {
            var base64Data = fileHelper.readVoiceBaseFile()
            if (base64Data && base64Data.length > 1000) {
                if (voicePlayer.loadFromBase64(base64Data)) { voicePlayer.playAudio(); return }
            }
        }
        if (fileHelper && fileHelper.resourceFileExists("qrc:/assets/voice_base.txt")) {
            var base64Data2 = fileHelper.readResourceFile("qrc:/assets/voice_base.txt")
            if (base64Data2 && base64Data2.length > 1000) {
                if (voicePlayer.loadFromBase64(base64Data2)) { voicePlayer.playAudio(); return }
            }
        }
        loadVoiceWithXMLHttpRequest()
    }
    function loadVoiceWithXMLHttpRequest() {
        var xhr = new XMLHttpRequest()
        xhr.open("GET", "qrc:/assets/voice_base.txt")
        xhr.onreadystatechange = function() {
            if (xhr.readyState === XMLHttpRequest.DONE) {
                if (xhr.status === 200) {
                    var base64Data = xhr.responseText.trim()
                    if (base64Data && base64Data.length > 1000) {
                        if (voicePlayer.loadFromBase64(base64Data)) { voicePlayer.playAudio(); return }
                    }
                }
                playDefaultVoiceFallback()
            }
        }
        xhr.send()
    }
    function playDefaultVoiceFallback() {
        var fallbackBase64 = "UklGRnoGAABXQVZFZm10IBAAAAABAAEAQB8AAEAfAAABAAgAZGF0YQoGAACBhYqFbF1fdJivrJBhNjVgodDbq2EcBj+a2/LDciUFLIHO8tiJNwgZaLvt559NEAxQp+PwtmMcBjiR1/LMeSwFJHfH8N2QQAoUXrTp66hVFApGn+DyvmwhBSuBzvLZiTYIG2m98OScTgwOUarm7blmGgU7k9n1unEiBC13yO/eizEIHWq+8+OWT"
        if (voicePlayer.loadFromBase64(fallbackBase64)) voicePlayer.playAudio()
        else { voiceStatus.text = "❌ Không thể phát âm thanh mặc định"; voiceStatus.color = "#e74c3c" }
    }
    // ========================================================

    function addStepToBuffer(stepNumber, stepDescription, imageBase64, voiceBase64) {
        let stepData = { stepNumber, stepDescription, imageBase64, voiceBase64: voiceBase64 || "", timestamp: Date.now() }
        if (stepNumber === 1) { stepBuffer = []; hasReceivedStep1 = true }
        stepBuffer.push(stepData)
        if (stepBufferTimer) stepBufferTimer.stop()
        stepBufferTimer = Qt.createQmlObject('import QtQuick 2.15; Timer { interval: 3000; repeat: false; }', root)
        stepBufferTimer.triggered.connect(function() { processStepBuffer() })
        stepBufferTimer.start()
    }
    
    function processStepBuffer() {
        if (stepBuffer.length === 0) return
        stepBuffer.sort(function(a, b) { return a.stepNumber - b.stepNumber })
        for (let i = 0; i < stepBuffer.length; i++) {
            let s = stepBuffer[i]
            addOrUpdateStep(s.stepNumber, s.stepDescription, s.imageBase64, s.voiceBase64)
        }
        stepBuffer = []
        if (careSteps.length > 0) {
            let step1Index = -1
            for (let i = 0; i < careSteps.length; i++) if (careSteps[i].stepNumber === 1) { step1Index = i; break }
            if (step1Index >= 0) setCurrentStepAndScheduleAutoplay(step1Index)
            else setCurrentStepAndScheduleAutoplay(0)
        }
        debugAllSteps()
    }

    function applyNextQueuedStep() {
        if (tssDataQueue.length > 0) {
            let s = tssDataQueue.shift()
            addOrUpdateStep(s.stepNumber, s.stepDescription, s.imageBase64, s.voiceBase64)
            hasQueuedData = tssDataQueue.length > 0
        }
    }

    function addOrUpdateStep(stepNumber, stepDescription, imageBase64, voiceBase64) {
        let existingIndex = -1
        for (let i = 0; i < careSteps.length; i++) if (careSteps[i].stepNumber === stepNumber) { existingIndex = i; break }
        let stepData = {
            stepNumber, title: "Bước " + stepNumber + ": " + stepDescription,
            description: stepDescription,
            image: imageBase64 && imageBase64.length > 0 ? "data:image/png;base64," + imageBase64 : "qrc:/assets/j97.jpg",
            imageBase64: imageBase64 || "", voiceBase64: voiceBase64 || ""
        }
        if (existingIndex >= 0) careSteps[existingIndex] = stepData
        else careSteps.push(stepData)
        careSteps.sort(function(a, b) { return a.stepNumber - b.stepNumber })
        let newCareSteps = []
        for (let i = 0; i < careSteps.length; i++) {
            let s = careSteps[i]
            newCareSteps.push({ stepNumber: s.stepNumber, title: s.title, description: s.description, image: s.image, imageBase64: s.imageBase64 || "", voiceBase64: s.voiceBase64 || "" })
        }
        careSteps = newCareSteps
        if (careSteps.length === 1) setCurrentStepAndScheduleAutoplay(0)
    }

    function navigateToStep(stepNumber) {
        for (let i = 0; i < careSteps.length; i++) {
            if (careSteps[i].stepNumber === stepNumber) {
                setCurrentStepAndScheduleAutoplay(i)
                console.log("CareStepsScreen: Navigated to step", stepNumber)
                return
            }
        }
        console.log("CareStepsScreen: Step", stepNumber, "not found")
    }

    function clearTSSQueue() { tssDataQueue = []; hasQueuedData = false }
    function applyTSSDataDirectly(stepNumber, stepDescription, imageBase64, voiceBase64) { addOrUpdateStep(stepNumber, stepDescription, imageBase64, voiceBase64) }

    function debugAllSteps() {
        console.log("=== DEBUG ALL STEPS ===")
        console.log("Total steps:", careSteps.length)
        console.log("Current step index:", currentStepIndex)
        for (let i = 0; i < careSteps.length; i++) {
            console.log("Step", i, ":", careSteps[i].stepNumber, "-", careSteps[i].title.substring(0, 50) + "...")
            console.log("  - voiceBase64 length:", careSteps[i].voiceBase64 ? careSteps[i].voiceBase64.length : 0)
        }
        console.log("=======================")
    }
    
    function processAllReceivedSteps() {
        if (allReceivedSteps.length === 0) return
        allReceivedSteps.sort(function(a, b) { return a.stepNumber - b.stepNumber })
        for (let i = 0; i < allReceivedSteps.length; i++) {
            let s = allReceivedSteps[i]
            addOrUpdateStep(s.stepNumber, s.stepDescription, s.imageBase64, s.voiceBase64)
        }
        if (careSteps.length > 0) {
            let step1Index = -1
            for (let i = 0; i < careSteps.length; i++) if (careSteps[i].stepNumber === 1) { step1Index = i; break }
            if (step1Index >= 0) setCurrentStepAndScheduleAutoplay(step1Index)
            else setCurrentStepAndScheduleAutoplay(0)
        }
        debugAllSteps()
    }
    
    // ===== UI (nguyên bản) =====
    Rectangle { anchors.fill: parent; color: "#f0f0f0" }

    Rectangle {
        id: header
        anchors.top: parent.top; anchors.left: parent.left; anchors.right: parent.right
        height: 60; color: "#2c3e50"
        Text { anchors.centerIn: parent; text: "📋 Hướng dẫn chăm sóc sức khỏe"; font.pixelSize: 18; font.bold: true; color: "#ffffff" }
        Text {
            anchors.right: parent.right; anchors.rightMargin: 20; anchors.verticalCenter: parent.verticalCenter
            text: careSteps.length > 0 ? "Bước " + (currentStepIndex + 1) + "/" + careSteps.length : "Chưa có bước nào"
            font.pixelSize: 14; color: "#ecf0f1"
        }
    }

    RowLayout {
        anchors.top: header.bottom; anchors.left: parent.left; anchors.right: parent.right; anchors.bottom: bottomControls.top
        anchors.margins: 20; spacing: 20

        // Left list
        Rectangle {
            Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: parent.width * 0.3
            color: "#ffffff"; radius: 10; border.color: "#bdc3c7"; border.width: 1
            ColumnLayout {
                anchors.fill: parent; anchors.margins: 10; spacing: 5
                Text { text: "📝 Danh sách các bước"; font.pixelSize: 16; font.bold: true; color: "#2c3e50"; Layout.fillWidth: true }
                ScrollView {
                    Layout.fillWidth: true; Layout.fillHeight: true; clip: true
                    ListView {
                        id: stepsListView; model: careSteps; spacing: 5
                        delegate: Rectangle {
                            width: stepsListView.width; height: 60
                            color: index === currentStepIndex ? "#3498db" : "#ecf0f1"
                            radius: 8; border.color: "#bdc3c7"; border.width: 1
                            RowLayout {
                                anchors.fill: parent; anchors.margins: 10; spacing: 10
                                Rectangle {
                                    width: 30; height: 30; radius: 15
                                    color: index === currentStepIndex ? "#ffffff" : "#3498db"
                                    Text {
                                        anchors.centerIn: parent; text: modelData.stepNumber
                                        font.pixelSize: 12; font.bold: true
                                        color: index === currentStepIndex ? "#3498db" : "#ffffff"
                                    }
                                }
                                Text {
                                    text: (modelData.description && modelData.description.length > 30)
                                          ? modelData.description.substring(0, 30) + "..."
                                          : (modelData.description || "")
                                    font.pixelSize: 12
                                    color: index === currentStepIndex ? "#ffffff" : "#2c3e50"
                                    wrapMode: Text.WordWrap
                                    Layout.fillWidth: true; Layout.fillHeight: true
                                    verticalAlignment: Text.AlignVCenter
                                }
                            }
                            MouseArea {
                                anchors.fill: parent
                                onClicked: {
                                    setCurrentStepAndScheduleAutoplay(index)
                                    console.log("CareStepsScreen: Step clicked - Navigated to Index:", index, "Step Number:", modelData.stepNumber)
                                }
                            }
                        }
                    }
                }
            }
        }

        // Right detail
        Rectangle {
            Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: parent.width * 0.7
            color: "#ffffff"; radius: 10; border.color: "#bdc3c7"; border.width: 1
            ColumnLayout {
                anchors.fill: parent; anchors.margins: 20; spacing: 15
                Text {
                    text: careSteps.length > 0 ? careSteps[currentStepIndex].title : "Chưa có bước nào"
                    font.pixelSize: 18; font.bold: true; color: "#2c3e50"; wrapMode: Text.WordWrap; Layout.fillWidth: true
                }
                Rectangle {
                    Layout.fillWidth: true; Layout.preferredHeight: 200; color: "#f8f9fa"; radius: 8; border.color: "#dee2e6"; border.width: 1
                    Image {
                        anchors.fill: parent; anchors.margins: 10
                        source: careSteps.length > 0 ? careSteps[currentStepIndex].image : ""
                        fillMode: Image.PreserveAspectFit; smooth: true; antialiasing: true
                    }
                }
                Text {
                    text: careSteps.length > 0 ? careSteps[currentStepIndex].description : "Vui lòng chờ dữ liệu từ hệ thống..."
                    font.pixelSize: 14; color: "#34495e"; wrapMode: Text.WordWrap; lineHeight: 1.4
                    Layout.fillWidth: true; Layout.fillHeight: true
                }
            }
        }
    }

    // Bottom controls
    Rectangle {
        id: bottomControls
        anchors.bottom: parent.bottom; anchors.left: parent.left; anchors.right: parent.right
        height: 120; color: "#ecf0f1"; border.color: "#bdc3c7"; border.width: 1

        ColumnLayout {
            anchors.fill: parent; anchors.margins: 20; spacing: 10
            RowLayout {
                Layout.fillWidth: true; spacing: 10

                Button {
                    Layout.fillWidth: true; Layout.preferredHeight: 40
                    text: "⬅️ Quay lại"
                    enabled: currentStepIndex > 0 && careSteps.length > 0
                    background: Rectangle { color: parent.enabled ? (parent.pressed ? "#95a5a6" : "#7f8c8d") : "#bdc3c7"; radius: 8 }
                    contentItem: Text { text: parent.text; color: parent.enabled ? "#ffffff" : "#7f8c8d"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter; font.pixelSize: 14; font.bold: true }
                    onClicked: {
                        if (currentStepIndex > 0) setCurrentStepAndScheduleAutoplay(currentStepIndex - 1)
                    }
                }

                Button {
                    Layout.fillWidth: true; Layout.preferredHeight: 40
                    text: careSteps.length > 0 && currentStepIndex < careSteps.length - 1 ? "Tiếp theo ➡️" : "Hoàn thành ✅"
                    background: Rectangle { color: parent.pressed ? "#27ae60" : "#2ecc71"; radius: 8 }
                    contentItem: Text { text: parent.text; color: "#ffffff"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter; font.pixelSize: 14; font.bold: true }
                    onClicked: {
                        if (hasQueuedData) applyNextQueuedStep()
                        if (careSteps.length > 0 && currentStepIndex < careSteps.length - 1) setCurrentStepAndScheduleAutoplay(currentStepIndex + 1)
                        else if (stackView) stackView.pop()
                    }
                }


                Button {
                    Layout.fillWidth: true; Layout.preferredHeight: 40
                    text: "🏠 Trang chủ"
                    background: Rectangle { color: parent.pressed ? "#e67e22" : "#f39c12"; radius: 8 }
                    contentItem: Text { text: parent.text; color: "#ffffff"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter; font.pixelSize: 14; font.bold: true }
                    onClicked: { if (stackView) stackView.pop(stackView.get(0)) }
                }

            }
        }
    }

    // Progress bar
    Rectangle {
        anchors.top: header.bottom; anchors.left: parent.left; anchors.right: parent.right
        height: 4; color: "#ecf0f1"
        Rectangle {
            width: careSteps.length > 0 ? parent.width * ((currentStepIndex + 1) / careSteps.length) : 0
            height: parent.height; color: "#3498db"
            Behavior on width { NumberAnimation { duration: 300 } }
        }
    }

    // TSS Data Indicator
    Rectangle {
        id: tssDataIndicator
        anchors.top: parent.top; anchors.right: parent.right; anchors.margins: 10; anchors.topMargin: 45
        width: 120; height: 25; color: "#2196F3"; radius: 12; opacity: 0; visible: false
        Text { anchors.centerIn: parent; text: "📡 TSS Data Received"; color: "white"; font.pixelSize: 9; font.bold: true }
        SequentialAnimation on opacity {
            id: showAnimation; running: false
            NumberAnimation { to: 1.0; duration: 300 }
            PauseAnimation { duration: 2000 }
            NumberAnimation { to: 0.0; duration: 300 }
            onFinished: { tssDataIndicator.visible = false }
        }
    }

    // TSS Connections
    Connections {
        target: tssSocketBridge
        function onStepDataChanged() {
            var processTimer = Qt.createQmlObject('import QtQuick 2.15; Timer { interval: 100; repeat: false; }', root)
            processTimer.triggered.connect(function() {
                tssDataIndicator.visible = true; showAnimation.start()
                let s = {
                    stepNumber: tssSocketBridge.currentStepNumber,
                    stepDescription: tssSocketBridge.currentStepDescription,
                    imageBase64: tssSocketBridge.currentImageBase64 || "",
                    voiceBase64: tssSocketBridge.currentAudioBase64 || "",
                    timestamp: Date.now()
                }
                let existingIndex = -1
                for (let i = 0; i < allReceivedSteps.length; i++) if (allReceivedSteps[i].stepNumber === s.stepNumber) { existingIndex = i; break }
                if (existingIndex >= 0) allReceivedSteps[existingIndex] = s
                else allReceivedSteps.push(s)

                addStepToBuffer(s.stepNumber, s.stepDescription, s.imageBase64, s.voiceBase64)

                if (tssSocketBridge.currentStepNumber === 1) {
                    if (stepBufferTimer) stepBufferTimer.stop()
                    processAllReceivedSteps()   // sẽ set index + scheduleAutoplay()
                }
                processTimer.destroy()
            })
            processTimer.start()
        }
        function onLogMessage(message) { console.log("CareStepsScreen TSS Log:", message) }
        function onMessageReceived(message) { console.log("CareStepsScreen TSS Message:", message) }
    }

    // Kết nối TSS khi màn hình load
    Component.onCompleted: {
        console.log("CareStepsScreen: Component completed, connecting to TSS server")
        if (tssSocketBridge && !tssSocketBridge.isConnected) tssSocketBridge.connectToServer()
        
        if (stackView && stackView.get(0) && stackView.get(0).globalAllSteps && stackView.get(0).globalAllSteps.length > 0) {
            allReceivedSteps = []
            for (let i = 0; i < stackView.get(0).globalAllSteps.length; i++) {
                let g = stackView.get(0).globalAllSteps[i]
                allReceivedSteps.push({ stepNumber: g.stepNumber, stepDescription: g.stepDescription, imageBase64: g.imageBase64 || "", voiceBase64: g.voiceBase64 || "", timestamp: g.timestamp || Date.now() })
            }
            processAllReceivedSteps() // set index + scheduleAutoplay()
        } else if (tssSocketBridge && tssSocketBridge.currentStepNumber > 0) {
            allReceivedSteps.push({
                stepNumber: tssSocketBridge.currentStepNumber,
                stepDescription: tssSocketBridge.currentStepDescription,
                imageBase64: tssSocketBridge.currentImageBase64 || "",
                voiceBase64: tssSocketBridge.currentAudioBase64 || "",
                timestamp: Date.now()
            })
            if (tssSocketBridge.currentStepNumber === 1) processAllReceivedSteps()
            else {
                addOrUpdateStep(1, "Đang chờ dữ liệu cho bước 1...", "")
                setCurrentStepAndScheduleAutoplay(0)
            }
        } else {
            addOrUpdateStep(1, "Đang chờ dữ liệu cho bước 1...", "")
            setCurrentStepAndScheduleAutoplay(0)
        }
    }

    // Khi index đổi do người dùng bấm Back/Next/List → chỉ LÊN LỊCH phát
    onCurrentStepIndexChanged: {
        console.log("CareStepsScreen: Current step index changed to:", currentStepIndex)
        if (careSteps.length > 0 && currentStepIndex >= 0 && currentStepIndex < careSteps.length) {
            scheduleAutoplay()
        }
    }
}
