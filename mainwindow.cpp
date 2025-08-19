#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QApplication>
#include <QStyle>
#include <QScreen>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , networkManager(new QNetworkAccessManager(this))
    , audioManager(new AudioManager(this))
{
    ui->setupUi(this);

    // Set window properties
    setWindowTitle("Robot Face Interface");
    setMinimumSize(800, 600);

    // Center window on screen
    QScreen *screen = QApplication::primaryScreen();
    QRect screenGeometry = screen->geometry();
    int x = (screenGeometry.width() - width()) / 2;
    int y = (screenGeometry.height() - height()) / 2;
    move(x, y);

    // Create central widget and stacked widget
    centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);

    stackedWidget = new QStackedWidget(centralWidget);
    QVBoxLayout *mainLayout = new QVBoxLayout(centralWidget);
    mainLayout->addWidget(stackedWidget);
    mainLayout->setContentsMargins(0, 0, 0, 0);

    // Setup screens
    setupMainScreen();
    setupResponseScreen();
    setupImageScreen();

    // Connect network signals
    connect(networkManager, &QNetworkAccessManager::finished,
            this, &MainWindow::onNetworkReplyFinished);

    // Connect audio signals
    connect(audioManager, &AudioManager::errorOccurred, this, &MainWindow::onAudioError);
    connect(audioManager, &AudioManager::captureStarted, this, &MainWindow::onAudioCaptureStarted);
    connect(audioManager, &AudioManager::captureStopped, this, &MainWindow::onAudioCaptureStopped);
    connect(audioManager, &AudioManager::audioPlayed, this, &MainWindow::onAudioPlayed);

    // Show main screen
    stackedWidget->setCurrentWidget(mainScreen);

    // Set dark theme
    setStyleSheet(R"(
        QMainWindow {
            background-color: #1a1a1a;
            color: #ffffff;
        }
        QWidget {
            background-color: #1a1a1a;
            color: #ffffff;
        }
        QTextEdit, QLineEdit {
            background-color: #3a3a3a;
            border: 1px solid #555555;
            border-radius: 5px;
            color: #ffffff;
            padding: 5px;
        }
        QPushButton {
            background-color: #333333;
            border: 1px solid #555555;
            border-radius: 5px;
            color: #ffffff;
            padding: 8px 16px;
            font-weight: bold;
        }
        QPushButton:hover {
            background-color: #444444;
        }
        QPushButton:pressed {
            background-color: #222222;
        }
        QPushButton#resetButton {
            background-color: #cc0000;
        }
        QPushButton#resetButton:hover {
            background-color: #dd0000;
        }
        QPushButton#resetButton:pressed {
            background-color: #aa0000;
        }
        QPushButton#audioStartButton {
            background-color: #006600;
        }
        QPushButton#audioStartButton:hover {
            background-color: #007700;
        }
        QPushButton#audioStopButton {
            background-color: #660000;
        }
        QPushButton#audioStopButton:hover {
            background-color: #770000;
        }
        QPushButton#audioPlayButton {
            background-color: #000066;
        }
        QPushButton#audioPlayButton:hover {
            background-color: #000077;
        }
        QLabel {
            color: #ffffff;
        }
    )");
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setupMainScreen()
{
    mainScreen = new QWidget();
    QVBoxLayout *layout = new QVBoxLayout(mainScreen);
    layout->setSpacing(10);
    layout->setContentsMargins(20, 20, 20, 20);

    // Title
    QLabel *titleLabel = new QLabel("🤖 Robot Face Interface");
    titleLabel->setAlignment(Qt::AlignCenter);
    titleLabel->setStyleSheet("font-size: 18px; font-weight: bold; margin-bottom: 10px;");
    layout->addWidget(titleLabel);

    // Video widget
    videoWidget = new QVideoWidget();
    videoWidget->setMinimumHeight(300);
    videoWidget->setStyleSheet("border: 1px solid #ffffff; border-radius: 3px;");
    layout->addWidget(videoWidget);

    // Setup media player
    mediaPlayer = new QMediaPlayer();
    mediaPlayer->setVideoOutput(videoWidget);
    mediaPlayer->setSource(QUrl::fromLocalFile("assets/blinking_face.mp4"));
    mediaPlayer->setLoops(QMediaPlayer::Infinite);
    mediaPlayer->play();

    // Audio controls
    QHBoxLayout *audioLayout = new QHBoxLayout();

    audioStartButton = new QPushButton("🎤 Start Capture");
    audioStartButton->setObjectName("audioStartButton");
    audioStartButton->setMinimumHeight(30);
    connect(audioStartButton, &QPushButton::clicked, this, &MainWindow::onStartAudioCapture);

    audioStopButton = new QPushButton("⏹️ Stop Capture");
    audioStopButton->setObjectName("audioStopButton");
    audioStopButton->setMinimumHeight(30);
    audioStopButton->setEnabled(false);
    connect(audioStopButton, &QPushButton::clicked, this, &MainWindow::onStopAudioCapture);

    audioPlayButton = new QPushButton("▶️ Play Audio");
    audioPlayButton->setObjectName("audioPlayButton");
    audioPlayButton->setMinimumHeight(30);
    connect(audioPlayButton, &QPushButton::clicked, this, &MainWindow::onPlayAudio);

    audioLayout->addWidget(audioStartButton);
    audioLayout->addWidget(audioStopButton);
    audioLayout->addWidget(audioPlayButton);
    layout->addLayout(audioLayout);

    // Input area
    QHBoxLayout *inputLayout = new QHBoxLayout();
    messageInput = new QLineEdit();
    messageInput->setPlaceholderText("Type your message here...");
    messageInput->setMinimumHeight(40);
    connect(messageInput, &QLineEdit::returnPressed, this, &MainWindow::onSendMessage);

    sendButton = new QPushButton("Send");
    sendButton->setMinimumWidth(80);
    sendButton->setMinimumHeight(40);
    connect(sendButton, &QPushButton::clicked, this, &MainWindow::onSendMessage);

    inputLayout->addWidget(messageInput);
    inputLayout->addWidget(sendButton);
    layout->addLayout(inputLayout);

    // Reset button
    resetButton = new QPushButton("Reset Memory");
    resetButton->setObjectName("resetButton");
    resetButton->setMinimumHeight(30);
    connect(resetButton, &QPushButton::clicked, this, &MainWindow::onResetMemory);
    layout->addWidget(resetButton);

    // Image button
    imageButton = new QPushButton("📷 Open Image");
    imageButton->setMinimumHeight(30);
    connect(imageButton, &QPushButton::clicked, this, &MainWindow::onOpenImage);
    layout->addWidget(imageButton);

    // Status label
    statusLabel = new QTextEdit();
    statusLabel->setMaximumHeight(60);
    statusLabel->setReadOnly(true);
    statusLabel->setText("Ready to chat with robot...");
    statusLabel->setStyleSheet("background-color: #2a2a2a; border: none;");
    layout->addWidget(statusLabel);

    stackedWidget->addWidget(mainScreen);
}

void MainWindow::setupResponseScreen()
{
    responseScreen = new QWidget();
    QVBoxLayout *layout = new QVBoxLayout(responseScreen);
    layout->setSpacing(20);
    layout->setContentsMargins(20, 20, 20, 20);

    // Header
    QHBoxLayout *headerLayout = new QHBoxLayout();
    backButton = new QPushButton("← Back");
    backButton->setMinimumWidth(80);
    backButton->setMinimumHeight(40);
    connect(backButton, &QPushButton::clicked, this, &MainWindow::onBackToMain);

    QLabel *responseTitle = new QLabel("Robot Response");
    responseTitle->setAlignment(Qt::AlignCenter);
    responseTitle->setStyleSheet("font-size: 24px; font-weight: bold;");

    headerLayout->addWidget(backButton);
    headerLayout->addWidget(responseTitle);
    headerLayout->addStretch();
    layout->addLayout(headerLayout);

    // Response text
    responseText = new QTextEdit();
    responseText->setReadOnly(true);
    responseText->setStyleSheet("background-color: #2a2a2a; border: 1px solid #555555; border-radius: 5px;");
    layout->addWidget(responseText);

    stackedWidget->addWidget(responseScreen);
}

void MainWindow::setupImageScreen()
{
    imageScreen = new QWidget();
    QVBoxLayout *layout = new QVBoxLayout(imageScreen);
    layout->setSpacing(10);
    layout->setContentsMargins(10, 10, 10, 10);

    // Header
    QHBoxLayout *headerLayout = new QHBoxLayout();
    imageBackButton = new QPushButton("← Back");
    imageBackButton->setMinimumWidth(80);
    imageBackButton->setMinimumHeight(30);
    connect(imageBackButton, &QPushButton::clicked, this, &MainWindow::onBackToMain);

    QLabel *imageTitle = new QLabel("📷 Image Viewer");
    imageTitle->setAlignment(Qt::AlignCenter);
    imageTitle->setStyleSheet("font-size: 18px; font-weight: bold;");

    headerLayout->addWidget(imageBackButton);
    headerLayout->addWidget(imageTitle);
    headerLayout->addStretch();
    layout->addLayout(headerLayout);

    // Image label
    imageLabel = new QLabel();
    imageLabel->setAlignment(Qt::AlignCenter);
    imageLabel->setStyleSheet("border: 2px solid #ffffff; border-radius: 8px; background-color: #000000;");
    imageLabel->setMinimumHeight(400);

    // Load image
    QPixmap pixmap("assets/j97.jpg");
    if (!pixmap.isNull()) {
        pixmap = pixmap.scaled(imageLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
        imageLabel->setPixmap(pixmap);
    } else {
        imageLabel->setText("Không thể tải ảnh\nVui lòng kiểm tra đường dẫn file");
        imageLabel->setStyleSheet("border: 2px solid #ffffff; border-radius: 8px; background-color: #000000; color: #ffffff; font-size: 16px;");
    }

    layout->addWidget(imageLabel);

    // Image info
    QLabel *imageInfo = new QLabel("File: j97.jpg");
    imageInfo->setAlignment(Qt::AlignCenter);
    imageInfo->setStyleSheet("color: #888888; font-size: 12px;");
    layout->addWidget(imageInfo);

    stackedWidget->addWidget(imageScreen);
}

void MainWindow::onSendMessage()
{
    QString message = messageInput->text().trimmed();
    if (message.isEmpty()) {
        return;
    }

    updateStatus("Sending message...");

    QJsonObject data;
    data["message"] = message;

    sendNetworkRequest("/qa_bot/send_message", data);
    messageInput->clear();
}

void MainWindow::onResetMemory()
{
    updateStatus("Resetting memory...");

    QJsonObject data;
    sendNetworkRequest("/qa_bot/reset_memory", data);
}

void MainWindow::onOpenImage()
{
    stackedWidget->setCurrentWidget(imageScreen);
}

void MainWindow::onBackToMain()
{
    stackedWidget->setCurrentWidget(mainScreen);
}

void MainWindow::sendNetworkRequest(const QString &endpoint, const QJsonObject &data)
{
    QNetworkRequest request(QUrl("http://192.168.1.122:8989" + endpoint));
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");

    QJsonDocument doc(data);
    QByteArray jsonData = doc.toJson();

    networkManager->post(request, jsonData);
}

void MainWindow::onNetworkReplyFinished(QNetworkReply *reply)
{
    reply->deleteLater();

    if (reply->error() == QNetworkReply::NoError) {
        QString response = QString::fromUtf8(reply->readAll());

        // Try to parse JSON response
        QJsonDocument doc = QJsonDocument::fromJson(response.toUtf8());
        if (!doc.isNull() && doc.isObject()) {
            QJsonObject obj = doc.object();
            if (obj.contains("response")) {
                response = obj["response"].toString();
            } else if (obj.contains("message")) {
                response = obj["message"].toString();
            }
        }

        // Check if this was a reset memory request
        QString url = reply->url().toString();
        if (url.contains("reset_memory")) {
            updateStatus("Memory reset successfully!");
        } else {
            showResponse(response);
        }
    } else {
        QString errorText = "Error: " + QString::number(reply->error()) + " - " + reply->errorString();
        if (reply->url().toString().contains("reset_memory")) {
            updateStatus("Error resetting memory: " + QString::number(reply->error()));
        } else {
            showResponse(errorText);
        }
    }
}

void MainWindow::onNetworkError(QNetworkReply::NetworkError error)
{
    QString errorText = "Network error occurred";
    if (error == QNetworkReply::ConnectionRefusedError) {
        errorText = "Connection refused. Please check if the server is running.";
    }
    showResponse(errorText);
}

void MainWindow::showResponse(const QString &response)
{
    responseText->setText(response);
    stackedWidget->setCurrentWidget(responseScreen);
}

void MainWindow::updateStatus(const QString &status)
{
    statusLabel->setText(status);
}

// Audio-related functions
void MainWindow::onStartAudioCapture()
{
    if (audioManager->startCapture()) {
        updateStatus("Starting audio capture...");
    }
}

void MainWindow::onStopAudioCapture()
{
    audioManager->stopCapture();
    updateStatus("Stopping audio capture...");
}

void MainWindow::onPlayAudio()
{
    if (audioManager->playAudio()) {
        updateStatus("Playing audio...");
    }
}

void MainWindow::onAudioError(const QString &message)
{
    updateStatus("Audio Error: " + message);
    QMessageBox::warning(this, "Audio Error", message);
}

void MainWindow::onAudioCaptureStarted()
{
    updateAudioUI();
    updateStatus("Audio capture is now active");
}

void MainWindow::onAudioCaptureStopped()
{
    updateAudioUI();
    updateStatus("Audio capture stopped");
}

void MainWindow::onAudioPlayed()
{
    updateStatus("Audio playback completed");
}

void MainWindow::updateAudioUI()
{
    bool isCapturing = audioManager->isCapturing();
    audioStartButton->setEnabled(!isCapturing);
    audioStopButton->setEnabled(isCapturing);

    if (isCapturing) {
        audioStartButton->setText("🎤 Recording...");
        audioStartButton->setStyleSheet("background-color: #ff6600;");
    } else {
        audioStartButton->setText("🎤 Start Capture");
        audioStartButton->setStyleSheet("");
    }
}
