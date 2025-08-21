#include "tsssocketbridge.h"
#include <QDebug>
#include <QUrl>
#include <QJsonObject>
#include <QJsonDocument>
#include <QJsonParseError>

TSSSocketBridge::TSSSocketBridge(QObject *parent)
    : QObject(parent)
    , m_networkManager(new QNetworkAccessManager(this))
    , m_currentStepNumber(0)
{
    // Connect WebSocket signals
    connect(&m_webSocket, &QWebSocket::connected, this, &TSSSocketBridge::onWebSocketConnected);
    connect(&m_webSocket, &QWebSocket::disconnected, this, &TSSSocketBridge::onWebSocketDisconnected);
    connect(&m_webSocket, &QWebSocket::textMessageReceived, this, &TSSSocketBridge::onWebSocketTextMessageReceived);
    connect(&m_webSocket, static_cast<void(QWebSocket::*)(QAbstractSocket::SocketError)>(&QWebSocket::error), this, &TSSSocketBridge::onWebSocketError);
}

TSSSocketBridge::~TSSSocketBridge()
{
    if (m_webSocket.state() == QAbstractSocket::ConnectedState) {
        m_webSocket.close();
    }
}

void TSSSocketBridge::connectToServer(const QString &url)
{
    emit logMessage("Connecting to TSS WebSocket server: " + url);
    m_webSocket.open(QUrl(url));
}

void TSSSocketBridge::disconnectFromServer()
{
    emit logMessage("Disconnecting from TSS WebSocket server");
    m_webSocket.close();
}

void TSSSocketBridge::sendMessage(const QString &message)
{
    if (m_webSocket.state() == QAbstractSocket::ConnectedState) {
        m_webSocket.sendTextMessage(message);
        emit logMessage("Sent: " + message);
    } else {
        emit logMessage("Error: Not connected to TSS WebSocket server");
    }
}

void TSSSocketBridge::sendStepRequest(int stepNumber, const QString &stepDescription)
{
    if (m_webSocket.state() == QAbstractSocket::ConnectedState) {
        QJsonObject requestData;
        requestData["step_number"] = stepNumber;
        requestData["step_description"] = stepDescription;
        
        QJsonDocument doc(requestData);
        QString jsonString = doc.toJson(QJsonDocument::Compact);
        
        m_webSocket.sendTextMessage(jsonString);
        emit logMessage("Sent step request: " + jsonString);
    } else {
        emit logMessage("Error: Not connected to TSS WebSocket server");
    }
}



void TSSSocketBridge::onWebSocketConnected()
{
    emit logMessage("✅ TSS WebSocket connected!");
    emit connectionStatusChanged();
}

void TSSSocketBridge::onWebSocketDisconnected()
{
    emit logMessage("❌ TSS WebSocket disconnected");
    emit connectionStatusChanged();
    qDebug() << "TSS WebSocket disconnected, reason:" << m_webSocket.closeReason();
}

void TSSSocketBridge::onWebSocketTextMessageReceived(const QString &message)
{
    m_lastMessage = message;
    // emit logMessage("Received: " + message);
    emit messageReceived(message);
    
    // Try to parse JSON response
    QJsonParseError parseError;
    QJsonDocument jsonDoc = QJsonDocument::fromJson(message.toUtf8(), &parseError);
    
    if (parseError.error == QJsonParseError::NoError && jsonDoc.isObject()) {
        QJsonObject jsonObj = jsonDoc.object();
        
        // Check if this is a structured step response
        if (jsonObj.contains("step_number") && jsonObj.contains("step_description")) {
            m_currentStepNumber = jsonObj["step_number"].toInt();
            m_currentStepDescription = jsonObj["step_description"].toString();
            
            // Parse image data - check both possible keys
            if (jsonObj.contains("image")) {
                m_currentImageBase64 = jsonObj["image"].toString();
            } else if (jsonObj.contains("img_base64")) {
                m_currentImageBase64 = jsonObj["img_base64"].toString();
            } else {
                m_currentImageBase64 = "";
            }
            
            // Parse audio data - check both possible keys
            if (jsonObj.contains("audio")) {
                m_currentAudioBase64 = jsonObj["audio"].toString();
            } else if (jsonObj.contains("audio_base64")) {
                m_currentAudioBase64 = jsonObj["audio_base64"].toString();
            } else {
                m_currentAudioBase64 = "";
            }
            
            // qDebug() << "TSSSocketBridge: Parsed step data:";
            // qDebug() << "  Step Number:" << m_currentStepNumber;
            // qDebug() << "  Step Description:" << m_currentStepDescription;
            // qDebug() << "  Available keys:" << jsonObj.keys();
            // qDebug() << "  Image key used:" << (jsonObj.contains("image") ? "image" : (jsonObj.contains("img_base64") ? "img_base64" : "none"));
            // qDebug() << "  Audio key used:" << (jsonObj.contains("audio") ? "audio" : (jsonObj.contains("audio_base64") ? "audio_base64" : "none"));
            // qDebug() << "  Image Base64 Length:" << m_currentImageBase64.length();
            // qDebug() << "  Image Base64 Preview:" << m_currentImageBase64.left(50) + "...";
            // qDebug() << "  Audio Base64 Length:" << m_currentAudioBase64.length();
            // qDebug() << "  Audio Base64 Preview:" << m_currentAudioBase64.left(50) + "...";
            
            emit logMessage("Parsed step data - Step " + QString::number(m_currentStepNumber) + 
                          ": " + m_currentStepDescription);
            emit stepDataChanged();
        } else if (jsonObj.contains("error")) {
            emit logMessage("Server error: " + jsonObj["error"].toString());
        }
    } else {
        emit logMessage("Received non-JSON message or parse error: " + parseError.errorString());
    }
}

void TSSSocketBridge::onWebSocketError(QAbstractSocket::SocketError error)
{
    QString errorMsg = "TSS WebSocket error: " + m_webSocket.errorString();
    emit logMessage(errorMsg);
    qDebug() << errorMsg;
}
