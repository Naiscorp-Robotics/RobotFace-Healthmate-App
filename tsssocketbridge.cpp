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
    connect(&m_webSocket, &QWebSocket::errorOccurred, this, &TSSSocketBridge::onWebSocketError);
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
    emit logMessage("Received: " + message);
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
            m_currentImageBase64 = jsonObj["img_base64"].toString();
            m_currentAudioBase64 = jsonObj["audio_base64"].toString();
            
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
