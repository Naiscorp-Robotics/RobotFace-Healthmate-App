#include "websocketbridge.h"
#include <QDebug>
#include <QUrl>
#include <QTimer>

WebSocketBridge::WebSocketBridge(QObject *parent)
    : QObject(parent)
    , m_networkManager(new QNetworkAccessManager(this))
    , m_autoConnect(true)
    , m_serverUrl("ws://localhost:12345")
{
    // Connect WebSocket signals
    connect(&m_webSocket, &QWebSocket::connected, this, &WebSocketBridge::onWebSocketConnected);
    connect(&m_webSocket, &QWebSocket::disconnected, this, &WebSocketBridge::onWebSocketDisconnected);
    connect(&m_webSocket, &QWebSocket::textMessageReceived, this, &WebSocketBridge::onWebSocketTextMessageReceived);
    connect(&m_webSocket, &QWebSocket::errorOccurred, this, &WebSocketBridge::onWebSocketError);
    
    // Auto-connect after a short delay to ensure the application is fully initialized
    if (m_autoConnect) {
        qDebug() << "WebSocketBridge: Auto-connect enabled, will connect in 1 second";
        QTimer::singleShot(1000, this, &WebSocketBridge::autoConnectToServer);
    } else {
        qDebug() << "WebSocketBridge: Auto-connect disabled";
    }
}

WebSocketBridge::~WebSocketBridge()
{
    if (m_webSocket.state() == QAbstractSocket::ConnectedState) {
        m_webSocket.close();
    }
}

void WebSocketBridge::connectToServer(const QString &url)
{
    if (!url.isEmpty()) {
        m_serverUrl = url;
        emit serverUrlChanged();
    }
    
    emit logMessage("🔗 Connecting to WebSocket server: " + m_serverUrl);
    emit logMessage("Connection attempt started...");
    m_webSocket.open(QUrl(m_serverUrl));
}

void WebSocketBridge::disconnectFromServer()
{
    emit logMessage("🔌 Disconnecting from WebSocket server");
    emit logMessage("Server: " + m_serverUrl);
    m_webSocket.close();
}

void WebSocketBridge::sendMessage(const QString &message)
{
    if (m_webSocket.state() == QAbstractSocket::ConnectedState) {
        m_webSocket.sendTextMessage(message);
        emit logMessage("📤 Sent message: " + message);
        emit logMessage("Message length: " + QString::number(message.length()) + " characters");
    } else {
        emit logMessage("❌ Error: Not connected to WebSocket server");
        emit logMessage("Current state: " + QString::number(m_webSocket.state()));
    }
}

void WebSocketBridge::autoConnectToServer()
{
    qDebug() << "WebSocketBridge::autoConnectToServer called";
    qDebug() << "Auto-connect enabled:" << m_autoConnect;
    qDebug() << "Currently connected:" << isConnected();
    
    if (m_autoConnect && !isConnected()) {
        emit logMessage("🔄 Auto-connecting to WebSocket server...");
        emit logMessage("Target server: " + m_serverUrl);
        connectToServer();
    } else {
        qDebug() << "Auto-connect skipped - autoConnect:" << m_autoConnect << "isConnected:" << isConnected();
    }
}

void WebSocketBridge::setAutoConnect(bool autoConnect)
{
    if (m_autoConnect != autoConnect) {
        m_autoConnect = autoConnect;
        emit autoConnectChanged();
        
        if (m_autoConnect && !isConnected()) {
            QTimer::singleShot(1000, this, &WebSocketBridge::autoConnectToServer);
        }
    }
}

void WebSocketBridge::setServerUrl(const QString &url)
{
    if (m_serverUrl != url) {
        m_serverUrl = url;
        emit serverUrlChanged();
    }
}

void WebSocketBridge::onWebSocketConnected()
{
    emit logMessage("✅ WebSocket connected!");
    emit logMessage("WebSocket server: " + m_serverUrl);
    emit connectionStatusChanged();
}

void WebSocketBridge::onWebSocketDisconnected()
{
    emit logMessage("❌ WebSocket disconnected");
    emit logMessage("Disconnect reason: " + m_webSocket.closeReason());
    emit connectionStatusChanged();
    qDebug() << "WebSocket disconnected, reason:" << m_webSocket.closeReason();
    
    // Auto-reconnect if autoConnect is enabled
    if (m_autoConnect) {
        emit logMessage("Attempting to reconnect in 3 seconds...");
        QTimer::singleShot(3000, this, &WebSocketBridge::autoConnectToServer);
    }
}

void WebSocketBridge::onWebSocketTextMessageReceived(const QString &message)
{
    m_lastMessage = message;
    emit logMessage("📥 Received message: " + message);
    emit logMessage("Message length: " + QString::number(message.length()) + " characters");
    emit messageReceived(message);
}

void WebSocketBridge::onWebSocketError(QAbstractSocket::SocketError error)
{
    QString errorMsg = "WebSocket error: " + m_webSocket.errorString();
    emit logMessage(errorMsg);
    emit logMessage("Error code: " + QString::number(error));
    qDebug() << errorMsg;
}


