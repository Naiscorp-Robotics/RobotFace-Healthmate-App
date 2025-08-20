#include "websocketbridge.h"
#include <QDebug>
#include <QUrl>

WebSocketBridge::WebSocketBridge(QObject *parent)
    : QObject(parent)
    , m_networkManager(new QNetworkAccessManager(this))
{
    // Connect WebSocket signals
    connect(&m_webSocket, &QWebSocket::connected, this, &WebSocketBridge::onWebSocketConnected);
    connect(&m_webSocket, &QWebSocket::disconnected, this, &WebSocketBridge::onWebSocketDisconnected);
    connect(&m_webSocket, &QWebSocket::textMessageReceived, this, &WebSocketBridge::onWebSocketTextMessageReceived);
    connect(&m_webSocket, &QWebSocket::errorOccurred, this, &WebSocketBridge::onWebSocketError);
}

WebSocketBridge::~WebSocketBridge()
{
    if (m_webSocket.state() == QAbstractSocket::ConnectedState) {
        m_webSocket.close();
    }
}

void WebSocketBridge::connectToServer(const QString &url)
{
    emit logMessage("Connecting to WebSocket server: " + url);
    m_webSocket.open(QUrl(url));
}

void WebSocketBridge::disconnectFromServer()
{
    emit logMessage("Disconnecting from WebSocket server");
    m_webSocket.close();
}

void WebSocketBridge::sendMessage(const QString &message)
{
    if (m_webSocket.state() == QAbstractSocket::ConnectedState) {
        m_webSocket.sendTextMessage(message);
        emit logMessage("Sent: " + message);
    } else {
        emit logMessage("Error: Not connected to WebSocket server");
    }
}



void WebSocketBridge::onWebSocketConnected()
{
    emit logMessage("✅ WebSocket connected!");
    emit connectionStatusChanged();
}

void WebSocketBridge::onWebSocketDisconnected()
{
    emit logMessage("❌ WebSocket disconnected");
    emit connectionStatusChanged();
    qDebug() << "WebSocket disconnected, reason:" << m_webSocket.closeReason();
}

void WebSocketBridge::onWebSocketTextMessageReceived(const QString &message)
{
    m_lastMessage = message;
    emit logMessage("Received: " + message);
    emit messageReceived(message);
}

void WebSocketBridge::onWebSocketError(QAbstractSocket::SocketError error)
{
    QString errorMsg = "WebSocket error: " + m_webSocket.errorString();
    emit logMessage(errorMsg);
    qDebug() << errorMsg;
}
