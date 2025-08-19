// signalingclient.cpp
#include "signalingclient.h"
#include <QJsonDocument>

SignalingClient::SignalingClient(const QUrl &url) {
    connect(&ws, &QWebSocket::connected, this, &SignalingClient::onConnected);
    ws.open(url);
}

void SignalingClient::onConnected() {
    connect(&ws, &QWebSocket::textMessageReceived,
            this, &SignalingClient::onTextMessage);
}

void SignalingClient::onTextMessage(const QString &message) {
    QJsonDocument doc = QJsonDocument::fromJson(message.toUtf8());
    if (doc.isObject()) emit received(doc.object());
}

void SignalingClient::sendMessage(const QJsonObject &msg) {
    ws.sendTextMessage(QJsonDocument(msg).toJson(QJsonDocument::Compact));
}
