#include "StreamUploader.h"
#include <QWebSocket>
#include <QDebug>

StreamUploader::StreamUploader(QObject *parent)
    : QObject(parent)
    , m_webSocket(new QWebSocket())
{
    m_webSocket->setParent(this);
    connect(m_webSocket, &QWebSocket::connected, this, &StreamUploader::onConnected);
    connect(m_webSocket, &QWebSocket::disconnected, this, &StreamUploader::onDisconnected);
    connect(m_webSocket, QOverload<QAbstractSocket::SocketError>::of(&QWebSocket::error),
            this, &StreamUploader::onError);
    connect(m_webSocket, &QWebSocket::bytesWritten, this, &StreamUploader::onBytesWritten);
}

StreamUploader::~StreamUploader()
{
    m_webSocket->close();
    m_webSocket->deleteLater();
}

void StreamUploader::uploadAudioData(const QByteArray &audioData, const QString &serverUrl)
{
    if (audioData.isEmpty()) {
        emit uploadError("No audio data to upload");
        return;
    }

    QString finalUrl = serverUrl.isEmpty() ? m_serverUrl : serverUrl;
    if (finalUrl.isEmpty()) {
        emit uploadError("Server URL not set");
        return;
    }

    if (m_webSocket->state() != QAbstractSocket::ConnectedState) {
        m_pendingData = audioData;
        m_webSocket->open(QUrl(finalUrl));
    } else {
        qint64 bytesSent = m_webSocket->sendBinaryMessage(audioData);
        emit uploadProgress(bytesSent, audioData.size());
    }
}

void StreamUploader::setServerUrl(const QString &url)
{
    m_serverUrl = url;
}

void StreamUploader::onConnected()
{
    qDebug() << "WebSocket connected";
    if (!m_pendingData.isEmpty()) {
        qint64 bytesSent = m_webSocket->sendBinaryMessage(m_pendingData);
        emit uploadProgress(bytesSent, m_pendingData.size());
        m_pendingData.clear();
    }
}

void StreamUploader::onDisconnected()
{
    qDebug() << "WebSocket disconnected";
    emit uploadFinished(true, "Upload completed and connection closed");
}

void StreamUploader::onError(QAbstractSocket::SocketError error)
{
    QString errorMsg = QString("WebSocket error: %1").arg(m_webSocket->errorString());
    qWarning() << errorMsg;
    emit uploadError(errorMsg);
}

void StreamUploader::onBytesWritten(qint64 bytes)
{
    emit uploadProgress(bytes, bytes);
    emit uploadFinished(true, "Binary message sent successfully");
}
