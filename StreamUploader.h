#ifndef STREAMUPLOADER_H
#define STREAMUPLOADER_H

#include <QObject>
#include <QWebSocket>
#include <QByteArray>

class StreamUploader : public QObject
{
    Q_OBJECT
public:
    explicit StreamUploader(QObject *parent = nullptr);
    ~StreamUploader();

    void uploadAudioData(const QByteArray &audioData, const QString &serverUrl = QString());
    void setServerUrl(const QString &url);

signals:
    void uploadProgress(qint64 bytesSent, qint64 bytesTotal);
    void uploadFinished(bool success, const QString &message);
    void uploadError(const QString &errorMessage);

private slots:
    void onConnected();
    void onDisconnected();
    void onError(QAbstractSocket::SocketError error);
    void onBytesWritten(qint64 bytes);

private:
    QWebSocket *m_webSocket;
    QString m_serverUrl;
    QByteArray m_pendingData;
};

#endif // STREAMUPLOADER_H
