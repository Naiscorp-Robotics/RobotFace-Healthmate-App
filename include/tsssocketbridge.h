#ifndef TSSSOCKETBRIDGE_H
#define TSSSOCKETBRIDGE_H

#include <QObject>
#include <QWebSocket>
#include <QNetworkAccessManager>
#include <QJsonObject>
#include <QJsonDocument>

class TSSSocketBridge : public QObject
{
    Q_OBJECT
    Q_PROPERTY(bool isConnected READ isConnected NOTIFY connectionStatusChanged)
    Q_PROPERTY(QString lastMessage READ lastMessage NOTIFY messageReceived)
    Q_PROPERTY(int currentStepNumber READ currentStepNumber NOTIFY stepDataChanged)
    Q_PROPERTY(QString currentStepDescription READ currentStepDescription NOTIFY stepDataChanged)
    Q_PROPERTY(QString currentImageBase64 READ currentImageBase64 NOTIFY stepDataChanged)
    Q_PROPERTY(QString currentAudioBase64 READ currentAudioBase64 NOTIFY stepDataChanged)

public:
    explicit TSSSocketBridge(QObject *parent = nullptr);
    ~TSSSocketBridge();

    Q_INVOKABLE void connectToServer(const QString &url = "ws://localhost:12346");
    Q_INVOKABLE void disconnectFromServer();
    Q_INVOKABLE void sendMessage(const QString &message);
    Q_INVOKABLE void sendStepRequest(int stepNumber, const QString &stepDescription = "");

    
    bool isConnected() const { return m_webSocket.state() == QAbstractSocket::ConnectedState; }
    QString lastMessage() const { return m_lastMessage; }
    int currentStepNumber() const { return m_currentStepNumber; }
    QString currentStepDescription() const { return m_currentStepDescription; }
    QString currentImageBase64() const { return m_currentImageBase64; }
    QString currentAudioBase64() const { return m_currentAudioBase64; }

signals:
    void connectionStatusChanged();
    void messageReceived(const QString &message);
    void logMessage(const QString &message);
    void stepDataChanged();

private slots:
    void onWebSocketConnected();
    void onWebSocketDisconnected();
    void onWebSocketTextMessageReceived(const QString &message);
    void onWebSocketError(QAbstractSocket::SocketError error);

private:
    QWebSocket m_webSocket;
    QNetworkAccessManager *m_networkManager;
    QString m_lastMessage;
    int m_currentStepNumber;
    QString m_currentStepDescription;
    QString m_currentImageBase64;
    QString m_currentAudioBase64;
};

#endif // TSSSOCKETBRIDGE_H
