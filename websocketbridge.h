#ifndef WEBSOCKETBRIDGE_H
#define WEBSOCKETBRIDGE_H

#include <QObject>
#include <QWebSocket>
#include <QNetworkAccessManager>

class WebSocketBridge : public QObject
{
    Q_OBJECT
    Q_PROPERTY(bool isConnected READ isConnected NOTIFY connectionStatusChanged)
    Q_PROPERTY(QString lastMessage READ lastMessage NOTIFY messageReceived)

public:
    explicit WebSocketBridge(QObject *parent = nullptr);
    ~WebSocketBridge();

    Q_INVOKABLE void connectToServer(const QString &url = "ws://localhost:12345");
    Q_INVOKABLE void disconnectFromServer();
    Q_INVOKABLE void sendMessage(const QString &message);


    bool isConnected() const { return m_webSocket.state() == QAbstractSocket::ConnectedState; }
    QString lastMessage() const { return m_lastMessage; }

signals:
    void connectionStatusChanged();
    void messageReceived(const QString &message);
    void logMessage(const QString &message);

private slots:
    void onWebSocketConnected();
    void onWebSocketDisconnected();
    void onWebSocketTextMessageReceived(const QString &message);
    void onWebSocketError(QAbstractSocket::SocketError error);

private:
    QWebSocket m_webSocket;
    QNetworkAccessManager *m_networkManager;
    QString m_lastMessage;
};

#endif // WEBSOCKETBRIDGE_H
