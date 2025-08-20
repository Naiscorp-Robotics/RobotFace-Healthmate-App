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
    Q_PROPERTY(bool autoConnect READ autoConnect WRITE setAutoConnect NOTIFY autoConnectChanged)
    Q_PROPERTY(QString serverUrl READ serverUrl WRITE setServerUrl NOTIFY serverUrlChanged)

public:
    explicit WebSocketBridge(QObject *parent = nullptr);
    ~WebSocketBridge();

    Q_INVOKABLE void connectToServer(const QString &url = "ws://localhost:12345");
    Q_INVOKABLE void disconnectFromServer();
    Q_INVOKABLE void sendMessage(const QString &message);
    Q_INVOKABLE void autoConnectToServer();


    bool isConnected() const { return m_webSocket.state() == QAbstractSocket::ConnectedState; }
    QString lastMessage() const { return m_lastMessage; }
    bool autoConnect() const { return m_autoConnect; }
    QString serverUrl() const { return m_serverUrl; }

    void setAutoConnect(bool autoConnect);
    void setServerUrl(const QString &url);

signals:
    void connectionStatusChanged();
    void messageReceived(const QString &message);
    void logMessage(const QString &message);
    void autoConnectChanged();
    void serverUrlChanged();

private slots:
    void onWebSocketConnected();
    void onWebSocketDisconnected();
    void onWebSocketTextMessageReceived(const QString &message);
    void onWebSocketError(QAbstractSocket::SocketError error);

private:
    QWebSocket m_webSocket;
    QNetworkAccessManager *m_networkManager;
    QString m_lastMessage;
    bool m_autoConnect;
    QString m_serverUrl;
};

#endif // WEBSOCKETBRIDGE_H
