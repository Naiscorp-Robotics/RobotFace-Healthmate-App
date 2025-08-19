#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QMessageBox>
#include <QtNetwork/QTcpSocket>
#include <QtNetwork/QHostAddress>
#include <QDebug>
#include <QString>
#include <QWebSocket>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE


class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_btnConnect_clicked();
    void on_btnSend_clicked();
    void onConnected();
    void onTextMessageReceived(const QString &message);

private:
    Ui::MainWindow *ui;
    QNetworkAccessManager *networkManager;
    QTcpSocket *TCPSocket;
    QWebSocket m_webSocket;
};

#endif // MAINWINDOW_H
