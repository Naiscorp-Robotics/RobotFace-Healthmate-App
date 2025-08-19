#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtNetwork/QTcpSocket>
#include <QtNetwork/QHostAddress>
#include <QMessageBox>
#include <QDebug>
#include <QString>
#include <QByteArray>
#include <QWebSocket>
// #include <QWebEngineView>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE


class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
private:
    Ui::MainWindow *ui;
    QTcpSocket *TCPSocket;
    QWebSocket m_webSocket;
    // QWebEngineView* m_webView;
    void on_btnConnect_clicked();
    void on_btnSend_clicked();
    void onConnected();
    void onTextMessageReceived(const QString &message);
};
#endif // MAINWINDOW_H
