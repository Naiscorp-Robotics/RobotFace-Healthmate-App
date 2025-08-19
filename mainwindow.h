#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QWebSocket>  // WebSocket client

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_btnConnect_clicked();
    void on_btnSend_clicked();
    void onConnected();
    void onDisconnected();  // New: handle disconnect
    void onTextMessageReceived(const QString &message);

private:
    Ui::MainWindow *ui;
    QWebSocket m_webSocket;  // WebSocket instance
    // QWebEngineView* m_webView;  // Uncomment if you need WebRTC view
};

#endif // MAINWINDOW_H
