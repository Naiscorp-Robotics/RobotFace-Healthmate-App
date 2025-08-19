#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QVBoxLayout>
#include <QDebug>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    TCPSocket = new QTcpSocket();
    TCPSocket->connectToHost(QHostAddress::LocalHost,8080);
    TCPSocket->open(QIODevice::ReadWrite);
    if (TCPSocket->isOpen())
    {
        QMessageBox::information(this, "Qt With Ketan", "Connected To The Server.");
    }
    else
    {
        QMessageBox::information(this, "Qt With Ketan", "Not Connected To The Server.");
    }

    // WebRTC WebView setup
    // m_webView = new QWebEngineView(this);
    // m_webView->load(QUrl("http://localhost:8000/webrtc.html"));  // Video call HTML page

    // // Thêm WebView vào widget placeholder trong .ui
    // QVBoxLayout *layout = new QVBoxLayout(ui->webViewContainer);
    // layout->addWidget(m_webView);
    // layout->setContentsMargins(0, 0, 0, 0);  // Remove padding

    // Kết nối tín hiệu WebSocket
    connect(&m_webSocket, &QWebSocket::connected, this, &MainWindow::onConnected);
    connect(&m_webSocket, &QWebSocket::textMessageReceived, this, &MainWindow::onTextMessageReceived);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_btnConnect_clicked()
{
    ui->textEditLog->append("Connecting to WebSocket server...");
    m_webSocket.open(QUrl("ws://http://0.0.0.0:8080/"));  // Server address
}

void MainWindow::on_btnSend_clicked()
{
    QString message = ui->lineEditMessage->text();
    m_webSocket.sendTextMessage(message);
    ui->textEditLog->append("Sent: " + message);
    ui->lineEditMessage->clear();
}

void MainWindow::onConnected()
{
    ui->textEditLog->append("✅ WebSocket connected!");
    m_webSocket.sendTextMessage("Hello from Qt client!");
}

void MainWindow::onTextMessageReceived(const QString &message)
{
    ui->textEditLog->append("Received: " + message);
}
