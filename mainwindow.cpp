#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QVBoxLayout>
#include <QDebug>
#include <QMessageBox>
#include <QWebSocket>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // Kết nối tín hiệu WebSocket
    connect(&m_webSocket, &QWebSocket::connected, this, &MainWindow::onConnected);
    connect(&m_webSocket, &QWebSocket::textMessageReceived, this, &MainWindow::onTextMessageReceived);
    connect(&m_webSocket, &QWebSocket::disconnected, this, &MainWindow::onDisconnected);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_btnConnect_clicked()
{
    ui->textEditLog->append("🔄 Connecting to WebSocket server...");
    // Kết nối đúng với server Python (localhost:12345)
    m_webSocket.open(QUrl("ws://localhost:12345"));
}

void MainWindow::on_btnSend_clicked()
{
    QString message = ui->lineEditMessage->text();
    if (!message.isEmpty() && m_webSocket.isValid())
    {
        m_webSocket.sendTextMessage(message);
        ui->textEditLog->append("📤 Sent: " + message);
        ui->lineEditMessage->clear();
    }
    else
    {
        ui->textEditLog->append("⚠️ Cannot send message (not connected or empty).");
    }
}

void MainWindow::onConnected()
{
    ui->textEditLog->append("✅ WebSocket connected!");
    m_webSocket.sendTextMessage("Hello from Qt client!");
}

void MainWindow::onDisconnected()
{
    ui->textEditLog->append("❌ WebSocket disconnected!");
}

void MainWindow::onTextMessageReceived(const QString &message)
{
    ui->textEditLog->append("📥 Received: " + message);
}
