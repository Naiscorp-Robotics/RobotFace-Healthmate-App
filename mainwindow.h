#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QVideoWidget>
#include <QMediaPlayer>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTextEdit>
#include <QLineEdit>
#include <QPushButton>
#include <QLabel>
#include <QStackedWidget>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QJsonDocument>
#include <QJsonObject>
#include <QFileDialog>
#include <QMessageBox>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onSendMessage();
    void onResetMemory();
    void onOpenImage();
    void onBackToMain();
    void onNetworkReplyFinished(QNetworkReply *reply);
    void onNetworkError(QNetworkReply::NetworkError error);

private:
    Ui::MainWindow *ui;
    QWidget *centralWidget;
    QStackedWidget *stackedWidget;
    
    // Main screen widgets
    QWidget *mainScreen;
    QVideoWidget *videoWidget;
    QMediaPlayer *mediaPlayer;
    QTextEdit *statusLabel;
    QLineEdit *messageInput;
    QPushButton *sendButton;
    QPushButton *resetButton;
    QPushButton *imageButton;
    
    // Response screen widgets
    QWidget *responseScreen;
    QTextEdit *responseText;
    QPushButton *backButton;
    
    // Image screen widgets
    QWidget *imageScreen;
    QLabel *imageLabel;
    QPushButton *imageBackButton;
    
    // Network
    QNetworkAccessManager *networkManager;
    
    void setupMainScreen();
    void setupResponseScreen();
    void setupImageScreen();
    void sendNetworkRequest(const QString &endpoint, const QJsonObject &data);
    void showResponse(const QString &response);
    void updateStatus(const QString &status);
};

#endif // MAINWINDOW_H
