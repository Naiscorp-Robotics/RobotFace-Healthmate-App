#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPushButton>
#include <QLabel>
#include "AudioRecorder.h"
#include "StreamUploader.h"

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
    void onRecordButtonClicked();
    void onAudioDataAvailable(const QByteArray &data);
    void onRecordingStatusChanged(bool recording);
    void onUploadFinished(bool success, const QString &message);

private:
    Ui::MainWindow *ui;
    AudioRecorder *m_audioRecorder;
    StreamUploader *m_streamUploader;
    bool m_isRecording;

    void setupUI();
    void updateUI();
};

#endif // MAINWINDOW_H
