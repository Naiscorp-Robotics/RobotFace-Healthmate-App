#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>
#include <QStyle>
#include <QDebug>
#include <QTimer>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , m_audioRecorder(new AudioRecorder(this))
    , m_streamUploader(new StreamUploader(this))
    , m_isRecording(false)
{
    ui->setupUi(this);
    setupUI();

    // Kết nối signals và slots
    connect(ui->recordButton, &QPushButton::clicked,
            this, &MainWindow::onRecordButtonClicked);
    connect(m_audioRecorder, &AudioRecorder::audioDataAvailable,
            this, &MainWindow::onAudioDataAvailable);
    connect(m_audioRecorder, &AudioRecorder::recordingStatusChanged,
            this, &MainWindow::onRecordingStatusChanged);
    connect(m_streamUploader, &StreamUploader::uploadFinished,
            this, &MainWindow::onUploadFinished);

    // Thiết lập URL server (thay đổi thành URL thực tế của bạn)
    m_streamUploader->setServerUrl("ws://localhost:12345");
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setupUI()
{
    setWindowTitle("Audio Stream Recorder");
    updateUI();
}

void MainWindow::onRecordButtonClicked()
{
    if (!m_isRecording) {
        // Bắt đầu ghi âm
        if (m_audioRecorder->startRecording()) {
            m_isRecording = true;
            ui->statusLabel->setText("Trạng thái: Đang ghi âm...");
        } else {
            QMessageBox::warning(this, "Lỗi", "Không thể bắt đầu ghi âm. Kiểm tra microphone!");
        }
    } else {
        // Dừng ghi âm
        m_audioRecorder->stopRecording();
        m_isRecording = false;
        ui->statusLabel->setText("Trạng thái: Đang tải lên...");

        // Lấy toàn bộ dữ liệu audio và upload
        QByteArray audioData = m_audioRecorder->getAudioData();
        if (!audioData.isEmpty()) {
            m_streamUploader->uploadAudioData(audioData, "");
        }
    }

    updateUI();
}

void MainWindow::onAudioDataAvailable(const QByteArray &data)
{
    // Stream dữ liệu audio theo thời gian thực
    if (m_isRecording && !data.isEmpty()) {
        m_streamUploader->uploadAudioData(data, "");
    }
}

void MainWindow::onRecordingStatusChanged(bool recording)
{
    m_isRecording = recording;
    updateUI();
}

void MainWindow::onUploadFinished(bool success, const QString &message)
{
    // Chỉ cập nhật trạng thái, không hiển thị hộp thoại thông báo
    if (success) {
        ui->statusLabel->setText("Trạng thái: Tải lên thành công!");
        // Đã loại bỏ QMessageBox::information
    } else {
        ui->statusLabel->setText("Trạng thái: Lỗi tải lên!");
        // Đã loại bỏ QMessageBox::warning
    }

    // Tự động chuyển về trạng thái sẵn sàng sau 2 giây
    QTimer::singleShot(2000, this, [this]() {
        if (!m_isRecording) {
            ui->statusLabel->setText("Trạng thái: Sẵn sàng");
        }
    });
}

void MainWindow::updateUI()
{
    if (m_isRecording) {
        ui->recordButton->setText("Dừng ghi âm");
        ui->recordButton->setStyleSheet(
            "QPushButton {"
            "    background-color: #28a745;"
            "    color: white;"
            "    border: none;"
            "    padding: 15px;"
            "    font-size: 16px;"
            "    border-radius: 10px;"
            "    font-weight: bold;"
            "}"
            "QPushButton:hover {"
            "    background-color: #218838;"
            "}"
            "QPushButton:pressed {"
            "    background-color: #1e7e34;"
            "}"
            );
    } else {
        ui->recordButton->setText("Bắt đầu ghi âm");
        ui->recordButton->setStyleSheet(
            "QPushButton {"
            "    background-color: #dc3545;"
            "    color: white;"
            "    border: none;"
            "    padding: 15px;"
            "    font-size: 16px;"
            "    border-radius: 10px;"
            "    font-weight: bold;"
            "}"
            "QPushButton:hover {"
            "    background-color: #c82333;"
            "}"
            "QPushButton:pressed {"
            "    background-color: #bd2130;"
            "}"
            );
    }
}
