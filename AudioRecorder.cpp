#include "AudioRecorder.h"
#include <QAudioFormat>
#include <QMediaDevices>
#include <QAudioInput>
#include <QAudioSource>
#include <QDebug>

AudioRecorder::AudioRecorder(QObject *parent)
    : QObject(parent)
    , m_audioInput(nullptr)
    , m_audioIO(nullptr)
    , m_isRecording(false)
{
}

AudioRecorder::~AudioRecorder()
{
    stopRecording();
}

bool AudioRecorder::startRecording()
{
    if (m_isRecording) {
        return false;
    }

    // Thiết lập định dạng audio
    QAudioFormat format;
    format.setSampleRate(44100);
    format.setChannelCount(1);
    format.setSampleFormat(QAudioFormat::Int16);

    // Kiểm tra thiết bị đầu vào
    QAudioDevice inputDevice = QMediaDevices::defaultAudioInput();
    if (inputDevice.isNull()) {
        qWarning() << "No audio input device available";
        return false;
    }

    if (!inputDevice.isFormatSupported(format)) {
        qWarning() << "Default format not supported, trying to use nearest";
        format = inputDevice.preferredFormat();
    }

    // SỬA: Sử dụng QAudioSource thay vì QAudioInput cho Qt6
    m_audioInput = new QAudioSource(inputDevice, format, this);

    // SỬA: Signal stateChanged thuộc về QAudioSource trong Qt6
    connect(m_audioInput, &QAudioSource::stateChanged, this, [this](QAudio::State state) {
        if (state == QAudio::StoppedState && m_isRecording) {
            stopRecording();
        }
    });

    // SỬA: start() trả về QIODevice*
    m_audioIO = m_audioInput->start();
    if (!m_audioIO) {
        qWarning() << "Failed to start audio input";
        delete m_audioInput;
        m_audioInput = nullptr;
        return false;
    }

    // Kết nối signal để nhận dữ liệu audio
    connect(m_audioIO, &QIODevice::readyRead, this, &AudioRecorder::handleDataAvailable);

    m_isRecording = true;
    m_audioData.clear();

    emit recordingStatusChanged(true);
    return true;
}

void AudioRecorder::stopRecording()
{
    if (!m_isRecording) {
        return;
    }

    if (m_audioInput) {
        m_audioInput->stop();
        if (m_audioIO) {
            disconnect(m_audioIO, &QIODevice::readyRead, this, &AudioRecorder::handleDataAvailable);
        }
        m_audioInput->deleteLater();
        m_audioInput = nullptr;
    }

    m_audioIO = nullptr;
    m_isRecording = false;

    emit recordingStatusChanged(false);
}

bool AudioRecorder::isRecording() const
{
    return m_isRecording;
}

QByteArray AudioRecorder::getAudioData() const
{
    return m_audioData;
}

void AudioRecorder::handleDataAvailable()
{
    if (!m_audioIO) {
        return;
    }

    qint64 bytesReady = m_audioIO->bytesAvailable();
    if (bytesReady > 0) {
        QByteArray data = m_audioIO->readAll();
        m_audioData.append(data);

        emit audioDataAvailable(data);
    }
}
