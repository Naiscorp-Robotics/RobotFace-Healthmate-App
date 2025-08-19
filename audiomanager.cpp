#include "audiomanager.h"
#include <QDebug>
#include <QThread>

AudioManager::AudioManager(QObject *parent)
    : QObject(parent)
    , m_captureHandle(nullptr)
    , m_playbackHandle(nullptr)
    , m_isCapturing(false)
{
}

AudioManager::~AudioManager()
{
    stopCapture();
}

bool AudioManager::startCapture()
{
    int err;

    // Open PCM device for capture
    err = snd_pcm_open(&m_captureHandle, "default", SND_PCM_STREAM_CAPTURE, 0);
    if (err < 0) {
        emit errorOccurred(QString("Cannot open audio device: %1").arg(snd_strerror(err)));
        return false;
    }

    // Set hardware parameters
    snd_pcm_hw_params_t *hw_params;
    snd_pcm_hw_params_alloca(&hw_params);

    err = snd_pcm_hw_params_any(m_captureHandle, hw_params);
    if (err < 0) {
        emit errorOccurred(QString("Cannot initialize hardware parameter structure: %1").arg(snd_strerror(err)));
        snd_pcm_close(m_captureHandle);
        m_captureHandle = nullptr;
        return false;
    }

    err = snd_pcm_hw_params_set_access(m_captureHandle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED);
    if (err < 0) {
        emit errorOccurred(QString("Cannot set access type: %1").arg(snd_strerror(err)));
        snd_pcm_close(m_captureHandle);
        m_captureHandle = nullptr;
        return false;
    }

    err = snd_pcm_hw_params_set_format(m_captureHandle, hw_params, SND_PCM_FORMAT_S16_LE);
    if (err < 0) {
        emit errorOccurred(QString("Cannot set sample format: %1").arg(snd_strerror(err)));
        snd_pcm_close(m_captureHandle);
        m_captureHandle = nullptr;
        return false;
    }

    unsigned int sample_rate = 44100;
    err = snd_pcm_hw_params_set_rate_near(m_captureHandle, hw_params, &sample_rate, 0);
    if (err < 0) {
        emit errorOccurred(QString("Cannot set sample rate: %1").arg(snd_strerror(err)));
        snd_pcm_close(m_captureHandle);
        m_captureHandle = nullptr;
        return false;
    }

    err = snd_pcm_hw_params_set_channels(m_captureHandle, hw_params, 1); // Mono for simplicity
    if (err < 0) {
        emit errorOccurred(QString("Cannot set channel count: %1").arg(snd_strerror(err)));
        snd_pcm_close(m_captureHandle);
        m_captureHandle = nullptr;
        return false;
    }

    // Set buffer size
    snd_pcm_uframes_t buffer_size = 1024;
    err = snd_pcm_hw_params_set_buffer_size_near(m_captureHandle, hw_params, &buffer_size);
    if (err < 0) {
        emit errorOccurred(QString("Cannot set buffer size: %1").arg(snd_strerror(err)));
        snd_pcm_close(m_captureHandle);
        m_captureHandle = nullptr;
        return false;
    }

    err = snd_pcm_hw_params(m_captureHandle, hw_params);
    if (err < 0) {
        emit errorOccurred(QString("Cannot set parameters: %1").arg(snd_strerror(err)));
        snd_pcm_close(m_captureHandle);
        m_captureHandle = nullptr;
        return false;
    }

    // Prepare the interface
    err = snd_pcm_prepare(m_captureHandle);
    if (err < 0) {
        emit errorOccurred(QString("Cannot prepare audio interface: %1").arg(snd_strerror(err)));
        snd_pcm_close(m_captureHandle);
        m_captureHandle = nullptr;
        return false;
    }

    m_isCapturing = true;
    emit captureStarted();

    qDebug() << "Audio capture started successfully";
    return true;
}

void AudioManager::stopCapture()
{
    if (m_captureHandle) {
        snd_pcm_drop(m_captureHandle);
        snd_pcm_close(m_captureHandle);
        m_captureHandle = nullptr;
    }
    m_isCapturing = false;
    emit captureStopped();
    qDebug() << "Audio capture stopped";
}

bool AudioManager::playAudio()
{
    // Simple implementation - you can expand this for actual playback
    if (m_audioBuffer.isEmpty()) {
        emit errorOccurred("No audio data to play");
        return false;
    }

    // Simulate playback delay
    QThread::msleep(500);
    emit audioPlayed();
    return true;
}

bool AudioManager::isCapturing() const
{
    return m_isCapturing;
}
