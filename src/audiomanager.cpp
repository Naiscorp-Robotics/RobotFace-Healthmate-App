#include "audiomanager.h"
#include <QDebug>
#include <QThread>
#include <QDateTime>
#include <QDir>
#include <QTimer>

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

    // Clear previous buffer
    m_audioBuffer.clear();
    emit hasRecordedDataChanged();

    m_isCapturing = true;
    emit isCapturingChanged();
    emit captureStarted();

    // Start capturing audio data in a separate thread or using timer
    QTimer::singleShot(0, this, &AudioManager::captureAudioData);

    qDebug() << "Audio capture started successfully";
    return true;
}

void AudioManager::captureAudioData()
{
    if (!m_isCapturing || !m_captureHandle)
        return;

    const int buffer_size = 4096;
    char buffer[buffer_size];

    int err = snd_pcm_readi(m_captureHandle, buffer, buffer_size / 2); // 16-bit samples = 2 bytes
    if (err < 0) {
        emit errorOccurred(QString("Read error: %1").arg(snd_strerror(err)));
        stopCapture();
        return;
    }

    // Append captured data to buffer
    m_audioBuffer.append(buffer, err * 2); // 2 bytes per sample (16-bit)
    emit hasRecordedDataChanged();

    // Continue capturing
    QTimer::singleShot(0, this, &AudioManager::captureAudioData);
}

void AudioManager::stopCapture()
{
    if (m_captureHandle) {
        snd_pcm_drop(m_captureHandle);
        snd_pcm_close(m_captureHandle);
        m_captureHandle = nullptr;
    }

    bool wasCapturing = m_isCapturing;
    m_isCapturing = false;

    if (wasCapturing) {
        emit isCapturingChanged();
        emit captureStopped();
        qDebug() << "Audio capture stopped. Captured" << m_audioBuffer.size() << "bytes";
    }
}

bool AudioManager::playAudio()
{
    if (m_audioBuffer.isEmpty()) {
        emit errorOccurred("No audio data to play");
        return false;
    }

    if (!setupPlayback()) {
        emit errorOccurred("Failed to setup playback");
        return false;
    }

    // Play the audio buffer
    int err = snd_pcm_writei(m_playbackHandle, m_audioBuffer.constData(), m_audioBuffer.size() / 2);
    if (err < 0) {
        emit errorOccurred(QString("Playback error: %1").arg(snd_strerror(err)));
        snd_pcm_close(m_playbackHandle);
        m_playbackHandle = nullptr;
        return false;
    }

    snd_pcm_drain(m_playbackHandle);
    snd_pcm_close(m_playbackHandle);
    m_playbackHandle = nullptr;

    emit audioPlayed();
    return true;
}

bool AudioManager::setupPlayback()
{
    int err;

    err = snd_pcm_open(&m_playbackHandle, "default", SND_PCM_STREAM_PLAYBACK, 0);
    if (err < 0) {
        emit errorOccurred(QString("Cannot open playback device: %1").arg(snd_strerror(err)));
        return false;
    }

    snd_pcm_hw_params_t *hw_params;
    snd_pcm_hw_params_alloca(&hw_params);

    err = snd_pcm_hw_params_any(m_playbackHandle, hw_params);
    if (err < 0) {
        emit errorOccurred(QString("Cannot initialize playback parameters: %1").arg(snd_strerror(err)));
        return false;
    }

    err = snd_pcm_hw_params_set_access(m_playbackHandle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED);
    if (err < 0) {
        emit errorOccurred(QString("Cannot set playback access type: %1").arg(snd_strerror(err)));
        return false;
    }

    err = snd_pcm_hw_params_set_format(m_playbackHandle, hw_params, SND_PCM_FORMAT_S16_LE);
    if (err < 0) {
        emit errorOccurred(QString("Cannot set playback format: %1").arg(snd_strerror(err)));
        return false;
    }

    unsigned int sample_rate = 44100;
    err = snd_pcm_hw_params_set_rate_near(m_playbackHandle, hw_params, &sample_rate, 0);
    if (err < 0) {
        emit errorOccurred(QString("Cannot set playback rate: %1").arg(snd_strerror(err)));
        return false;
    }

    err = snd_pcm_hw_params_set_channels(m_playbackHandle, hw_params, 1);
    if (err < 0) {
        emit errorOccurred(QString("Cannot set playback channels: %1").arg(snd_strerror(err)));
        return false;
    }

    err = snd_pcm_hw_params(m_playbackHandle, hw_params);
    if (err < 0) {
        emit errorOccurred(QString("Cannot set playback parameters: %1").arg(snd_strerror(err)));
        return false;
    }

    err = snd_pcm_prepare(m_playbackHandle);
    if (err < 0) {
        emit errorOccurred(QString("Cannot prepare playback: %1").arg(snd_strerror(err)));
        return false;
    }

    return true;
}

bool AudioManager::saveToFile(const QString &filename)
{
    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly)) {
        emit errorOccurred("Cannot open file for writing: " + filename);
        return false;
    }

    // Write WAV header (simplified)
    // Note: This is a basic implementation, you might want to use a proper WAV library
    QByteArray header(44, 0);

    // RIFF header
    header.replace(0, 4, "RIFF");
    quint32 fileSize = m_audioBuffer.size() + 36;
    header.replace(4, 4, reinterpret_cast<const char*>(&fileSize), 4);
    header.replace(8, 4, "WAVE");

    // fmt chunk
    header.replace(12, 4, "fmt ");
    quint32 fmtSize = 16;
    header.replace(16, 4, reinterpret_cast<const char*>(&fmtSize), 4);
    quint16 audioFormat = 1; // PCM
    header.replace(20, 2, reinterpret_cast<const char*>(&audioFormat), 2);
    quint16 numChannels = 1;
    header.replace(22, 2, reinterpret_cast<const char*>(&numChannels), 2);
    quint32 sampleRate = 44100;
    header.replace(24, 4, reinterpret_cast<const char*>(&sampleRate), 4);
    quint32 byteRate = sampleRate * numChannels * 2; // 16-bit = 2 bytes
    header.replace(28, 4, reinterpret_cast<const char*>(&byteRate), 4);
    quint16 blockAlign = numChannels * 2;
    header.replace(32, 2, reinterpret_cast<const char*>(&blockAlign), 2);
    quint16 bitsPerSample = 16;
    header.replace(34, 2, reinterpret_cast<const char*>(&bitsPerSample), 2);

    // data chunk
    header.replace(36, 4, "data");
    quint32 dataSize = m_audioBuffer.size();
    header.replace(40, 4, reinterpret_cast<const char*>(&dataSize), 4);

    if (file.write(header) != header.size() || file.write(m_audioBuffer) != m_audioBuffer.size()) {
        file.close();
        emit errorOccurred("Failed to write audio data to file");
        return false;
    }

    file.close();
    emit recordingSaved(true);
    return true;
}

bool AudioManager::loadFromFile(const QString &filename)
{
    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly)) {
        emit errorOccurred("Cannot open file for reading: " + filename);
        return false;
    }

    // Skip WAV header (simplified)
    file.seek(44);

    m_audioBuffer = file.readAll();
    file.close();

    emit hasRecordedDataChanged();
    return true;
}

bool AudioManager::isCapturing() const
{
    return m_isCapturing;
}

bool AudioManager::hasRecordedData() const
{
    return !m_audioBuffer.isEmpty();
}

QString AudioManager::getAudioAsBase64() const
{
    if (m_audioBuffer.isEmpty()) {
        qDebug() << "AudioManager::getAudioAsBase64() - No audio data available";
        return QString();
    }
    
    QString base64Data = m_audioBuffer.toBase64();
    qDebug() << "AudioManager::getAudioAsBase64() - Generated base64 data";
    qDebug() << "  - Audio buffer size:" << m_audioBuffer.size() << "bytes";
    qDebug() << "  - Base64 string length:" << base64Data.length() << "characters";
    qDebug() << "  - Base64 preview:" << base64Data.left(50) + "...";
    
    return base64Data;
}

bool AudioManager::loadFromBase64(const QString &base64Data)
{
    if (base64Data.isEmpty()) {
        emit errorOccurred("Base64 data is empty");
        return false;
    }
    
    QByteArray audioData = QByteArray::fromBase64(base64Data.toUtf8());
    if (audioData.isEmpty()) {
        emit errorOccurred("Failed to decode base64 data");
        return false;
    }
    
    m_audioBuffer = audioData;
    emit hasRecordedDataChanged();
    return true;
}

QString AudioManager::getWavAsBase64() const
{
    if (m_audioBuffer.isEmpty()) {
        qDebug() << "AudioManager::getWavAsBase64() - No audio data available";
        return QString();
    }
    
    qDebug() << "AudioManager::getWavAsBase64() - Creating WAV file with base64 encoding";
    qDebug() << "  - Audio buffer size:" << m_audioBuffer.size() << "bytes";
    qDebug() << "  - Sample rate: 44100 Hz, Channels: 1, Bit depth: 16";
    
    // Create WAV header + audio data
    QByteArray wavData;
    QByteArray header(44, 0);

    // RIFF header
    header.replace(0, 4, "RIFF");
    quint32 fileSize = m_audioBuffer.size() + 36;
    header.replace(4, 4, reinterpret_cast<const char*>(&fileSize), 4);
    header.replace(8, 4, "WAVE");

    // fmt chunk
    header.replace(12, 4, "fmt ");
    quint32 fmtSize = 16;
    header.replace(16, 4, reinterpret_cast<const char*>(&fmtSize), 4);
    quint16 audioFormat = 1; // PCM
    header.replace(20, 2, reinterpret_cast<const char*>(&audioFormat), 2);
    quint16 numChannels = 1;
    header.replace(22, 2, reinterpret_cast<const char*>(&numChannels), 2);
    quint32 sampleRate = 44100;
    header.replace(24, 4, reinterpret_cast<const char*>(&sampleRate), 4);
    quint32 byteRate = sampleRate * numChannels * 2; // 16-bit = 2 bytes
    header.replace(28, 4, reinterpret_cast<const char*>(&byteRate), 4);
    quint16 blockAlign = numChannels * 2;
    header.replace(32, 2, reinterpret_cast<const char*>(&blockAlign), 2);
    quint16 bitsPerSample = 16;
    header.replace(34, 2, reinterpret_cast<const char*>(&bitsPerSample), 2);

    // data chunk
    header.replace(36, 4, "data");
    quint32 dataSize = m_audioBuffer.size();
    header.replace(40, 4, reinterpret_cast<const char*>(&dataSize), 4);

    // Combine header + audio data
    wavData = header + m_audioBuffer;
    
    QString base64Data = wavData.toBase64();
    
    qDebug() << "AudioManager::getWavAsBase64() - WAV file created and encoded";
    qDebug() << "  - WAV file size:" << wavData.size() << "bytes (header + audio)";
    qDebug() << "  - Base64 string length:" << base64Data.length() << "characters";
    qDebug() << "  - Base64 WAV preview:" << base64Data.left(50) + "...";
    qDebug() << "  - Full Base64 WAV data:" << base64Data;
    
    return base64Data;
}
