#include "audiomanager.h"
#include <QDebug>
#include <QThread>
#include <QDateTime>
#include <QDir>
#include <QTimer>
#include <QFileInfo>
#include <QTextStream>
#include <QDataStream>
#include <QRegularExpression>
#include <QtEndian>

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

    qDebug() << "Starting audio capture...";

    // Open PCM device for capture
    err = snd_pcm_open(&m_captureHandle, "default", SND_PCM_STREAM_CAPTURE, 0);
    if (err < 0) {
        QString errorMsg = QString("Cannot open audio device: %1").arg(snd_strerror(err));
        qDebug() << "Audio capture error:" << errorMsg;
        emit errorOccurred(errorMsg);
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
    
    // Store actual audio parameters from hardware
    unsigned int actual_channels;
    snd_pcm_hw_params_get_channels(hw_params, &actual_channels);
    m_sampleRate = sample_rate;
    m_channels = actual_channels;
    m_bitsPerSample = 16;
    
    qDebug() << "AudioManager::startCapture() - Actual audio parameters:";
    qDebug() << "  - Sample rate:" << m_sampleRate << "Hz";
    qDebug() << "  - Channels:" << m_channels;
    qDebug() << "  - Bits per sample:" << m_bitsPerSample;

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

    // Continue capturing with a small delay to prevent blocking
    QTimer::singleShot(10, this, &AudioManager::captureAudioData);
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

    qDebug() << "AudioManager::playAudio() - Starting playback";
    qDebug() << "  - Buffer size:" << m_audioBuffer.size() << "bytes";
    qDebug() << "  - Estimated duration:" << calculateDuration(m_audioBuffer.size()) << "seconds";

    if (!setupPlayback()) {
        emit errorOccurred("Failed to setup playback");
        return false;
    }

    qDebug() << "AudioManager::playAudio() - Playback setup completed, starting audio output";

    // Play the audio buffer in chunks to prevent blocking
    const int chunk_size = 4096;
    const char* data = m_audioBuffer.constData();
    int total_samples = m_audioBuffer.size() / 2; // 16-bit samples
    int samples_written = 0;

    while (samples_written < total_samples) {
        int samples_to_write = qMin(chunk_size, total_samples - samples_written);
        int err = snd_pcm_writei(m_playbackHandle, data + samples_written * 2, samples_to_write);
        
        if (err < 0) {
            emit errorOccurred(QString("Playback error: %1").arg(snd_strerror(err)));
            snd_pcm_close(m_playbackHandle);
            m_playbackHandle = nullptr;
            return false;
        }
        
        samples_written += err;
        
        // Small delay to prevent blocking
        QThread::msleep(1);
    }

    snd_pcm_drain(m_playbackHandle);
    snd_pcm_close(m_playbackHandle);
    m_playbackHandle = nullptr;

    qDebug() << "AudioManager::playAudio() - Playback completed successfully";
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

    unsigned int sample_rate = m_sampleRate;
    err = snd_pcm_hw_params_set_rate_near(m_playbackHandle, hw_params, &sample_rate, 0);
    if (err < 0) {
        emit errorOccurred(QString("Cannot set playback rate: %1").arg(snd_strerror(err)));
        return false;
    }

    err = snd_pcm_hw_params_set_channels(m_playbackHandle, hw_params, m_channels);
    if (err < 0) {
        emit errorOccurred(QString("Cannot set playback channels: %1").arg(snd_strerror(err)));
        return false;
    }
    
    qDebug() << "AudioManager::setupPlayback() - Using audio parameters:";
    qDebug() << "  - Sample rate:" << sample_rate << "Hz";
    qDebug() << "  - Channels:" << m_channels;
    qDebug() << "  - Bits per sample:" << m_bitsPerSample;

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
    qDebug() << "=== saveToFile called ===";
    qDebug() << "Filename:" << filename;
    qDebug() << "Audio buffer empty:" << m_audioBuffer.isEmpty();
    qDebug() << "Audio buffer size:" << m_audioBuffer.size() << "bytes";
    
    if (m_audioBuffer.isEmpty()) {
        qDebug() << "ERROR: No audio data to save";
        emit errorOccurred("No audio data to save");
        return false;
    }

    // Ensure the directory exists
    QFileInfo fileInfo(filename);
    QDir dir = fileInfo.absoluteDir();
    qDebug() << "Directory path:" << dir.absolutePath();
    qDebug() << "Directory exists:" << dir.exists();
    
    if (!dir.exists()) {
        qDebug() << "Creating directory:" << dir.absolutePath();
        if (!dir.mkpath(".")) {
            QString errorMsg = "Cannot create directory: " + dir.absolutePath();
            qDebug() << "ERROR:" << errorMsg;
            emit errorOccurred(errorMsg);
            return false;
        }
    }

    QFile file(filename);
    qDebug() << "Opening file for writing:" << filename;
    if (!file.open(QIODevice::WriteOnly)) {
        QString errorMsg = QString("Cannot open file for writing: %1 - %2").arg(filename).arg(file.errorString());
        qDebug() << "ERROR:" << errorMsg;
        emit errorOccurred(errorMsg);
        return false;
    }

    qDebug() << "File opened successfully";
    qDebug() << "Saving audio file:" << filename;
    qDebug() << "Audio buffer size:" << m_audioBuffer.size() << "bytes";

    // Write WAV header using actual audio parameters
    QByteArray header(44, 0);

    // Calculate WAV header values using actual parameters
    const quint16 numChannels = static_cast<quint16>(m_channels);
    const quint32 sampleRate = static_cast<quint32>(m_sampleRate);
    const quint16 bitsPerSample = static_cast<quint16>(m_bitsPerSample);
    const quint16 blockAlign = numChannels * (bitsPerSample / 8);
    const quint32 byteRate = sampleRate * blockAlign;
    const quint32 dataSize = m_audioBuffer.size();
    const quint32 fileSize = 36 + dataSize; // RIFF size

    // Helper functions for endian conversion
    auto put32 = [&](int off, quint32 v) { 
        quint32 le = qToLittleEndian(v); 
        header.replace(off, 4, reinterpret_cast<const char*>(&le), 4); 
    };
    auto put16 = [&](int off, quint16 v) { 
        quint16 le = qToLittleEndian(v); 
        header.replace(off, 2, reinterpret_cast<const char*>(&le), 2); 
    };

    // RIFF header
    header.replace(0, 4, "RIFF");
    put32(4, fileSize);
    header.replace(8, 4, "WAVE");

    // fmt chunk
    header.replace(12, 4, "fmt ");
    put32(16, 16); // fmt chunk size
    put16(20, 1);  // PCM format
    put16(22, numChannels);
    put32(24, sampleRate);
    put32(28, byteRate);
    put16(32, blockAlign);
    put16(34, bitsPerSample);
    
    // data chunk
    header.replace(36, 4, "data");
    put32(40, dataSize);

    qDebug() << "Writing WAV header, size:" << header.size() << "bytes";
    qint64 headerWritten = file.write(header);
    qDebug() << "Header written:" << headerWritten << "bytes";
    
    qDebug() << "Writing audio data, size:" << m_audioBuffer.size() << "bytes";
    qint64 dataWritten = file.write(m_audioBuffer);
    qDebug() << "Data written:" << dataWritten << "bytes";
    
    if (headerWritten != header.size() || dataWritten != m_audioBuffer.size()) {
        QString errorMsg = QString("Failed to write audio data to file. Header: %1/%2, Data: %3/%4")
                          .arg(headerWritten).arg(header.size())
                          .arg(dataWritten).arg(m_audioBuffer.size());
        qDebug() << "ERROR:" << errorMsg;
        file.close();
        emit errorOccurred(errorMsg);
        return false;
    }

    file.close();
    qDebug() << "File closed successfully";
    
    // Verify file was created and has correct size
    QFileInfo savedFile(filename);
    qDebug() << "Saved file exists:" << savedFile.exists();
    qDebug() << "Saved file size:" << savedFile.size() << "bytes";
    qDebug() << "Expected file size:" << (header.size() + m_audioBuffer.size()) << "bytes";
    
    if (!savedFile.exists()) {
        qDebug() << "ERROR: File was not created!";
        emit errorOccurred("File was not created after writing");
        return false;
    }
    
    qDebug() << "Audio file saved successfully:" << filename;
    qDebug() << "File size:" << savedFile.size() << "bytes";
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

bool AudioManager::checkAudioDevice()
{
    snd_pcm_t *testHandle;
    int err = snd_pcm_open(&testHandle, "default", SND_PCM_STREAM_CAPTURE, 0);
    if (err < 0) {
        qDebug() << "Audio device check failed:" << snd_strerror(err);
        return false;
    }
    
    snd_pcm_close(testHandle);
    qDebug() << "Audio device check passed";
    return true;
}

bool AudioManager::loadFromBase64(const QString &base64Data)
{
    if (base64Data.isEmpty()) {
        qDebug() << "AudioManager::loadFromBase64() - Base64 data is empty";
        emit errorOccurred("Base64 data is empty");
        return false;
    }
    
    qDebug() << "AudioManager::loadFromBase64() - Starting decode";
    qDebug() << "  - Base64 string length:" << base64Data.length() << "characters";
    qDebug() << "  - Base64 preview:" << base64Data.left(50) + "...";
    
    // Clean up base64 data - remove data URI prefix and whitespace
    QString cleanBase64 = base64Data.trimmed();
    cleanBase64.remove(QRegularExpression("^data:audio/[^;]+;base64,", QRegularExpression::CaseInsensitiveOption));
    cleanBase64.remove(QRegularExpression("\\s+"));
    
    qDebug() << "AudioManager::loadFromBase64() - Cleaned base64 length:" << cleanBase64.length() << "characters";
    
    // Validate base64 format (more lenient)
    if (cleanBase64.length() < 100) {
        qDebug() << "AudioManager::loadFromBase64() - Base64 data too short, likely invalid";
        emit errorOccurred("Base64 data too short");
        return false;
    }
    
    // Check if it's a valid base64 string (more lenient)
    QRegExp base64Pattern("^[A-Za-z0-9+/]*={0,2}$");
    if (!base64Pattern.exactMatch(cleanBase64)) {
        qDebug() << "AudioManager::loadFromBase64() - Invalid base64 format";
        emit errorOccurred("Invalid base64 format");
        return false;
    }
    
    // Decode base64 to binary data using optimized method
    QByteArray audioData;
    try {
        audioData = QByteArray::fromBase64(cleanBase64.toLatin1());
    } catch (...) {
        qDebug() << "AudioManager::loadFromBase64() - Exception during base64 decode";
        emit errorOccurred("Exception during base64 decode");
        return false;
    }
    
    if (audioData.isEmpty()) {
        qDebug() << "AudioManager::loadFromBase64() - Failed to decode base64 data";
        emit errorOccurred("Failed to decode base64 data");
        return false;
    }
    
    qDebug() << "AudioManager::loadFromBase64() - Successfully decoded";
    qDebug() << "  - Decoded data size:" << audioData.size() << "bytes";
    
    // Check if this is a WAV file (has WAV header)
    if (audioData.size() > 44 && audioData.startsWith("RIFF") && audioData.mid(8, 4) == "WAVE") {
        qDebug() << "AudioManager::loadFromBase64() - Detected WAV format, parsing header";
        
        // Parse WAV header to get actual audio parameters
        const char* p = audioData.constData();
        
        // Parse fmt chunk to get audio parameters
        if (audioData.size() >= 44) {
            // Read audio format parameters from fmt chunk
            quint16 numChannels = qFromLittleEndian(*reinterpret_cast<const quint16*>(p + 22));
            quint32 sampleRate = qFromLittleEndian(*reinterpret_cast<const quint32*>(p + 24));
            quint16 bitsPerSample = qFromLittleEndian(*reinterpret_cast<const quint16*>(p + 34));
            
            qDebug() << "AudioManager::loadFromBase64() - WAV parameters:";
            qDebug() << "  - Sample rate:" << sampleRate << "Hz";
            qDebug() << "  - Channels:" << numChannels;
            qDebug() << "  - Bits per sample:" << bitsPerSample;
            
            // Update audio parameters to match WAV file
            m_sampleRate = sampleRate;
            m_channels = numChannels;
            m_bitsPerSample = bitsPerSample;
        }
        
        // Extract audio data (skip 44-byte header)
        m_audioBuffer = audioData.mid(44);
        qDebug() << "AudioManager::loadFromBase64() - Extracted audio data size:" << m_audioBuffer.size() << "bytes";
        qDebug() << "AudioManager::loadFromBase64() - Estimated duration:" << calculateDuration(m_audioBuffer.size()) << "seconds";
    } else {
        qDebug() << "AudioManager::loadFromBase64() - Raw audio data (no WAV header)";
        qDebug() << "AudioManager::loadFromBase64() - Using current audio parameters:";
        qDebug() << "  - Sample rate:" << m_sampleRate << "Hz";
        qDebug() << "  - Channels:" << m_channels;
        qDebug() << "  - Bits per sample:" << m_bitsPerSample;
        
        // Raw audio data, use as is
        m_audioBuffer = audioData;
        qDebug() << "AudioManager::loadFromBase64() - Estimated duration:" << calculateDuration(m_audioBuffer.size()) << "seconds";
    }
    
    // Validate audio data size (should be reasonable for audio)
    if (m_audioBuffer.size() < 1000) {
        qDebug() << "AudioManager::loadFromBase64() - Audio data too small, likely invalid";
        emit errorOccurred("Audio data too small");
        return false;
    }
    
    if (m_audioBuffer.size() > 10 * 1024 * 1024) { // 10MB limit
        qDebug() << "AudioManager::loadFromBase64() - Audio data too large";
        emit errorOccurred("Audio data too large");
        return false;
    }
    emit hasRecordedDataChanged();
    
    qDebug() << "AudioManager::loadFromBase64() - Audio data loaded successfully";
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
    qDebug() << "  - Sample rate:" << m_sampleRate << " Hz, Channels:" << m_channels << ", Bit depth:" << m_bitsPerSample;
    
    // Create WAV header + audio data using actual audio parameters
    QByteArray wavData;
    QByteArray header(44, 0);

    // Calculate WAV header values using actual parameters
    const quint16 numChannels = static_cast<quint16>(m_channels);
    const quint32 sampleRate = static_cast<quint32>(m_sampleRate);
    const quint16 bitsPerSample = static_cast<quint16>(m_bitsPerSample);
    const quint16 blockAlign = numChannels * (bitsPerSample / 8);
    const quint32 byteRate = sampleRate * blockAlign;
    const quint32 dataSize = m_audioBuffer.size();
    const quint32 fileSize = 36 + dataSize; // RIFF size

    // Helper functions for endian conversion
    auto put32 = [&](int off, quint32 v) { 
        quint32 le = qToLittleEndian(v); 
        header.replace(off, 4, reinterpret_cast<const char*>(&le), 4); 
    };
    auto put16 = [&](int off, quint16 v) { 
        quint16 le = qToLittleEndian(v); 
        header.replace(off, 2, reinterpret_cast<const char*>(&le), 2); 
    };

    // RIFF header
    header.replace(0, 4, "RIFF");
    put32(4, fileSize);
    header.replace(8, 4, "WAVE");

    // fmt chunk
    header.replace(12, 4, "fmt ");
    put32(16, 16); // fmt chunk size
    put16(20, 1);  // PCM format
    put16(22, numChannels);
    put32(24, sampleRate);
    put32(28, byteRate);
    put16(32, blockAlign);
    put16(34, bitsPerSample);
    
    // data chunk
    header.replace(36, 4, "data");
    put32(40, dataSize);

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

QString AudioManager::getWavAsDataUri() const
{
    QString base64Data = getWavAsBase64();
    if (base64Data.isEmpty()) {
        return QString();
    }
    
    // Return as data URI for web usage
    return QString("data:audio/wav;base64,%1").arg(base64Data);
}

QString AudioManager::getAudioBufferInfo() const
{
    QString info = QString("Audio Buffer Info:\n");
    info += QString("- Size: %1 bytes\n").arg(m_audioBuffer.size());
    info += QString("- Empty: %1\n").arg(m_audioBuffer.isEmpty() ? "Yes" : "No");
    info += QString("- Duration: %1 seconds\n").arg(calculateDuration(m_audioBuffer.size()));
    return info;
}

QString AudioManager::autoSaveAudio()
{
    if (m_audioBuffer.isEmpty()) {
        qDebug() << "ERROR: No audio data to auto-save";
        return QString();
    }

    // Create timestamp for filename
    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd_hh-mm-ss");
    QString filename = QString("audio_%1.wav").arg(timestamp);
    
    // Get current working directory
    QString currentDir = QDir::currentPath();
    QString fullPath = QDir(currentDir).filePath(filename);
    
    qDebug() << "Auto-saving audio to:" << fullPath;
    
    if (saveToFile(fullPath)) {
        qDebug() << "Auto-save audio successful:" << fullPath;
        return fullPath;
    } else {
        qDebug() << "Auto-save audio failed:" << fullPath;
        return QString();
    }
}

QString AudioManager::autoSaveBase64()
{
    if (m_audioBuffer.isEmpty()) {
        qDebug() << "ERROR: No audio data to auto-save as base64";
        return QString();
    }

    // Create timestamp for filename
    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd_hh-mm-ss");
    QString filename = QString("audio_%1_base64.txt").arg(timestamp);
    
    // Get current working directory
    QString currentDir = QDir::currentPath();
    QString fullPath = QDir(currentDir).filePath(filename);
    
    qDebug() << "Auto-saving base64 to:" << fullPath;
    
    // Generate base64 data
    QString base64Data = getWavAsBase64();
    if (base64Data.isEmpty()) {
        qDebug() << "ERROR: Failed to generate base64 data";
        return QString();
    }
    
    // Save base64 to file
    QFile file(fullPath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        qDebug() << "ERROR: Cannot open base64 file for writing:" << fullPath << "-" << file.errorString();
        return QString();
    }
    
    QTextStream out(&file);
    out << base64Data;
    file.close();
    
    qDebug() << "Auto-save base64 successful:" << fullPath;
    qDebug() << "Base64 file size:" << base64Data.length() << "characters";
    return fullPath;
}

void AudioManager::autoSaveAll()
{
    qDebug() << "=== AUTO SAVE ALL ===";
    
    QString audioFile = autoSaveAudio();
    QString base64File = autoSaveBase64();
    
    if (!audioFile.isEmpty() && !base64File.isEmpty()) {
        qDebug() << "Auto-save all successful:";
        qDebug() << "  Audio file:" << audioFile;
        qDebug() << "  Base64 file:" << base64File;
        emit recordingSaved(true);
        emit autoSaveCompleted(audioFile, base64File);
    } else {
        qDebug() << "Auto-save all failed";
        emit recordingSaved(false);
    }
    
    qDebug() << "=== END AUTO SAVE ALL ===";
}

bool AudioManager::loadFromByteArray(const QByteArray &audioData)
{
    if (audioData.isEmpty()) {
        qDebug() << "AudioManager::loadFromByteArray() - Audio data is empty";
        emit errorOccurred("Audio data is empty");
        return false;
    }
    
    qDebug() << "AudioManager::loadFromByteArray() - Loading audio data directly";
    qDebug() << "  - Audio data size:" << audioData.size() << "bytes";
    qDebug() << "  - Estimated duration:" << calculateDuration(audioData.size()) << "seconds";
    
    // Validate audio data size
    if (audioData.size() < 1000) {
        qDebug() << "AudioManager::loadFromByteArray() - Audio data too small";
        emit errorOccurred("Audio data too small");
        return false;
    }
    
    if (audioData.size() > 10 * 1024 * 1024) { // 10MB limit
        qDebug() << "AudioManager::loadFromByteArray() - Audio data too large";
        emit errorOccurred("Audio data too large");
        return false;
    }
    
    // Store in audio buffer
    m_audioBuffer = audioData;
    emit hasRecordedDataChanged();
    
    qDebug() << "AudioManager::loadFromByteArray() - Audio data loaded successfully";
    return true;
}

// Helper function to calculate audio duration
double AudioManager::calculateDuration(int bufferSize) const {
    const int bytesPerSample = m_bitsPerSample / 8;
    return static_cast<double>(bufferSize) / (m_sampleRate * m_channels * bytesPerSample);
}
