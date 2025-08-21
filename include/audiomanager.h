#ifndef AUDIOMANAGER_H
#define AUDIOMANAGER_H

#include <QObject>
#include <alsa/asoundlib.h>
#include <QByteArray>
#include <QFile>
#include <QAudioFormat>

class AudioManager : public QObject
{
    Q_OBJECT
    Q_PROPERTY(bool isCapturing READ isCapturing NOTIFY isCapturingChanged)
    Q_PROPERTY(bool hasRecordedData READ hasRecordedData NOTIFY hasRecordedDataChanged)

public:
    explicit AudioManager(QObject *parent = nullptr);
    ~AudioManager();

    Q_INVOKABLE bool startCapture();
    Q_INVOKABLE void stopCapture();
    Q_INVOKABLE bool playAudio();
    Q_INVOKABLE bool saveToFile(const QString &filename);
    Q_INVOKABLE bool loadFromFile(const QString &filename);
    
    // Base64 encoding/decoding functions
    Q_INVOKABLE QString getAudioAsBase64() const;
    Q_INVOKABLE bool loadFromBase64(const QString &base64Data);
    Q_INVOKABLE QString getWavAsBase64() const;

    // SỬA: Thêm const và chỉ khai báo, implementation trong .cpp
    bool isCapturing() const;
    bool hasRecordedData() const;

    // Thêm các phương thức format
    Q_INVOKABLE int sampleRate() const { return 44100; }
    Q_INVOKABLE int channelCount() const { return 1; }
    Q_INVOKABLE int sampleSize() const { return 16; }
    
    // Kiểm tra trạng thái audio device
    Q_INVOKABLE bool checkAudioDevice();
    
    // Debug methods
    Q_INVOKABLE int getAudioBufferSize() const { return m_audioBuffer.size(); }
    Q_INVOKABLE QString getAudioBufferInfo() const;
    
    // Auto-save methods
    Q_INVOKABLE QString autoSaveAudio();
    Q_INVOKABLE QString autoSaveBase64();
    Q_INVOKABLE void autoSaveAll();

signals:
    void errorOccurred(const QString &message);
    void captureStarted();
    void captureStopped();
    void audioPlayed();
    void isCapturingChanged();
    void hasRecordedDataChanged();
    void recordingSaved(bool success);
    void autoSaveCompleted(const QString &audioFile, const QString &base64File);

private:
    void captureAudioData();
    bool setupPlayback();

    snd_pcm_t *m_captureHandle;
    snd_pcm_t *m_playbackHandle;
    bool m_isCapturing;
    QByteArray m_audioBuffer;
    QFile m_outputFile;
};

#endif // AUDIOMANAGER_H
