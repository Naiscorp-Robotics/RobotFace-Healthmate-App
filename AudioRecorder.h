#ifndef AUDIORECORDER_H
#define AUDIORECORDER_H

#include <QObject>
#include <QByteArray>
#include <QBuffer>
#include <QAudioSource>  // Đổi từ QAudioInput sang QAudioSource
#include <QIODevice>

class AudioRecorder : public QObject
{
    Q_OBJECT

public:
    explicit AudioRecorder(QObject *parent = nullptr);
    ~AudioRecorder();

    bool startRecording();
    void stopRecording();
    bool isRecording() const;
    QByteArray getAudioData() const;

signals:
    void recordingStatusChanged(bool recording);
    void audioDataAvailable(const QByteArray &data);

private slots:
    void handleDataAvailable();

private:
    QAudioSource *m_audioInput;  // Đổi từ QAudioInput* sang QAudioSource*
    QIODevice *m_audioIO;
    QByteArray m_audioData;
    bool m_isRecording;
};

#endif // AUDIORECORDER_H
