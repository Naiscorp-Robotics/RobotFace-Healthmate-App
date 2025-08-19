#ifndef AUDIOMANAGER_H
#define AUDIOMANAGER_H

#include <QObject>
#include <alsa/asoundlib.h>
#include <QByteArray>

class AudioManager : public QObject
{
    Q_OBJECT
public:
    explicit AudioManager(QObject *parent = nullptr);
    ~AudioManager();

    bool startCapture();
    void stopCapture();
    bool playAudio();
    bool isCapturing() const;

signals:
    void errorOccurred(const QString &message);
    void captureStarted();
    void captureStopped();
    void audioPlayed();

private:
    snd_pcm_t *m_captureHandle;
    snd_pcm_t *m_playbackHandle;
    bool m_isCapturing;
    QByteArray m_audioBuffer;
};

#endif // AUDIOMANAGER_H
