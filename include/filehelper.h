#ifndef FILEHELPER_H
#define FILEHELPER_H

#include <QObject>
#include <QString>

class FileHelper : public QObject
{
    Q_OBJECT

public:
    explicit FileHelper(QObject *parent = nullptr);

    Q_INVOKABLE QString readResourceFile(const QString &resourcePath);
    Q_INVOKABLE bool resourceFileExists(const QString &resourcePath);
    Q_INVOKABLE QString readVoiceBaseFile();
};

#endif // FILEHELPER_H
