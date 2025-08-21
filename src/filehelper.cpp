#include "filehelper.h"
#include <QFile>
#include <QTextStream>
#include <QDebug>

FileHelper::FileHelper(QObject *parent)
    : QObject(parent)
{
}

QString FileHelper::readResourceFile(const QString &resourcePath)
{
    qDebug() << "FileHelper: Reading resource file:" << resourcePath;
    
    QFile file(resourcePath);
    if (!file.open(QIODevice::ReadOnly)) {  // Remove QIODevice::Text for binary files
        qDebug() << "FileHelper: Failed to open resource file:" << resourcePath;
        qDebug() << "FileHelper: Error:" << file.errorString();
        return QString();
    }
    
    QByteArray data = file.readAll();
    file.close();
    
    // Convert to QString for base64 data
    QString content = QString::fromUtf8(data);
    
    qDebug() << "FileHelper: Successfully read resource file:" << resourcePath;
    qDebug() << "FileHelper: File size:" << data.size() << "bytes";
    qDebug() << "FileHelper: Content length:" << content.length() << "characters";
    
    return content;
}

bool FileHelper::resourceFileExists(const QString &resourcePath)
{
    QFile file(resourcePath);
    bool exists = file.exists();
    qDebug() << "FileHelper: Resource file exists:" << resourcePath << "=" << exists;
    return exists;
}

QString FileHelper::readVoiceBaseFile()
{
    qDebug() << "FileHelper: Reading voice_base.txt file directly...";
    
    // Try multiple resource paths
    QStringList possiblePaths = {
        "qrc:/assets/voice_base.txt",
        ":/assets/voice_base.txt",
        "qrc:/voice_base.txt"
    };
    
    for (const QString &path : possiblePaths) {
        qDebug() << "FileHelper: Trying path:" << path;
        
        QFile file(path);
        if (file.exists()) {
            qDebug() << "FileHelper: File exists at:" << path;
            
            if (file.open(QIODevice::ReadOnly)) {
                QByteArray data = file.readAll();
                file.close();
                
                QString content = QString::fromUtf8(data);
                qDebug() << "FileHelper: Successfully read voice_base.txt from" << path;
                qDebug() << "FileHelper: File size:" << data.size() << "bytes";
                qDebug() << "FileHelper: Content length:" << content.length() << "characters";
                
                return content;
            } else {
                qDebug() << "FileHelper: Failed to open file at" << path << "Error:" << file.errorString();
            }
        } else {
            qDebug() << "FileHelper: File does not exist at:" << path;
        }
    }
    
    qDebug() << "FileHelper: Failed to read voice_base.txt from any path";
    return QString();
}
