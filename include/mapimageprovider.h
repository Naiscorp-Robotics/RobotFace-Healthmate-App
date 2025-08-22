#ifndef MAPIMAGEPROVIDER_H
#define MAPIMAGEPROVIDER_H

#include <QQuickImageProvider>
#include <QImage>
#include <QMutex>

class MapImageProvider : public QQuickImageProvider
{
public:
    MapImageProvider();
    
    QImage requestImage(const QString &id, QSize *size, const QSize &requestedSize) override;
    void updateImage(const QImage &image);
    
private:
    QImage m_image;
    QMutex m_mutex;
};

#endif // MAPIMAGEPROVIDER_H
