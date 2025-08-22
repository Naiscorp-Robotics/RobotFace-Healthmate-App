#include "mapimageprovider.h"
#include <QMutexLocker>

MapImageProvider::MapImageProvider()
    : QQuickImageProvider(QQuickImageProvider::Image)
{
}

QImage MapImageProvider::requestImage(const QString &id, QSize *size, const QSize &requestedSize)
{
    Q_UNUSED(id)
    
    QMutexLocker locker(&m_mutex);
    
    if (size) {
        *size = m_image.size();
    }
    
    if (requestedSize.isValid() && !m_image.isNull()) {
        return m_image.scaled(requestedSize, Qt::KeepAspectRatio, Qt::SmoothTransformation);
    }
    
    return m_image;
}

void MapImageProvider::updateImage(const QImage &image)
{
    QMutexLocker locker(&m_mutex);
    m_image = image;
}
