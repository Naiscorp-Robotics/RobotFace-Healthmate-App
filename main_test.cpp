#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QDebug>
#include "websocketbridge.h"

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);
    
    qDebug() << "Starting Robot App Test...";
    
    QQmlApplicationEngine engine;
    
    qDebug() << "Creating WebSocket bridge...";
    
    // Create WebSocket bridge on stack to avoid heap allocation issues
    WebSocketBridge websocketBridge(&engine);
    
    qDebug() << "Exposing WebSocket bridge to QML...";
    
    // Expose WebSocket bridge to QML
    engine.rootContext()->setContextProperty("websocketBridge", &websocketBridge);
    
    // Load the test QML file first
    const QUrl testUrl(QStringLiteral("qrc:/TestMain.qml"));
    QObject::connect(&engine, &QQmlApplicationEngine::objectCreated,
                     &app, [testUrl](QObject *obj, const QUrl &objUrl) {
        if (!obj && testUrl == objUrl) {
            qDebug() << "Failed to load QML file:" << objUrl;
            QCoreApplication::exit(-1);
        } else if (obj) {
            qDebug() << "Successfully loaded QML file:" << objUrl;
        }
    }, Qt::QueuedConnection);
    
    qDebug() << "Loading QML file...";
    engine.load(testUrl);
    
    qDebug() << "Starting event loop...";
    return app.exec();
}
