#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QDir>
#include "websocketbridge.h"
#include "tsssocketbridge.h"

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);
    
    QQmlApplicationEngine engine;
    
    // Set the working directory to the application directory
    QDir::setCurrent(QCoreApplication::applicationDirPath());
    
    // Create and register WebSocket bridge
    WebSocketBridge *websocketBridge = new WebSocketBridge(&engine);
    
    // Create and register TSS Socket bridge
    TSSSocketBridge *tssSocketBridge = new TSSSocketBridge(&engine);
    
    // Expose WebSocket bridge to QML
    engine.rootContext()->setContextProperty("websocketBridge", websocketBridge);
    
    // Expose TSS Socket bridge to QML
    engine.rootContext()->setContextProperty("tssSocketBridge", tssSocketBridge);
    
    // Load the main QML file
    const QUrl url(QStringLiteral("qrc:/Main.qml"));
    QObject::connect(&engine, &QQmlApplicationEngine::objectCreated,
                     &app, [url](QObject *obj, const QUrl &objUrl) {
        if (!obj && url == objUrl)
            QCoreApplication::exit(-1);
    }, Qt::QueuedConnection);
    
    engine.load(url);
    
    return app.exec();
}
