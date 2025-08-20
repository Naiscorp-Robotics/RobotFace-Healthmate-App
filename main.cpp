#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QDir>
#include "websocketbridge.h"
#include "audiomanager.h"

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);

    QQmlApplicationEngine engine;

    // Set the working directory to the application directory
    QDir::setCurrent(QCoreApplication::applicationDirPath());

    // Create and register WebSocket bridge
    WebSocketBridge *websocketBridge = new WebSocketBridge(&engine);

    // Expose WebSocket bridge to QML
    engine.rootContext()->setContextProperty("websocketBridge", websocketBridge);

    // Đăng ký AudioManager với QML
    qmlRegisterType<AudioManager>("Audio", 1, 0, "AudioManager");

    // Load the main QML file
    const QUrl url(QStringLiteral("qrc:/Main.qml"));
    QObject::connect(&engine, &QQmlApplicationEngine::objectCreated,
                     &app, [url](QObject *obj, const QUrl &objUrl) {
                         if (!obj && url == objUrl)
                             QCoreApplication::exit(-1);
                     }, Qt::QueuedConnection);

    engine.load(QUrl(QStringLiteral("qrc:/Main.qml")));

    return app.exec();
}
