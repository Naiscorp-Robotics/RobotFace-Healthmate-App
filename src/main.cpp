#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QDir>
#include <QDebug>
#include "../include/websocketbridge.h"

int main(int argc, char *argv[])
{
    // Enable high DPI support (must be set before creating QApplication)
    QGuiApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    
    QGuiApplication app(argc, argv);
    
    QQmlApplicationEngine engine;
    
    // Create WebSocket bridge with proper parent
    WebSocketBridge *websocketBridge = new WebSocketBridge(&app);
    
    // Expose WebSocket bridge to QML
    engine.rootContext()->setContextProperty("websocketBridge", websocketBridge);
    
    // Set up error handling for QML warnings and errors
    QObject::connect(&engine, &QQmlApplicationEngine::warnings,
                    [](const QList<QQmlError> &warnings) {
        for (const QQmlError &error : warnings) {
            qWarning() << "QML Warning:" << error.toString();
        }
    });

    // Use the Main.qml file
    QString qmlFile = "qrc:/Main.qml";
    qDebug() << "Loading Main.qml interface";
    
    // Load the selected QML file with try-catch for any C++ exceptions
    const QUrl url(qmlFile);
    
    QObject::connect(&engine, &QQmlApplicationEngine::objectCreated,
                     &app, [url](QObject *obj, const QUrl &objUrl) {
        if (!obj && url == objUrl) {
            qDebug() << "Failed to load QML file:" << url;
            QCoreApplication::exit(-1);
        } else {
            qDebug() << "QML loaded successfully";
        }
    }, Qt::QueuedConnection);
    
    try {
        qDebug() << "About to load QML file:" << url.toString();
        engine.load(url);
        qDebug() << "QML file loaded, checking for errors...";
        
        // Check if any root objects were created
        if (engine.rootObjects().isEmpty()) {
            qCritical() << "No root objects created - QML loading failed!";
            return -1;
        }
        
        qDebug() << "QML engine initialized successfully with" 
                 << engine.rootObjects().size() << "root objects";
    } 
    catch (const std::exception& e) {
        qCritical() << "Exception during QML loading:" << e.what();
        return -1;
    }
    catch (...) {
        qCritical() << "Unknown exception during QML loading";
        return -1;
    }
    
    return app.exec();
}
