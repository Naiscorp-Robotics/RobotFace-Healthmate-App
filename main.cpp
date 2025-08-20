#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QDir>
#include <QDebug>
#include "websocketbridge.h"

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

    // Determine which QML file to load (use fixed Main.qml by default)
    QString qmlFile = "qrc:/Main.qml";
    
    // Check command line arguments for test modes
    for (int i = 1; i < argc; ++i) {
        QString arg = QString::fromUtf8(argv[i]);
        if (arg == "--simple" || arg == "-s") {
            qmlFile = "qrc:/MainSimple.qml";
            qDebug() << "Using simplified QML version for testing";
            break;
        }
        else if (arg == "--intermediate" || arg == "-i") {
            qmlFile = "qrc:/MainIntermediate.qml";
            qDebug() << "Using intermediate QML version with WebSocketPanel";
            break;
        }
        else if (arg == "--test" || arg == "-t") {
            qmlFile = "qrc:/TestMain.qml";
            qDebug() << "Using test QML version";
            break;
        }
        else if (arg == "--websocket" || arg == "-w") {
            qmlFile = "qrc:/MainWebSocketOnly.qml";
            qDebug() << "Using WebSocket-only QML version";
            break;
        }
        else if (arg == "--video" || arg == "-v") {
            qmlFile = "qrc:/MainVideoOnly.qml";
            qDebug() << "Using Video-only QML version";
            break;
        }
        else if (arg == "--optimized" || arg == "-o") {
            qmlFile = "qrc:/MainOptimizedVideo.qml";
            qDebug() << "Using Optimized Video QML version";
            break;
        }
        else if (arg == "--combined" || arg == "-c") {
            qmlFile = "qrc:/MainCombined.qml";
            qDebug() << "Using Combined (WebSocket + Optimized Video) QML version";
            break;
        }
        else if (arg == "--original" || arg == "-r") {
            qmlFile = "qrc:/Main.qml";
            qDebug() << "Using Original QML version (may crash)";
            break;
        }
        else if (arg == "--minimal" || arg == "-m") {
            qmlFile = "qrc:/MainMinimal.qml";
            qDebug() << "Using Minimal QML version (video only)";
            break;
        }
        else if (arg == "--simplified" || arg == "-f") {
            qmlFile = "qrc:/MainSimplified.qml";
            qDebug() << "Using Simplified QML version (video + websocket panel)";
            break;
        }
    }
    
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
