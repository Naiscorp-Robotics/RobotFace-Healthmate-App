#include "mainwindow.h"
#include <QApplication>
#include <QtWebSockets>  // Required for QWebSocket

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    // Set application metadata
    QApplication::setApplicationName("Socket Demo");
    QApplication::setApplicationVersion("1.0");
    QApplication::setOrganizationName("Your Company");

    MainWindow w;
    w.show();

    return a.exec();
}
