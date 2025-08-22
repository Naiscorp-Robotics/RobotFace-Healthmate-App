#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    // Thiết lập thông tin ứng dụng
    QApplication::setApplicationName("Audio Stream Recorder");
    QApplication::setApplicationVersion("1.0");
    QApplication::setOrganizationName("YourCompany");

    MainWindow w;
    w.show();

    return a.exec();
}
