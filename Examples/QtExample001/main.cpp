#include "MainWindow.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    if (false == w.Init()) 
    {
        return EXIT_FAILURE;
    }
    w.show();
    return a.exec();
}
