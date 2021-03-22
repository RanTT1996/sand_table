#include <QApplication>
#include "vehicleconsole.h"

int main(int argc, char *argv[])
{
    //QCoreApplication::setSetuidAllowed(true);
    QApplication a(argc, argv);
    VehicleConsole w;
    w.show();
    return a.exec();
}
