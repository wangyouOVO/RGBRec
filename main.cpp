#include "recon.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Recon w;
    w.show();

    return a.exec();
}
