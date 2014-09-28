#include "pclobjectextractor.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    PCLObjectExtractor viewer;
    viewer.show();

    return app.exec();
}
