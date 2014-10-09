#include "src/qt/pclobjectextractor.h"
#include <QApplication>
#include <QtPlugin>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    PCLObjectExtractor viewer;
    app.setWindowIcon(QIcon("icons/icon.png"));
    viewer.show();

    return app.exec();
}
