#include "pclobjectextractor.h"
#include "ui_pclobjectextractor.h"

PCLObjectExtractor::PCLObjectExtractor(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::PCLObjectExtractor)
{
    ui->setupUi(this);
    mCloud.reset(new PointCloudT);
    mCloud->points.resize(200);
    for(size_t i = 0; i < mCloud->points.size(); i++)
    {
        mCloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        mCloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        mCloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
        mCloud->points[i].r = 255;
        mCloud->points[i].g = 0;
        mCloud->points[i].b = 0;
    }
    // Set up QVTK window
    mViewer.reset(new pcl::visualization::PCLVisualizer("Viewer", false));
    ui->qvtkWidget->SetRenderWindow(mViewer->getRenderWindow());
    mViewer->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
    ui->qvtkWidget->update();

    mViewer->addPointCloud(mCloud, "cloud");
    mViewer->resetCamera();
    ui->qvtkWidget->update();
}

PCLObjectExtractor::~PCLObjectExtractor()
{
    delete ui;
}
