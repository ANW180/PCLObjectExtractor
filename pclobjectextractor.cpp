#include "pclobjectextractor.h"
#include "ui_pclobjectextractor.h"

using namespace pcl;
using namespace std;


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
    mViewer->setupInteractor(ui->qvtkWidget->GetInteractor(),
                             ui->qvtkWidget->GetRenderWindow());
    ui->qvtkWidget->update();

    mViewer->setShowFPS(false);
    mViewer->addPointCloud(mCloud, "cloud");
    mViewer->resetCamera();
    mViewer->registerPointPickingCallback(&PointSelectionCallback,
                                          this);
    mViewer->registerAreaPickingCallback(&AreaSelectionCallback,
                                         this);
    connect(this,
            SIGNAL(PointHighlightSignal(int)),
            this,
            SLOT(PointHighlightSlot(int)));
    connect(this,
            SIGNAL(AreaHighlightSignal(std::vector<int>)),
            this,
            SLOT(AreaHighlightSlot(std::vector<int>)));
    ui->qvtkWidget->update();
}


PCLObjectExtractor::~PCLObjectExtractor()
{
    delete ui;
}


void PCLObjectExtractor::PointHighlightSlot(int pointIndex)
{
    mCloud->points[pointIndex].r = 255;
    mCloud->points[pointIndex].g = 0;
    mCloud->points[pointIndex].b = 0;
    mViewer->updatePointCloud(mCloud, "cloud");
    ui->qvtkWidget->update();
}


void PCLObjectExtractor::AreaHighlightSlot(std::vector<int> pointIndecies)
{
    for(int i = 0; i < pointIndecies.size(); i++)
    {
        mCloud->points[pointIndecies.at(i)].r = 0;
        mCloud->points[pointIndecies.at(i)].g = 255;
        mCloud->points[pointIndecies.at(i)].b = 0;
        mViewer->updatePointCloud(mCloud, "cloud");
        ui->qvtkWidget->update();
    }
}


void PCLObjectExtractor::PointSelectionCallback(
                                const visualization::PointPickingEvent& event,
                                void* args)
{
    PCLObjectExtractor *ui = (PCLObjectExtractor*) args;
    float x, y, z;
    if(event.getPointIndex() == -1)
    {
        ui->statusBar()->showMessage(tr("No point was clicked"));
    }
    else
    {
        event.getPoint(x, y, z);
        ui->statusBar()->showMessage(
                    QString("X: %1 Y: %2 Z: %3")
                    .arg(QString::number(x, 'g', 3))
                    .arg(QString::number(y, 'g', 3))
                    .arg(QString::number(z, 'g', 3)));
        ui->PointHighlightSignal(event.getPointIndex());
    }
}


void PCLObjectExtractor::AreaSelectionCallback(
                                const visualization::AreaPickingEvent &event,
                                void *args)
{
    PCLObjectExtractor *ui = (PCLObjectExtractor*) args;
    vector<int> indicies;
    if(!event.getPointsIndices(indicies))
    {
        ui->statusBar()->showMessage(tr("No area was selected"));
    }
    else
    {
        ui->statusBar()->showMessage(
                    QString("%1 points selected")
                    .arg(indicies.size()));
        ui->AreaHighlightSignal(indicies);
    }
}
