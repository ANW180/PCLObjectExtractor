#include "pclobjectextractor.h"
#include "ui_pclobjectextractor.h"

using namespace pcl;
using namespace std;


PCLObjectExtractor::PCLObjectExtractor(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::PCLObjectExtractor)
{
    ui->setupUi(this);

    // Create random point cloud for testing.
    mLoadedPointCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB> );
    mSelectedPointCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB> );
    mLoadedPointCloud->points.resize(200);
    for(size_t i = 0; i < mLoadedPointCloud->points.size(); i++)
    {
        mLoadedPointCloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        mLoadedPointCloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        mLoadedPointCloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
        mLoadedPointCloud->points[i].r = 255;
        mLoadedPointCloud->points[i].g = 0;
        mLoadedPointCloud->points[i].b = 0;
    }

    // Set up QVTK widgets
    mPointCloudViewer.reset(new pcl::visualization::PCLVisualizer("Viewer", false));
    mSelectionViewer.reset(new pcl::visualization::PCLVisualizer("Viewer", false));
    ui->qvtkWidget->SetRenderWindow(mPointCloudViewer->getRenderWindow());
    ui->qvtkWidget_2->SetRenderWindow(mSelectionViewer->getRenderWindow());
    mPointCloudViewer->setupInteractor(ui->qvtkWidget->GetInteractor(),
                             ui->qvtkWidget->GetRenderWindow());
    mSelectionViewer->setupInteractor(ui->qvtkWidget_2->GetInteractor(),
                             ui->qvtkWidget_2->GetRenderWindow());
    mPointCloudViewer->setShowFPS(false);
    mSelectionViewer->setShowFPS(false);
    mPointCloudViewer->addPointCloud(mLoadedPointCloud, "cloud");
    mSelectionViewer->addPointCloud(mSelectedPointCloud, "cloud");
    mPointCloudViewer->resetCamera();
    mSelectionViewer->resetCamera();
    mPointCloudViewer->registerPointPickingCallback(&PointSelectionCallback,
                                          this);
    mPointCloudViewer->registerAreaPickingCallback(&AreaSelectionCallback,
                                         this);
    connect(this,
            SIGNAL(PointHighlightSignal(int)),
            this,
            SLOT(PointHighlightSlot(int)));
    connect(this,
            SIGNAL(AreaHighlightSignal(std::vector<int>)),
            this,
            SLOT(AreaHighlightSlot(std::vector<int>)));
}


PCLObjectExtractor::~PCLObjectExtractor()
{
    delete ui;
}


void PCLObjectExtractor::PointHighlightSlot(int pointIndex)
{
    mLoadedPointCloud->points[pointIndex].r = 0;
    mLoadedPointCloud->points[pointIndex].g = 255;
    mLoadedPointCloud->points[pointIndex].b = 0;
    mPointCloudViewer->updatePointCloud(mLoadedPointCloud, "cloud");
    ui->qvtkWidget->update();
    pcl::PointXYZRGB selectedPoint = mLoadedPointCloud->points[pointIndex];
    selectedPoint.g = 255;
    pcl::PointCloud<pcl::PointXYZRGB>::const_iterator it;
    for(it = mSelectedPointCloud->points.begin();
            it != mSelectedPointCloud->points.end();
            it++)
    {
        // Same point
        if(fabs((*it).x - selectedPoint.x) < 0.00001 &&
            fabs((*it).y - selectedPoint.y) < 0.00001 &&
            fabs((*it).z - selectedPoint.z) < 0.00001)
        {
            return;
        }
    }
    mSelectedPointCloud->points.push_back(selectedPoint);
    mSelectionViewer->updatePointCloud(mSelectedPointCloud, "cloud");
    mSelectionViewer->resetCamera();
    ui->qvtkWidget_2->update();
}


void PCLObjectExtractor::AreaHighlightSlot(std::vector<int> pointIndecies)
{
    for(int i = 0; i < pointIndecies.size(); i++)
    {
        mLoadedPointCloud->points[pointIndecies.at(i)].r = 0;
        mLoadedPointCloud->points[pointIndecies.at(i)].g = 255;
        mLoadedPointCloud->points[pointIndecies.at(i)].b = 0;
        mPointCloudViewer->updatePointCloud(mLoadedPointCloud, "cloud");
        ui->qvtkWidget->update();
        pcl::PointXYZRGB selectedPoint = mLoadedPointCloud->points[pointIndecies.at(i)];
        selectedPoint.g = 255;
        pcl::PointCloud<pcl::PointXYZRGB>::const_iterator it;
        bool addPoint = true;
        for(it = mSelectedPointCloud->points.begin();
                it != mSelectedPointCloud->points.end();
                it++)
        {
            // Same point
            if(fabs((*it).x - selectedPoint.x) < 0.00001 &&
                fabs((*it).y - selectedPoint.y) < 0.00001 &&
                fabs((*it).z - selectedPoint.z) < 0.00001)
            {
                addPoint = false;
            }
        }
        if(addPoint)
        {
            mSelectedPointCloud->points.push_back(selectedPoint);
            mSelectionViewer->updatePointCloud(mSelectedPointCloud, "cloud");
            mSelectionViewer->resetCamera();
            ui->qvtkWidget_2->update();
        }
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
        ui->statusBar()->showMessage(tr("No points were selected"));
    }
    else
    {
        ui->statusBar()->showMessage(
                    QString("%1 points selected")
                    .arg(indicies.size()));
        ui->AreaHighlightSignal(indicies);
    }
}
