#include "pclobjectextractor.h"
#include "ui_pclobjectextractor.h"

using namespace pcl;
using namespace std;


PCLObjectExtractor::PCLObjectExtractor(QWidget *parent) :
    QMainWindow(parent),
    mUi(new Ui::PCLObjectExtractor)
{
    mUi->setupUi(this);
    mUi->saveButton->setEnabled(false);
    mpLoadedPointCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB> );
    mpSelectedPointCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB> );
    mpPointCloudViewer.reset(new pcl::visualization::PCLVisualizer("Viewer", false));
    mpSelectionViewer.reset(new pcl::visualization::PCLVisualizer("Viewer", false));
    mFileDialog.setDefaultSuffix(QString("pcd"));

    // Set up QVTK widgets and GUI widgets
    mUi->qvtkWidget->SetRenderWindow(mpPointCloudViewer->getRenderWindow());
    mUi->qvtkWidget_2->SetRenderWindow(mpSelectionViewer->getRenderWindow());
    mpPointCloudViewer->setupInteractor(mUi->qvtkWidget->GetInteractor(),
                             mUi->qvtkWidget->GetRenderWindow());
    mpSelectionViewer->setupInteractor(mUi->qvtkWidget_2->GetInteractor(),
                             mUi->qvtkWidget_2->GetRenderWindow());
    mpPointCloudViewer->setShowFPS(false);
    mpSelectionViewer->setShowFPS(false);
    mpPointCloudViewer->addPointCloud(mpLoadedPointCloud, "cloud");
    mpSelectionViewer->addPointCloud(mpSelectedPointCloud, "cloud");
    mpPointCloudViewer->resetCamera();
    mpSelectionViewer->resetCamera();
    mpPointCloudViewer->registerPointPickingCallback(&PointSelectionCallback,
                                          this);
    mpPointCloudViewer->registerAreaPickingCallback(&AreaSelectionCallback,
                                         this);

    // Connect all gui signals/slots
    connect(this,
                     SIGNAL(PointHighlightSignal(int)),
                     this,
                     SLOT(PointHighlightSlot(int)));
    connect(this,
                     SIGNAL(AreaHighlightSignal(std::vector<int>)),
                     this,
                     SLOT(AreaHighlightSlot(std::vector<int>)));
    connect(mUi->actionExit,
                     SIGNAL(triggered()),
                     this,
                     SLOT(close()));
    connect(mUi->actionHelp,
                     SIGNAL(triggered()),
                     this,
                     SLOT(on_helpAction_triggered()));
}


PCLObjectExtractor::~PCLObjectExtractor()
{
    delete mUi;
}


void PCLObjectExtractor::PointHighlightSlot(int pointIndex)
{
    mpLoadedPointCloud->points[pointIndex].r = 255;
    mpLoadedPointCloud->points[pointIndex].g = 0;
    mpLoadedPointCloud->points[pointIndex].b = 0;
    mpPointCloudViewer->updatePointCloud(mpLoadedPointCloud, "cloud");
    mUi->qvtkWidget->update();
    pcl::PointXYZRGB selectedPoint = mpLoadedPointCloud->points[pointIndex];
    pcl::PointCloud<pcl::PointXYZRGB>::const_iterator it;
    for(it = mpSelectedPointCloud->points.begin();
            it != mpSelectedPointCloud->points.end();
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
    mpSelectedPointCloud->points.push_back(selectedPoint);
    mpSelectionViewer->updatePointCloud(mpSelectedPointCloud, "cloud");
    mpSelectionViewer->resetCamera();
    mUi->qvtkWidget_2->update();
    mNumPointsSelected = (int)mpSelectedPointCloud->points.size();
    mUi->pointsSelectedLabel->setText(QString("Object contains: "
                                                                                + QString::number(mNumPointsSelected )
                                                                                + " points"));
    if(mpSelectedPointCloud->points.size() > 0)
    {
        mUi->saveButton->setEnabled(true);
        mUi->saveButton->setText(QString("Save"));
    }
}


void PCLObjectExtractor::AreaHighlightSlot(std::vector<int> pointIndecies)
{
    for(int i = 0; i < pointIndecies.size(); i++)
    {
        mpLoadedPointCloud->points[pointIndecies.at(i)].r = 255;
        mpLoadedPointCloud->points[pointIndecies.at(i)].g = 0;
        mpLoadedPointCloud->points[pointIndecies.at(i)].b = 0;
        mpPointCloudViewer->updatePointCloud(mpLoadedPointCloud, "cloud");
        mUi->qvtkWidget->update();
        pcl::PointXYZRGB selectedPoint = mpLoadedPointCloud->points[pointIndecies.at(i)];
        pcl::PointCloud<pcl::PointXYZRGB>::const_iterator it;
        bool addPoint = true;
        for(it = mpSelectedPointCloud->points.begin();
                it != mpSelectedPointCloud->points.end();
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
            mpSelectedPointCloud->points.push_back(selectedPoint);
            mpSelectionViewer->updatePointCloud(mpSelectedPointCloud, "cloud");
            mpSelectionViewer->resetCamera();
            mUi->qvtkWidget_2->update();
        }
    }
    mNumPointsSelected = (int)mpSelectedPointCloud->points.size();
    mUi->pointsSelectedLabel->setText(QString("Object contains: "
                                                                                + QString::number(mNumPointsSelected )
                                                                                + " points"));
    if(mpSelectedPointCloud->points.size() > 0)
    {
        mUi->saveButton->setEnabled(true);
        mUi->saveButton->setText(QString("Save"));
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


void PCLObjectExtractor::on_helpAction_triggered()
{
    QMessageBox::about(this,
                                               "About PCL Object Extractor",
                                               "This program allows users to create point cloud library compatible point clouds by croping existing clouds loaded into the program. These new object models can then be saved out into the PCL .PCD format.");
}


void PCLObjectExtractor::on_loadButton_clicked()
{
    QString fileName = mFileDialog.getOpenFileName(this,
                                                                                                         tr("Load Point Cloud"),
                                                                                                         QDir::currentPath(),
                                                                                                         "PCD Files (*.pcd)");
    if(!fileName.isEmpty())
    {
        if(pcl::io::loadPCDFile(fileName.toUtf8().constData(),
                                                   *mpLoadedPointCloud) == -1)
        {
            QMessageBox::information(this,
                                                                    "Error",
                                                                     fileName + " failed to open.");
            return;
        }
        else
        {
            mpSelectedPointCloud->points.clear();
            mUi->qvtkWidget_2->update();
            mpSelectionViewer->updatePointCloud(mpSelectedPointCloud, "cloud");
            mpPointCloudViewer->updatePointCloud(mpLoadedPointCloud, "cloud");
            mpPointCloudViewer->resetCamera();
            mUi->qvtkWidget->update();
            mUi->pointCloudSourceLabel->setText(fileName);
        }
    }
}


void PCLObjectExtractor::on_saveButton_clicked()
{
    QString fileName = mFileDialog.getSaveFileName(this,
                                                                                                       tr("Save Selected Object"),
                                                                                                       QDir::currentPath(),
                                                                                                       "PCD Files (*.pcd)");
    if(!fileName.isEmpty())
    {
        if(!(fileName.endsWith(".pcd")))
        {
            fileName += ".pcd";
        }
        if(mpSelectedPointCloud->points.size() > 0)
        {
            mpSelectedPointCloud->height = 1;
            mpSelectedPointCloud->width = (int)mpSelectedPointCloud->points.size();
            mpSelectedPointCloud->is_dense = false;
            if(pcl::io::savePCDFileASCII(fileName.toUtf8().constData(),
                                                                 *mpSelectedPointCloud) == -1)
            {
                QMessageBox::information(this,
                                                                        "Error",
                                                                         fileName + " failed to save.");
                return;
            }
            mUi->saveButton->setEnabled(false);
            mUi->saveButton->setText(QString("Saved"));
        }
    }
}
