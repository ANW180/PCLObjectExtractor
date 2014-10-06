#include "pclobjectextractor.h"
#include "ui_pclobjectextractor.h"

using namespace pcl;

PCLObjectExtractor::PCLObjectExtractor(QWidget *parent) :
    QMainWindow(parent),
    mUi(new Ui::PCLObjectExtractor)
{
    mUi->setupUi(this);
    mUi->saveButton->setEnabled(false);
    mpLoadedPointCloud.reset(new PointCloud<PointXYZRGB> );
    mpSelectedPointCloud.reset(new PointCloud<PointXYZRGB> );
    mpPointCloudViewer.reset(new visualization::PCLVisualizer("Viewer", false));
    mpSelectionViewer.reset(new visualization::PCLVisualizer("Viewer", false));
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
    mpSelectionViewer->registerPointPickingCallback(&PointRemoveCallback,
                                                    this);
    mpSelectionViewer->registerAreaPickingCallback(&AreaRemoveCallback,
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
    connect(this,
            SIGNAL(PointRemoveSignal(int)),
            this,
            SLOT(PointRemoveSlot(int)));
    connect(this,
            SIGNAL(AreaRemoveSignal(std::vector<int>)),
            this,
            SLOT(AreaRemoveSlot(std::vector<int>)));
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
    PointXYZRGB selectedPoint = mpLoadedPointCloud->points[pointIndex];
    PointCloud<PointXYZRGB>::const_iterator it;
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
                                              + QString::number(mNumPointsSelected)
                                              + " points"));
    if(mpSelectedPointCloud->points.size() > 0)
    {
        mUi->saveButton->setEnabled(true);
        mUi->saveButton->setText(QString("Save"));
    }
}


void PCLObjectExtractor::AreaHighlightSlot(std::vector<int> pointIndices)
{
    for(int i = 0; i < pointIndices.size(); i++)
    {
        PointXYZRGB selectedPoint = mpLoadedPointCloud->points[pointIndices.at(i)];
        PointCloud<PointXYZRGB>::const_iterator it;
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
        }
    }
    mpSelectionViewer->updatePointCloud(mpSelectedPointCloud, "cloud");
    mpSelectionViewer->resetCamera();
    mUi->qvtkWidget_2->update();
    mNumPointsSelected = (int)mpSelectedPointCloud->points.size();
    mUi->pointsSelectedLabel->setText(QString("Object contains: "
                                              + QString::number(mNumPointsSelected)
                                              + " points"));
    if(mpSelectedPointCloud->points.size() > 0)
    {
        mUi->saveButton->setEnabled(true);
        mUi->saveButton->setText(QString("Save"));
    }
}


void PCLObjectExtractor::PointRemoveSlot(int pointIndex)
{
    PointXYZRGB selectedPoint = mpSelectedPointCloud->points[pointIndex];
    PointCloud<PointXYZRGB>::iterator it;
    for(it = mpSelectedPointCloud->points.begin();
        it != mpSelectedPointCloud->points.end();
        it++)
    {
        // Same point
        if(fabs((*it).x - selectedPoint.x) < 0.00001 &&
                fabs((*it).y - selectedPoint.y) < 0.00001 &&
                fabs((*it).z - selectedPoint.z) < 0.00001)
        {
            mpSelectedPointCloud->points.erase(it);
            mpSelectionViewer->updatePointCloud(mpSelectedPointCloud, "cloud");
            mpSelectionViewer->resetCamera();
            mUi->qvtkWidget_2->update();
            mNumPointsSelected = (int)mpSelectedPointCloud->points.size();
            mUi->pointsSelectedLabel->setText(QString("Object contains: "
                                                      + QString::number(mNumPointsSelected)
                                                      + " points"));
            if(mpSelectedPointCloud->points.size() > 0)
            {
                mUi->saveButton->setEnabled(true);
                mUi->saveButton->setText(QString("Save"));
            }
            else if(mpSelectedPointCloud->points.size() == 0)
            {
                mUi->saveButton->setEnabled(false);
            }
            return;
        }
    }
}


void PCLObjectExtractor::AreaRemoveSlot(std::vector<int> pointIndices)
{
    PointCloud<PointXYZRGB> pointsToRemove;
    for(int i = 0; i < pointIndices.size(); i++)
    {
        pointsToRemove.push_back(
                    mpSelectedPointCloud->points[pointIndices.at(i)]);
    }
    PointCloud<PointXYZRGB>::iterator it1;
    for(it1 = pointsToRemove.begin();
        it1 < pointsToRemove.end();
        it1++)
    {
        PointCloud<PointXYZRGB>::iterator it2;
        for(it2 = mpSelectedPointCloud->points.begin();
            it2 != mpSelectedPointCloud->points.end();)
        {
            // Same point
            if(fabs((*it2).x -(*it1).x) < 0.00001 &&
                    fabs((*it2).y - (*it1).y) < 0.00001 &&
                    fabs((*it2).z - (*it1).z) < 0.00001)
            {
                it2 = mpSelectedPointCloud->points.erase(it2);
                break;
            }
            else
            {
                it2++;
            }
        }
    }
    mpSelectionViewer->updatePointCloud(mpSelectedPointCloud, "cloud");
    mpSelectionViewer->resetCamera();
    mUi->qvtkWidget_2->update();
    mNumPointsSelected = (int)mpSelectedPointCloud->points.size();
    mUi->pointsSelectedLabel->setText(QString("Object contains: "
                                              + QString::number(mNumPointsSelected)
                                              + " points"));
    if(mpSelectedPointCloud->points.size() > 0)
    {
        mUi->saveButton->setEnabled(true);
        mUi->saveButton->setText(QString("Save"));
    }
    else if(mpSelectedPointCloud->points.size() == 0)
    {
        mUi->saveButton->setEnabled(false);
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
        if(io::loadPCDFile(fileName.toUtf8().constData(),
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
            mpSelectionViewer->updatePointCloud(mpSelectedPointCloud, "cloud");
            mpPointCloudViewer->updatePointCloud(mpLoadedPointCloud, "cloud");
            mpPointCloudViewer->resetCamera();
            mpSelectionViewer->resetCamera();
            mUi->qvtkWidget->update();
            mUi->qvtkWidget_2->update();
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
            if(io::savePCDFileASCII(fileName.toUtf8().constData(),
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
        emit ui->PointHighlightSignal(event.getPointIndex());
    }
}


void PCLObjectExtractor::AreaSelectionCallback(
        const visualization::AreaPickingEvent &event,
        void *args)
{
    PCLObjectExtractor *ui = (PCLObjectExtractor*) args;
    std::vector<int> indices;
    if(!event.getPointsIndices(indices))
    {
        ui->statusBar()->showMessage(tr("No points were selected"));
    }
    else
    {
        ui->statusBar()->showMessage(
                    QString("%1 points selected")
                    .arg(indices.size()));
        emit ui->AreaHighlightSignal(indices);
    }
}


void PCLObjectExtractor::PointRemoveCallback(
        const visualization::PointPickingEvent &event,
        void *args)
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
        emit ui->PointRemoveSignal(event.getPointIndex());
    }
}


void PCLObjectExtractor::AreaRemoveCallback(
        const visualization::AreaPickingEvent &event,
        void *args)
{
    PCLObjectExtractor *ui = (PCLObjectExtractor*) args;
    std::vector<int> indices;
    if(!event.getPointsIndices(indices))
    {
        ui->statusBar()->showMessage(tr("No points were selected"));
    }
    else
    {
        ui->statusBar()->showMessage(
                    QString("%1 points selected")
                    .arg(indices.size()));
        emit ui->AreaRemoveSignal(indices);
    }
}
