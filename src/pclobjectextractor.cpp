#include "pclobjectextractor.h"
#include "ui_pclobjectextractor.h"

using namespace pcl;

PCLObjectExtractor::PCLObjectExtractor(QWidget *parent) :
    QMainWindow(parent),
    mUi(new Ui::PCLObjectExtractor)
{
    mUi->setupUi(this);
    mUi->saveButton->setEnabled(false);
    mpLoadedPointCloud.reset(new PCLPointCloud2);
    mpSelectedPointCloud.reset(new PCLPointCloud2 );
    mpPointCloudViewer.reset(new visualization::PCLVisualizer("Viewer", false));
    mpSelectionViewer.reset(new visualization::PCLVisualizer("Viewer", false));
    mFileDialog.setDefaultSuffix(QString("pcd"));

    // Set up QVTK widgets and viewers.
    mUi->qvtkWidget->SetRenderWindow(mpPointCloudViewer->getRenderWindow());
    mUi->qvtkWidget_2->SetRenderWindow(mpSelectionViewer->getRenderWindow());
    mpPointCloudViewer->setupInteractor(mUi->qvtkWidget->GetInteractor(),
                                        mUi->qvtkWidget->GetRenderWindow());
    mpSelectionViewer->setupInteractor(mUi->qvtkWidget_2->GetInteractor(),
                                       mUi->qvtkWidget_2->GetRenderWindow());
    mpPointCloudViewer->setShowFPS(false);
    mpSelectionViewer->setShowFPS(false);
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
            SLOT(on_actionHelp_triggered()));
}


PCLObjectExtractor::~PCLObjectExtractor()
{
    delete mUi;
}


void PCLObjectExtractor::PointHighlightSlot(int pointIndex)
{
    PointXYZ selectedPoint = mpLoadedPointCloud->points[pointIndex];
    PointCloud<PointXYZ>::const_iterator it;
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
    mUi->pointsSelectedLabel->setText(
                QString(QString::number(mNumPointsSelected)
                        + " points"));
    if(mpSelectedPointCloud->points.size() > 0)
    {
        mUi->saveButton->setEnabled(true);
        mUi->saveButton->setText(QString("Save"));
    }
}


void PCLObjectExtractor::AreaHighlightSlot(std::vector<int> pointIndices)
{
    PointCloud<PointXYZ> pointsToAdd;
    bool errorOccured = false;
    for(int i = 0; i < pointIndices.size(); i++)
    {
        int index = pointIndices.at(i);
        if(index <= mpLoadedPointCloud->points.size() &&
                index >= 0)
        {
            pointsToAdd.push_back(
                        mpLoadedPointCloud->points[index]);
        }
        else
        {
            errorOccured = true;
        }
    }
    if(errorOccured)
    {
        QMessageBox::information(this,
                                 "VTK Error",
                                 "An error occured in selecting some points.");
    }
    PointCloud<PointXYZ>::iterator it1;
    for(it1 = pointsToAdd.begin();
        it1 != pointsToAdd.end();
        it1++)
    {
        bool alreadyAdded = false;
        PointCloud<PointXYZ>::const_iterator it2;
        for(it2 = mpSelectedPointCloud->points.begin();
            it2 != mpSelectedPointCloud->points.end();
            it2++)
        {
            // Same point
            if(fabs((*it2).x -(*it1).x) < 0.00001 &&
                    fabs((*it2).y - (*it1).y) < 0.00001 &&
                    fabs((*it2).z - (*it1).z) < 0.00001)
            {
                alreadyAdded = true;
                break;
            }
        }
        if(!alreadyAdded)
        {
            mpSelectedPointCloud->points.push_back(*it1);
        }
    }
    mpSelectionViewer->updatePointCloud(mpSelectedPointCloud, "cloud");
    mpSelectionViewer->resetCamera();
    mUi->qvtkWidget_2->update();
    mNumPointsSelected = (int)mpSelectedPointCloud->points.size();
    mUi->pointsSelectedLabel->setText(
                QString(QString::number(mNumPointsSelected)
                        + " points"));
    if(mpSelectedPointCloud->points.size() > 0)
    {
        mUi->saveButton->setEnabled(true);
        mUi->saveButton->setText(QString("Save"));
    }
}


void PCLObjectExtractor::PointRemoveSlot(int pointIndex)
{
    PointXYZ selectedPoint = mpSelectedPointCloud->points[pointIndex];
    PointCloud<PointXYZ>::iterator it;
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
            mUi->pointsSelectedLabel->setText(
                        QString(QString::number(mNumPointsSelected)
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
    PointCloud<PointXYZ> pointsToRemove;
    bool errorOccured = false;
    for(int i = 0; i < pointIndices.size(); i++)
    {
        int index = pointIndices.at(i);
        if(index <= mpSelectedPointCloud->points.size() &&
                index >= 0)
        {
            pointsToRemove.push_back(
                        mpSelectedPointCloud->points[index]);
        }
        else
        {
            errorOccured = true;
        }
    }
    if(errorOccured)
    {
        QMessageBox::information(this,
                                 "VTK Error",
                                 "There was an error in removing some points.");
    }
    PointCloud<PointXYZ>::iterator it1;
    for(it1 = pointsToRemove.begin();
        it1 < pointsToRemove.end();
        it1++)
    {
        PointCloud<PointXYZ>::iterator it2;
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
    mUi->pointsSelectedLabel->setText(
                QString(QString::number(mNumPointsSelected)
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


void PCLObjectExtractor::on_actionHelp_triggered()
{
    QMessageBox::about(this,
                       "About PCL Object Extractor",
                       QString("This program allows users to create crop ")
                       + "objects from existing point clouds by croping loaded "
                       + "PCD files and saving them out in the program. To "
                       + "select points first load the desired PCD file then "
                       + "click and make active the point cloud visualizer. "
                       + "While holding shift and left clicking you can select "
                       + "individual points or you can push 'x' to enable area "
                       + "selection. Deselect points by doing the same in the "
                       + "selected object visualizer window.");
}


void PCLObjectExtractor::on_actionExit_triggered()
{
    close();
}


void PCLObjectExtractor::on_loadButton_clicked()
{
    mUi->colorDisplayBox->setEnabled(false);
    mUi->normalsDisplayBox->setEnabled(false);
    mUi->intensityDisplayBox->setEnabled(false);
    mUi->colorSaveBox->setEnabled(false);
    mUi->normalsSaveBox->setEnabled(false);
    mUi->intensitySaveBox->setEnabled(false);
    QString fileName = mFileDialog.getOpenFileName(this,
                                                   tr("Load Point Cloud"),
                                                   QDir::currentPath(),
                                                   "PCD Files (*.pcd)");
    if(!fileName.isEmpty())
    {
        PCLPointCloud2 cloud;
        Eigen::Vector4f origin;
        Eigen::Quaternionf orientation;
        int fileVersion, dataType;
        unsigned int dataIdx;
        if(mPCDReader.readHeader(fileName.toUtf8().constData(),
                                 cloud,
                                 origin,
                                 orientation,
                                 fileVersion,
                                 dataType,
                                 dataIdx) < 0)
        {
            QMessageBox::information(this,
                                     "Error",
                                     fileName + " failed to open.");
            return;
        }
        std::string type;
        for (size_t i = 0; i < cloud.fields.size(); i++)
        {
            type += cloud.fields.at(i).name;

        }
        if(type.find(std::string("xyz")) == std::string::npos)
        {
            QMessageBox::information(this,
                                     "Error",
                                     fileName + " does not contain 3D points");
            return;
        }
        if(type.find(std::string("rgb")) != std::string::npos)
        {
            mUi->colorDisplayBox->setEnabled(true);
            mUi->colorSaveBox->setEnabled(true);
        }
        if(type.find(std::string("normal_xnormal_ynormal_z")) !=
                std::string::npos)
        {
            mUi->normalsDisplayBox->setEnabled(true);
            mUi->normalsSaveBox->setEnabled(true);
        }
        if(type.find(std::string("intensity")) !=
                std::string::npos)
        {
            mUi->intensityDisplayBox->setEnabled(true);
            mUi->intensitySaveBox->setEnabled(true);
        }
        if(mPCDReader.read(fileName.toUtf8().constData(),
                           cloud) < 0)
        {
            QMessageBox::information(this,
                                     "Error",
                                     fileName + " failed to open.");
            return;
        }
        fromPCLPointCloud2(cloud, *mpLoadedPointCloud);
        mpSelectedPointCloud->points.clear();
        mpSelectionViewer->updatePointCloud(mpSelectedPointCloud, "cloud");
        mpPointCloudViewer->updatePointCloud(mpLoadedPointCloud, "cloud");
        mpPointCloudViewer->resetCamera();
        mpSelectionViewer->resetCamera();
        mUi->qvtkWidget->update();
        mUi->qvtkWidget_2->update();
        mUi->pointCloudSourceLabel->setText(fileName.split("/").last());
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
            mpSelectedPointCloud->width = (int)
                    mpSelectedPointCloud->points.size();
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
