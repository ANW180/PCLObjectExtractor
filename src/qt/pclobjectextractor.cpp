#include "pclobjectextractor.h"
#include "ui_pclobjectextractor.h"

using namespace pcl;


/**
 * @brief PCLObjectExtractor::PCLObjectExtractor
 * @param parent
 */
PCLObjectExtractor::PCLObjectExtractor(QWidget *parent) :
    QMainWindow(parent),
    mUi(new Ui::PCLObjectExtractor)
{
    mUi->setupUi(this);
    mpLoadedCloud.reset(new PointCloud<PointXYZ>);
    mpSelectedCloud.reset(new PointCloud<PointXYZ>);
    mpLoadedViewer.reset(new visualization::PCLVisualizer("Viewer", false));
    mpSelectedViewer.reset(new visualization::PCLVisualizer("Viewer", false));
    mFileDialog.setDefaultSuffix(QString("pcd"));

    // Set up QVTK widgets and viewers.
    mUi->qvtkWidget->SetRenderWindow(mpLoadedViewer->getRenderWindow());
    mUi->qvtkWidget_2->SetRenderWindow(mpSelectedViewer->getRenderWindow());
    mpLoadedViewer->setupInteractor(mUi->qvtkWidget->GetInteractor(),
                                    mUi->qvtkWidget->GetRenderWindow());
    mpSelectedViewer->setupInteractor(mUi->qvtkWidget_2->GetInteractor(),
                                      mUi->qvtkWidget_2->GetRenderWindow());
    mpLoadedViewer->addPointCloud(mpLoadedCloud, "");
    mpSelectedViewer->addPointCloud(mpSelectedCloud, "");
    mpLoadedViewer->setShowFPS(false);
    mpSelectedViewer->setShowFPS(false);
    mpLoadedViewer->registerPointPickingCallback(&PointSelectionCallback, this);
    mpLoadedViewer->registerAreaPickingCallback(&AreaSelectionCallback, this);
    mpSelectedViewer->registerPointPickingCallback(&PointRemoveCallback, this);
    mpSelectedViewer->registerAreaPickingCallback(&AreaRemoveCallback, this);

    // Connect custom signals/slots
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
}


/**
 * @brief PCLObjectExtractor::~PCLObjectExtractor
 */
PCLObjectExtractor::~PCLObjectExtractor()
{
    delete mUi;
}


/**
 * @brief PCLObjectExtractor::PointHighlightSlot
 * @param pointIndex
 */
void PCLObjectExtractor::PointHighlightSlot(int pointIndex)
{
    PointXYZ selectedPoint = mpLoadedCloud->points[pointIndex];
    PointCloud<PointXYZ>::const_iterator it;
    if(mpSelectedCloud->points.size() > 0)
    {
        for(it = mpSelectedCloud->points.begin();
            it != mpSelectedCloud->points.end();
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
    }
    mpSelectedCloud->points.push_back(selectedPoint);
    mpSelectedViewer->updatePointCloud(mpSelectedCloud, "");
    mpSelectedViewer->resetCamera();
    mUi->qvtkWidget_2->update();
    mNumPointsSelected = (int)mpSelectedCloud->points.size();
    mUi->pointsSelectedLabel->setText(
                QString(QString::number(mNumPointsSelected)
                        + " points"));
    mUi->highlightProgressBar->setValue(100);
    if(mpSelectedCloud->points.size() > 0)
    {
        mUi->saveButton->setEnabled(true);
        mUi->saveButton->setText(QString("Save"));
    }
}


/**
 * @brief PCLObjectExtractor::AreaHighlightSlot
 * @param pointIndices
 */
void PCLObjectExtractor::AreaHighlightSlot(std::vector<int> pointIndices)
{
    PointCloud<PointXYZ> pointsToAdd;
    bool errorOccured = false;
    for(int i = 0; i < pointIndices.size(); i++)
    {
        int index = pointIndices.at(i);
        if(index <= mpLoadedCloud->points.size() &&
                index >= 0)
        {
            pointsToAdd.push_back(
                        mpLoadedCloud->points[index]);
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
    mUi->highlightProgressBar->setValue(0);
    if(mpSelectedCloud->points.size() > 0)
    {
        PointCloud<PointXYZ>::iterator it1;
        for(it1 = pointsToAdd.begin();
            it1 != pointsToAdd.end();
            it1++)
        {
            mUi->highlightProgressBar->setValue(int(float(it1 -
                                                          pointsToAdd.begin()) /
                                                    (float)(pointsToAdd.size())
                                                    * 100.));
            bool alreadyAdded = false;
            PointCloud<PointXYZ>::const_iterator it2;
            for(it2 = mpSelectedCloud->points.begin();
                it2 != mpSelectedCloud->points.end();
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
                mpSelectedCloud->points.push_back(*it1);
            }
        }
    }
    else
    {
        PointCloud<PointXYZ>::iterator it1;
        for(it1 = pointsToAdd.begin();
            it1 != pointsToAdd.end();
            it1++)
        {
            mUi->highlightProgressBar->setValue(int(float(it1 -
                                                          pointsToAdd.begin()) /
                                                    (float)(pointsToAdd.size())
                                                    * 100.));
            mpSelectedCloud->points.push_back(*it1);
        }
    }
    mUi->highlightProgressBar->setValue(100);
    mNumPointsSelected = (int)mpSelectedCloud->points.size();
    mUi->pointsSelectedLabel->setText(
                QString(QString::number(mNumPointsSelected)
                        + " points"));
    mpSelectedViewer->updatePointCloud(mpSelectedCloud, "");
    mpSelectedViewer->resetCamera();
    mUi->qvtkWidget_2->update();
    if(mpSelectedCloud->points.size() > 0)
    {
        mUi->saveButton->setEnabled(true);
        mUi->saveButton->setText(QString("Save"));
    }
}


/**
 * @brief PCLObjectExtractor::PointRemoveSlot
 * @param pointIndex
 */
void PCLObjectExtractor::PointRemoveSlot(int pointIndex)
{
    PointXYZ selectedPoint = mpSelectedCloud->points[pointIndex];
    PointCloud<PointXYZ>::iterator it;
    for(it = mpSelectedCloud->points.begin();
        it != mpSelectedCloud->points.end();
        it++)
    {
        // Same point
        if(fabs((*it).x - selectedPoint.x) < 0.00001 &&
                fabs((*it).y - selectedPoint.y) < 0.00001 &&
                fabs((*it).z - selectedPoint.z) < 0.00001)
        {
            mpSelectedCloud->points.erase(it);
            mpSelectedViewer->updatePointCloud(mpSelectedCloud, "");
            mpSelectedViewer->resetCamera();
            mUi->qvtkWidget_2->update();
            mNumPointsSelected = (int)mpSelectedCloud->points.size();
            mUi->pointsSelectedLabel->setText(
                        QString(QString::number(mNumPointsSelected)
                                + " points"));
            mUi->removeProgressBar->setValue(100);
            if(mpSelectedCloud->points.size() > 0)
            {
                mUi->saveButton->setEnabled(true);
                mUi->saveButton->setText(QString("Save"));
            }
            else if(mpSelectedCloud->points.size() == 0)
            {
                mUi->saveButton->setEnabled(false);
            }
            return;
        }
    }
}


/**
 * @brief PCLObjectExtractor::AreaRemoveSlot
 * @param pointIndices
 */
void PCLObjectExtractor::AreaRemoveSlot(std::vector<int> pointIndices)
{
    PointCloud<PointXYZ> pointsToRemove;
    bool errorOccured = false;
    for(int i = 0; i < pointIndices.size(); i++)
    {
        int index = pointIndices.at(i);
        if(index <= mpSelectedCloud->points.size() &&
                index >= 0)
        {
            pointsToRemove.push_back(
                        mpSelectedCloud->points[index]);
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
    mUi->removeProgressBar->setValue(0);
    PointCloud<PointXYZ>::iterator it1;
    for(it1 = pointsToRemove.begin();
        it1 < pointsToRemove.end();
        it1++)
    {
        mUi->removeProgressBar->setValue(int(float(it1 -
                                                   pointsToRemove.begin()) /
                                             (float)(pointsToRemove.size())
                                             * 100.));
        PointCloud<PointXYZ>::iterator it2;
        for(it2 = mpSelectedCloud->points.begin();
            it2 != mpSelectedCloud->points.end();)
        {
            // Same point
            if(fabs((*it2).x -(*it1).x) < 0.00001 &&
                    fabs((*it2).y - (*it1).y) < 0.00001 &&
                    fabs((*it2).z - (*it1).z) < 0.00001)
            {
                it2 = mpSelectedCloud->points.erase(it2);
                break;
            }
            else
            {
                it2++;
            }
        }
    }
    mUi->removeProgressBar->setValue(100);
    mNumPointsSelected = (int)mpSelectedCloud->points.size();
    mUi->pointsSelectedLabel->setText(
                QString(QString::number(mNumPointsSelected)
                        + " points"));
    mpSelectedViewer->updatePointCloud(mpSelectedCloud, "");
    mpSelectedViewer->resetCamera();
    mUi->qvtkWidget_2->update();
    if(mpSelectedCloud->points.size() > 0)
    {
        mUi->saveButton->setEnabled(true);
        mUi->saveButton->setText(QString("Save"));
    }
    else if(mpSelectedCloud->points.size() == 0)
    {
        mUi->saveButton->setEnabled(false);
    }
}


/**
 * @brief PCLObjectExtractor::on_actionHelp_triggered
 */
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


/**
 * @brief PCLObjectExtractor::on_actionExit_triggered
 */
void PCLObjectExtractor::on_actionExit_triggered()
{
    close();
}


/**
 * @brief PCLObjectExtractor::on_loadButton_clicked
 */
void PCLObjectExtractor::on_loadButton_clicked()
{
    QString fileName = mFileDialog.getOpenFileName(this,
                                                   tr("Load Point Cloud"),
                                                   QDir::homePath(),
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
        if(mPCDReader.read(fileName.toUtf8().constData(),
                           cloud) < 0)
        {
            QMessageBox::information(this,
                                     "Error",
                                     fileName + " failed to open.");
            return;
        }
        fromPCLPointCloud2(cloud, *mpLoadedCloud);
        mpSelectedCloud->points.clear();
        mpSelectedViewer->updatePointCloud(mpSelectedCloud, "");
        mpLoadedViewer->updatePointCloud(mpLoadedCloud, "");
        mpLoadedViewer->resetCamera();
        mpSelectedViewer->resetCamera();
        mUi->qvtkWidget->update();
        mUi->qvtkWidget_2->update();
        mUi->pointCloudSourceLabel->setText(fileName.split("/").last());
    }
}


/**
 * @brief PCLObjectExtractor::on_saveButton_clicked
 */
void PCLObjectExtractor::on_saveButton_clicked()
{
    QString selectedFilter;
    QString fileName = mFileDialog.getSaveFileName(this,
                                                   tr("Save Selected Object"),
                                                   QDir::homePath(),
                                                   QString("ASCII PCD(*.pcd)") +
                                                   ";;Binary PCD(*.pcd)" +
                                                   ";;Compressed Binary PCD" +
                                                   "(*.pcd)",
                                                   &selectedFilter);
    if(!fileName.isEmpty())
    {
        if(!(fileName.endsWith(".pcd")))
        {
            fileName += ".pcd";
        }
        if(mpSelectedCloud->points.size() > 0)
        {
            mpSelectedCloud->height = 1;
            mpSelectedCloud->width = (int)
                    mpSelectedCloud->points.size();
            mpSelectedCloud->is_dense = false;
            if(selectedFilter.operator ==("ASCII PCD(*.pcd)"))
            {
                if(io::savePCDFileASCII(fileName.toUtf8().constData(),
                                        *mpSelectedCloud) == -1)
                {
                    QMessageBox::information(this,
                                             "Error",
                                             fileName + " failed to save.");
                    return;
                }
            }
            else if(selectedFilter.operator ==("Binary PCD(*.pcd)"))
            {
                if(io::savePCDFileBinary(fileName.toUtf8().constData(),
                                         *mpSelectedCloud) == -1)
                {
                    QMessageBox::information(this,
                                             "Error",
                                             fileName + " failed to save.");
                    return;
                }
            }
            else if(selectedFilter.operator ==("Compressed Binary PCD(*.pcd)"))
            {
                if(io::savePCDFileBinaryCompressed(fileName.toUtf8().
                                                   constData(),
                                                   *mpSelectedCloud) == -1)
                {
                    QMessageBox::information(this,
                                             "Error",
                                             fileName + " failed to save.");
                    return;
                }
            }
            mUi->saveButton->setText(QString("Saved"));
        }
    }
}


/**
 * @brief PCLObjectExtractor::PointSelectionCallback
 * @param event
 * @param args
 */
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


/**
 * @brief PCLObjectExtractor::AreaSelectionCallback
 * @param event
 * @param args
 */
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


/**
 * @brief PCLObjectExtractor::PointRemoveCallback
 * @param event
 * @param args
 */
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


/**
 * @brief PCLObjectExtractor::AreaRemoveCallback
 * @param event
 * @param args
 */
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
