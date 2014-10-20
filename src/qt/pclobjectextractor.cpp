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
    mPreviousWidgetFocus = 0;
    mUi->setupUi(this);
    mpLoadedCloud.reset(new PointCloud<PointXYZ>);
    mpSelectedCloud.reset(new PointCloud<PointXYZ>);
    mpModelCloud.reset(new PointCloud<PointXYZ>);
    mpSceneCloud.reset(new PointCloud<PointXYZ>);
    mpCloudViewer.reset(new visualization::PCLVisualizer("Viewer", false));
    mpSelectionViewer.reset(new visualization::PCLVisualizer("Viewer", false));
    mpModelViewer.reset(new visualization::PCLVisualizer("Viewer", false));
    mpSceneViewer.reset(new visualization::PCLVisualizer("Viewer", false));
    mFileDialog.setDefaultSuffix(QString("pcd"));

    // Set up QVTK widgets and viewers.
    mUi->qvtkWidget_1->SetRenderWindow(mpCloudViewer->getRenderWindow());
    mUi->qvtkWidget_2->SetRenderWindow(mpSelectionViewer->getRenderWindow());
    mUi->qvtkWidget_3->SetRenderWindow(mpModelViewer->getRenderWindow());
    mUi->qvtkWidget_4->SetRenderWindow(mpSceneViewer->getRenderWindow());
    mpCloudViewer->setupInteractor(mUi->qvtkWidget_1->GetInteractor(),
                                   mUi->qvtkWidget_1->GetRenderWindow());
    mpSelectionViewer->setupInteractor(mUi->qvtkWidget_2->GetInteractor(),
                                      mUi->qvtkWidget_2->GetRenderWindow());
    mpModelViewer->setupInteractor(mUi->qvtkWidget_3->GetInteractor(),
                                   mUi->qvtkWidget_3->GetRenderWindow());
    mpSceneViewer->setupInteractor(mUi->qvtkWidget_4->GetInteractor(),
                                   mUi->qvtkWidget_4->GetRenderWindow());
    mpCloudViewer->addPointCloud(mpLoadedCloud, "");
    mpSelectionViewer->addPointCloud(mpSelectedCloud, "");
    mpModelViewer->addPointCloud(mpModelCloud, "");
    mpSceneViewer->addPointCloud(mpSceneCloud, "");
    mpCloudViewer->setShowFPS(false);
    mpSelectionViewer->setShowFPS(false);
    mpModelViewer->setShowFPS(false);
    mpSceneViewer->setShowFPS(false);
    mpCloudViewer->registerPointPickingCallback(&PointSelectionCallback, this);
    mpCloudViewer->registerAreaPickingCallback(&AreaSelectionCallback, this);
    mpSelectionViewer->registerPointPickingCallback(&PointRemoveCallback, this);
    mpSelectionViewer->registerAreaPickingCallback(&AreaRemoveCallback, this);

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
    connect(mUi->qvtkWidget_1,
            SIGNAL(mouseEvent(QMouseEvent*)),
            this,
            SLOT(widget1Focus()));
    connect(mUi->qvtkWidget_2,
            SIGNAL(mouseEvent(QMouseEvent*)),
            this,
            SLOT(widget2Focus()));
    connect(mUi->qvtkWidget_3,
            SIGNAL(mouseEvent(QMouseEvent*)),
            this,
            SLOT(widget3Focus()));
    connect(mUi->qvtkWidget_4,
            SIGNAL(mouseEvent(QMouseEvent*)),
            this,
            SLOT(widget4Focus()));
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
    mpSelectedCloud->points.at(pointIndex) =
            mpLoadedCloud->points.at(pointIndex);
    UpdateSelectedPoints();
    mpSelectionViewer->resetCamera();
    mUi->qvtkWidget_2->update();
}


/**
 * @brief PCLObjectExtractor::AreaHighlightSlot
 * @param pointIndices
 */
void PCLObjectExtractor::AreaHighlightSlot(std::vector<int> pointIndices)
{
    bool errorOccured = false;
    for(int i = 0; i < pointIndices.size(); i++)
    {
        int index = pointIndices.at(i);
        if(index <= mpLoadedCloud->points.size() &&
                index >= 0)
        {
            mpSelectedCloud->points[index] = mpLoadedCloud->points[index];
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
    UpdateSelectedPoints();
    mpSelectionViewer->resetCamera();
    mUi->qvtkWidget_2->update();
}


/**
 * @brief PCLObjectExtractor::PointRemoveSlot
 * @param pointIndex
 */
void PCLObjectExtractor::PointRemoveSlot(int pointIndex)
{
    mpSelectedCloud->points[pointIndex].x = NAN;
    mpSelectedCloud->points[pointIndex].y = NAN;
    mpSelectedCloud->points[pointIndex].z = NAN;
    UpdateSelectedPoints();
    mUi->qvtkWidget_2->update();
}


/**
 * @brief PCLObjectExtractor::AreaRemoveSlot
 * @param pointIndices
 */
void PCLObjectExtractor::AreaRemoveSlot(std::vector<int> pointIndices)
{
    bool errorOccured = false;
    for(int i = 0; i < pointIndices.size(); i++)
    {
        int index = pointIndices.at(i);
        if(index <= mpSelectedCloud->points.size() &&
                index >= 0)
        {
            mpSelectedCloud->points[index].x = NAN;
            mpSelectedCloud->points[index].y = NAN;
            mpSelectedCloud->points[index].z = NAN;
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
        UpdateSelectedPoints();
        mUi->qvtkWidget_2->update();
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
        if(mPCDReader.read(fileName.toUtf8().constData(),
                           cloud) < 0)
        {
            QMessageBox::information(this,
                                     "Error",
                                     fileName + " failed to open.");
            return;
        }
        fromPCLPointCloud2(cloud, *mpLoadedCloud);
        mpSelectedCloud->points.resize(mpLoadedCloud->points.size());
        PointCloud<PointXYZ>::iterator it;
        for(it = mpSelectedCloud->points.begin();
            it < mpSelectedCloud->points.end();
            it++)
        {
            (*it).x = NAN;
            (*it).y = NAN;
            (*it).z = NAN;
        }
        mpSelectionViewer->updatePointCloud(mpSelectedCloud, "");
        mpCloudViewer->updatePointCloud(mpLoadedCloud, "");
        mpCloudViewer->resetCamera();
        mpSelectionViewer->resetCamera();
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
                                                   QDir::currentPath(),
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


void PCLObjectExtractor::widget1Focus()
{
    if(mPreviousWidgetFocus != 1 &&
            mPreviousWidgetFocus != 0)
    {
        ResetStyleSheet(1);
    }
    mUi->groupBox_1->setStyleSheet(QString("QGroupBox") + "{\n    border:" +
                                   "1px solid red;\n    border-radius:" +
                                   "9px;\n    margin-top: 0.5em;\n}\n\n" +
                                   "QGroupBox::title {\n    subcontrol-or" +
                                   "igin: margin;\n    left: 10px;\n    " +
                                   "padding: 0 3px 0 3px;\n}");
    mPreviousWidgetFocus = 1;
}


void PCLObjectExtractor::widget2Focus()
{
    if(mPreviousWidgetFocus != 2 &&
            mPreviousWidgetFocus != 0)
    {
        ResetStyleSheet(2);
    }
    mUi->groupBox_2->setStyleSheet(QString("QGroupBox") + "{\n    border:" +
                                   "1px solid red;\n    border-radius:" +
                                   "9px;\n    margin-top: 0.5em;\n}\n\n" +
                                   "QGroupBox::title {\n    subcontrol-or" +
                                   "igin: margin;\n    left: 10px;\n    " +
                                   "padding: 0 3px 0 3px;\n}");
    mPreviousWidgetFocus = 2;
}


void PCLObjectExtractor::widget3Focus()
{
    if(mPreviousWidgetFocus != 3 &&
            mPreviousWidgetFocus != 0)
    {
        ResetStyleSheet(3);
    }
//    mUi->groupBox_1->setStyleSheet(QString("QGroupBox") + "{\n    border:" +
//                                   "1px solid red;\n    border-radius:" +
//                                   "9px;\n    margin-top: 0.5em;\n}\n\n" +
//                                   "QGroupBox::title {\n    subcontrol-or" +
//                                   "igin: margin;\n    left: 10px;\n    " +
//                                   "padding: 0 3px 0 3px;\n}");
    mPreviousWidgetFocus = 3;
}


void PCLObjectExtractor::widget4Focus()
{
    if(mPreviousWidgetFocus != 4 &&
            mPreviousWidgetFocus != 0)
    {
        ResetStyleSheet(4);
    }
//    mUi->groupBox_1->setStyleSheet(QString("QGroupBox") + "{\n    border:" +
//                                   "1px solid red;\n    border-radius:" +
//                                   "9px;\n    margin-top: 0.5em;\n}\n\n" +
//                                   "QGroupBox::title {\n    subcontrol-or" +
//                                   "igin: margin;\n    left: 10px;\n    " +
//                                   "padding: 0 3px 0 3px;\n}");
    mPreviousWidgetFocus = 4;
}


void PCLObjectExtractor::ResetStyleSheet(int currentWidgetFocus)
{
    if(currentWidgetFocus == 1)
    {
        mUi->groupBox_2->setStyleSheet(QString("QGroupBox") + "{\n    border:" +
                                       "1px solid gray;\n    border-radius:" +
                                       "9px;\n    margin-top: 0.5em;\n}\n\n" +
                                       "QGroupBox::title {\n    subcontrol-or" +
                                       "igin: margin;\n    left: 10px;\n    " +
                                       "padding: 0 3px 0 3px;\n}");
    }
    else if(currentWidgetFocus == 2)
    {
        mUi->groupBox_1->setStyleSheet(QString("QGroupBox") + "{\n    border:" +
                                       "1px solid gray;\n    border-radius:" +
                                       "9px;\n    margin-top: 0.5em;\n}\n\n" +
                                       "QGroupBox::title {\n    subcontrol-or" +
                                       "igin: margin;\n    left: 10px;\n    " +
                                       "padding: 0 3px 0 3px;\n}");
    }
    else if(currentWidgetFocus == 3)
    {
        mUi->groupBox_1->setStyleSheet(QString("QGroupBox") + "{\n    border:" +
                                       "1px solid gray;\n    border-radius:" +
                                       "9px;\n    margin-top: 0.5em;\n}\n\n" +
                                       "QGroupBox::title {\n    subcontrol-or" +
                                       "igin: margin;\n    left: 10px;\n    " +
                                       "padding: 0 3px 0 3px;\n}");
        mUi->groupBox_2->setStyleSheet(QString("QGroupBox") + "{\n    border:" +
                                       "1px solid gray;\n    border-radius:" +
                                       "9px;\n    margin-top: 0.5em;\n}\n\n" +
                                       "QGroupBox::title {\n    subcontrol-or" +
                                       "igin: margin;\n    left: 10px;\n    " +
                                       "padding: 0 3px 0 3px;\n}");
    }
    else if(currentWidgetFocus == 4)
    {
        mUi->groupBox_1->setStyleSheet(QString("QGroupBox") + "{\n    border:" +
                                       "1px solid gray;\n    border-radius:" +
                                       "9px;\n    margin-top: 0.5em;\n}\n\n" +
                                       "QGroupBox::title {\n    subcontrol-or" +
                                       "igin: margin;\n    left: 10px;\n    " +
                                       "padding: 0 3px 0 3px;\n}");
        mUi->groupBox_2->setStyleSheet(QString("QGroupBox") + "{\n    border:" +
                                       "1px solid gray;\n    border-radius:" +
                                       "9px;\n    margin-top: 0.5em;\n}\n\n" +
                                       "QGroupBox::title {\n    subcontrol-or" +
                                       "igin: margin;\n    left: 10px;\n    " +
                                       "padding: 0 3px 0 3px;\n}");
    }
}


void PCLObjectExtractor::UpdateSelectedPoints()
{
    mpSelectionViewer->updatePointCloud(mpSelectedCloud, "");
    PointCloud<PointXYZ>::iterator it;
    int mNumPointsSelected = 0;
    for(it = mpSelectedCloud->points.begin();
        it != mpSelectedCloud->points.end();
        it++)
    {
        if((*it).x == (*it).x &&
           (*it).y == (*it).y &&
           (*it).z == (*it).z)
        {
            mNumPointsSelected++;
        }
    }
    mUi->pointsSelectedLabel->setText(
                QString(QString::number(mNumPointsSelected)
                        + " points"));
    if(mpSelectedCloud->points.size() > 0)
    {
        mUi->saveButton->setEnabled(true);
        mUi->saveButton->setText(QString("Save"));
    }
    else if(mpSelectedCloud->points.size() == 0)
    {
        mUi->saveButton->setEnabled(false);
        mUi->saveButton->setText(QString("Save"));
    }
}
