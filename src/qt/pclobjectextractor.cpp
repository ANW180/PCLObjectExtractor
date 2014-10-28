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
    mpLoaded.reset(new PointCloud<PointXYZ>);
    mpSelected.reset(new PointCloud<PointXYZ>);
    mpModel.reset(new PointCloud<PointXYZ>);
    mpScene.reset(new PointCloud<PointXYZ>);
    mpOutputModel.reset(new PointCloud<PointXYZ>);
    mpOutputScene.reset(new PointCloud<PointXYZ>);
    mpModelDescriptors.reset(new PointCloud<SHOT352>);
    mpSceneDescriptors.reset(new PointCloud<SHOT352>);
    mpModelNormals.reset(new PointCloud<Normal>);
    mpSceneNormals.reset(new PointCloud<Normal>);
    mpModelKeypoints.reset(new PointCloud<PointXYZ>);
    mpSceneKeypoints.reset(new PointCloud<PointXYZ>);
    mpUniformSampling.reset(new UniformSampling<PointXYZ>);
    mpNormalEstimate.reset(new NormalEstimationOMP<PointXYZ, Normal>);
    mpDescriptorEstimate.reset(new SHOTEstimationOMP<
                               PointXYZ,
                               Normal,
                               SHOT352>);
    mpModelSceneCorrespondences.reset(new Correspondences);
    mpModelReference.reset(new PointCloud<ReferenceFrame>);
    mpSceneReference.reset(new PointCloud<ReferenceFrame>);
    mpReferenceEstimator.reset(new BOARDLocalReferenceFrameEstimation<
                               PointXYZ,
                               Normal,
                               ReferenceFrame>);
    mpCloudViewer.reset(new visualization::PCLVisualizer("Viewer", false));
    mpSelectionViewer.reset(new visualization::PCLVisualizer("Viewer", false));
    mpModelViewer.reset(new visualization::PCLVisualizer("Viewer", false));
    mpSceneViewer.reset(new visualization::PCLVisualizer("Viewer", false));
    mpOutputViewer.reset(new visualization::PCLVisualizer("Viewer", false));
    mFileDialog.setDefaultSuffix(QString("pcd"));

    // Set up QVTK widgets and viewers.
    mUi->qvtkWidget_1->SetRenderWindow(mpCloudViewer->getRenderWindow());
    mUi->qvtkWidget_2->SetRenderWindow(mpSelectionViewer->getRenderWindow());
    mUi->qvtkWidget_3->SetRenderWindow(mpSceneViewer->getRenderWindow());
    mUi->qvtkWidget_4->SetRenderWindow(mpModelViewer->getRenderWindow());
    mUi->qvtkWidget_5->SetRenderWindow(mpOutputViewer->getRenderWindow());
    mpCloudViewer->setupInteractor(mUi->qvtkWidget_1->GetInteractor(),
                                   mUi->qvtkWidget_1->GetRenderWindow());
    mpSelectionViewer->setupInteractor(mUi->qvtkWidget_2->GetInteractor(),
                                      mUi->qvtkWidget_2->GetRenderWindow());
    mpSceneViewer->setupInteractor(mUi->qvtkWidget_3->GetInteractor(),
                                   mUi->qvtkWidget_3->GetRenderWindow());
    mpModelViewer->setupInteractor(mUi->qvtkWidget_4->GetInteractor(),
                                   mUi->qvtkWidget_4->GetRenderWindow());
    mpOutputViewer->setupInteractor(mUi->qvtkWidget_5->GetInteractor(),
                                   mUi->qvtkWidget_5->GetRenderWindow());
    mpCloudViewer->addPointCloud(mpLoaded, "");
    mpSelectionViewer->addPointCloud(mpSelected, "");
    mpModelViewer->addPointCloud(mpModel, "");
    mpSceneViewer->addPointCloud(mpScene, "");
    mpCloudViewer->setShowFPS(false);
    mpSelectionViewer->setShowFPS(false);
    mpModelViewer->setShowFPS(false);
    mpSceneViewer->setShowFPS(false);
    mpOutputViewer->setShowFPS(false);
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
}


/**
 * @brief PCLObjectExtractor::~PCLObjectExtractor
 */
PCLObjectExtractor::~PCLObjectExtractor()
{
//    mpCloudViewer.reset();
//    mpSelectionViewer.reset();
//    mpModelViewer.reset();
//    mpSceneViewer.reset();
//    mpLoaded.reset();
//    mpSelected.reset();
//    mpModel.reset();
//    mpScene.reset();
//    mpOutputModel.reset();
//    mpOutputScene.reset();
//    mpModelDescriptors.reset();
//    mpSceneDescriptors.reset();
//    mpModelNormals.reset();
//    mpSceneNormals.reset();
//    mpModelKeypoints.reset();
//    mpSceneKeypoints.reset();
//    mpUniformSampling.reset();
    delete mUi;
}


/**
 * @brief PCLObjectExtractor::PointHighlightSlot
 * @param pointIndex
 */
void PCLObjectExtractor::PointHighlightSlot(int pointIndex)
{
    mpSelected->points.at(pointIndex) =
            mpLoaded->points.at(pointIndex);
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
        if(index <= mpLoaded->points.size() &&
                index >= 0)
        {
            mpSelected->points[index] = mpLoaded->points[index];
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
    mpSelected->points[pointIndex].x = NAN;
    mpSelected->points[pointIndex].y = NAN;
    mpSelected->points[pointIndex].z = NAN;
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
    mNumPointsSelected = 0;
    std::vector<int>::iterator it;
    for(it = pointIndices.begin();
        it != pointIndices.end();
        it++)
    {
        if((*it) <= mpSelected->points.size() &&
                (*it) >= 0)
        {
            if(mpSelected->points.at(*it).x ==
                    mpSelected->points.at(*it).x &&
               mpSelected->points.at(*it).y ==
                    mpSelected->points.at(*it).y &&
               mpSelected->points.at(*it).z ==
                    mpSelected->points.at(*it).z)
            {
                mNumPointsSelected++;
            }
            mpSelected->points.at(*it).x = NAN;
            mpSelected->points.at(*it).y = NAN;
            mpSelected->points.at(*it).z = NAN;
        }
        else
        {
            errorOccured = true;
        }
    }
    if(mNumPointsSelected > 0)
    {
        mUi->statusbar->showMessage(QString("%1 points selected")
                                    .arg(mNumPointsSelected));
    }
    else
    {
        mUi->statusbar->showMessage(tr("No points were selected"));
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
 * @brief PCLObjectExtractor::on_loadCloudButton_clicked
 */
void PCLObjectExtractor::on_loadCloudButton_clicked()
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
        fromPCLPointCloud2(cloud, *mpLoaded);
        mpSelected->points.resize(mpLoaded->points.size());
        PointCloud<PointXYZ>::iterator it;
        for(it = mpSelected->points.begin();
            it < mpSelected->points.end();
            it++)
        {
            (*it).x = NAN;
            (*it).y = NAN;
            (*it).z = NAN;
        }
        mpSelectionViewer->updatePointCloud(mpSelected, "");
        mpCloudViewer->updatePointCloud(mpLoaded, "");
        mpCloudViewer->resetCamera();
        mpSelectionViewer->resetCamera();
        mUi->pointCloudSourceLabel->setText(fileName.split("/").last());
    }
}


/**
 * @brief PCLObjectExtractor::on_saveCloudButton_clicked
 */
void PCLObjectExtractor::on_saveCloudButton_clicked()
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
        if(mpSelected->points.size() > 0)
        {
            mpSelected->height = 1;
            mpSelected->width = (int)
                    mpSelected->points.size();
            mpSelected->is_dense = false;
            if(selectedFilter.operator ==("ASCII PCD(*.pcd)"))
            {
                if(io::savePCDFileASCII(fileName.toUtf8().constData(),
                                        *mpSelected) == -1)
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
                                         *mpSelected) == -1)
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
                                                   *mpSelected) == -1)
                {
                    QMessageBox::information(this,
                                             "Error",
                                             fileName + " failed to save.");
                    return;
                }
            }
            mUi->saveCloudButton->setText(QString("Saved"));
        }
    }
}


/**
 * @brief PCLObjectExtractor::on_loadModelButton_clicked
 */
void PCLObjectExtractor::on_loadModelButton_clicked()
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
        if(io::loadPCDFile(fileName.toUtf8().constData(),
                           *mpModel) < 0)
        {
            QMessageBox::information(this,
                                     "Error",
                                     fileName + " failed to open.");
            return;
        }
//        fromPCLPointCloud2(cloud, *mpModel);
        mpModelViewer->updatePointCloud(mpModel, "");
        mpModelViewer->resetCamera();
        mUi->modelCloudSourceLabel->setText(fileName.split("/").last());
    }
}


/**
 * @brief PCLObjectExtractor::on_loadSceneButton_clicked
 */
void PCLObjectExtractor::on_loadSceneButton_clicked()
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
        if(io::loadPCDFile(fileName.toUtf8().constData(),
                           *mpScene) < 0)
        {
            QMessageBox::information(this,
                                     "Error",
                                     fileName + " failed to open.");
            return;
        }
        //fromPCLPointCloud2(cloud, *mpScene);
        mpSceneViewer->updatePointCloud(mpScene, "");
        mpSceneViewer->resetCamera();
        mUi->sceneCloudSourceLabel->setText(fileName.split("/").last());
    }
}


/**
 * @brief PCLObjectExtractor::on_recognizeButton_clicked
 */
void PCLObjectExtractor::on_recognizeButton_clicked()
{
    if(mpScene->points.size() == 0 || mpModel->points.size() == 0)
    {
        return;
    }
    else
    {
        // Algorithm params
        float model_ss_ (0.01f);
        float scene_ss_ (0.03f);
        float rf_rad_ (0.015f);
        float descr_rad_ (0.02f);
        float cg_size_ (0.01f);
        float cg_thresh_ (5.0f);

//        // Compute cloud resolution
//        float resolution = static_cast<float>(ComputeCloudResolution(mpModel));
//        if (resolution != 0.0f)
//        {
//          model_ss_   *= resolution;
//          scene_ss_   *= resolution;
//          rf_rad_     *= resolution;
//          descr_rad_  *= resolution;
//          cg_size_    *= resolution;
//        }

        // Compute Normals
        mpNormalEstimate->setKSearch(10);
        mpNormalEstimate->setInputCloud(mpModel);
        mpNormalEstimate->compute(*mpModelNormals);
        mpNormalEstimate->setInputCloud(mpScene);
        mpNormalEstimate->compute(*mpSceneNormals);

        // Downsample to extract keypoints
        PointCloud<int> sampled_indices;
        mpUniformSampling->setInputCloud(mpModel);
        mpUniformSampling->setRadiusSearch(model_ss_);
        mpUniformSampling->compute(sampled_indices);
        copyPointCloud(*mpModel,
                       sampled_indices.points,
                       *mpModelKeypoints);

        mpUniformSampling->setInputCloud(mpScene);
        mpUniformSampling->setRadiusSearch(scene_ss_);
        mpUniformSampling->compute(sampled_indices);
        copyPointCloud(*mpScene,
                       sampled_indices.points,
                       *mpSceneKeypoints);

        // Compute descriptor for keypoints
        mpDescriptorEstimate->setRadiusSearch(descr_rad_);

        mpDescriptorEstimate->setInputCloud(mpModelKeypoints);
        mpDescriptorEstimate->setInputNormals(mpModelNormals);
        mpDescriptorEstimate->setSearchSurface(mpModel);
        mpDescriptorEstimate->compute(*mpModelDescriptors);

        mpDescriptorEstimate->setInputCloud(mpSceneKeypoints);
        mpDescriptorEstimate->setInputNormals(mpSceneNormals);
        mpDescriptorEstimate->setSearchSurface(mpScene);
        mpDescriptorEstimate->compute(*mpSceneDescriptors);

        // Find Model-Scene correspondences with KdTree
        KdTreeFLANN<SHOT352> match_search;
        match_search.setInputCloud(mpModelDescriptors);

        // For each scene keypoint descriptor, find nearest neighbor into the
        // model keypoints descriptor cloud and add it to the
        // correspondences vector.
        for(size_t i = 0; i < mpSceneDescriptors->size(); ++i)
        {
            std::vector<int> neigh_indices (1);
            std::vector<float> neigh_sqr_dists (1);
            // Skip NaNs
            if(!pcl_isfinite(mpSceneDescriptors->at(i).descriptor[0]))
            {
                continue;
            }
            int found_neighs = match_search.nearestKSearch(mpSceneDescriptors->at(i),
                                                           1,
                                                           neigh_indices,
                                                           neigh_sqr_dists);
            // Add match only if the squared descriptor distance is less than
            // 0.25 (SHOT descriptor distances are between 0 and 1 by design)
            if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f)
            {
              Correspondence corr(neigh_indices[0],
                                  static_cast<int>(i),
                                  neigh_sqr_dists[0]);
              mpModelSceneCorrespondences->push_back(corr);
            }
        }

        // Actual clustering
        mpReferenceEstimator->setFindHoles(true);
        mpReferenceEstimator->setRadiusSearch(rf_rad_);

        mpReferenceEstimator->setInputCloud(mpModelKeypoints);
        mpReferenceEstimator->setInputNormals(mpModelNormals);
        mpReferenceEstimator->setSearchSurface(mpModel);
        mpReferenceEstimator->compute(*mpModelReference);

        mpReferenceEstimator->setInputCloud(mpSceneKeypoints);
        mpReferenceEstimator->setInputNormals(mpSceneNormals);
        mpReferenceEstimator->setSearchSurface(mpScene);
        mpReferenceEstimator->compute(*mpSceneReference);

        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator
                <Eigen::Matrix4f> > rototranslations;
        std::vector<pcl::Correspondences> clustered_corrs;
        Hough3DGrouping<PointXYZ, PointXYZ,
                ReferenceFrame,
                ReferenceFrame> clusterer;
        clusterer.setHoughBinSize(cg_size_);
        clusterer.setHoughThreshold(cg_thresh_);
        clusterer.setUseInterpolation(true);
        clusterer.setUseDistanceWeight(false);

        clusterer.setInputCloud(mpModelKeypoints);
        clusterer.setInputRf(mpModelReference);
        clusterer.setSceneCloud(mpSceneKeypoints);
        clusterer.setSceneRf(mpSceneReference);
        clusterer.setModelSceneCorrespondences(mpModelSceneCorrespondences);

        if(clusterer.recognize(rototranslations, clustered_corrs))
        {
            mpOutputViewer->addPointCloud(mpScene, "scene");
            // Visualization
            // We are translating the model so that it doesn't end in the middle
            // of the scene representation
            for (size_t i = 0; i < rototranslations.size(); ++i)
            {
                for (size_t j = 0; j < clustered_corrs[i].size(); ++j)
                {
                    std::stringstream ss_line;
                    ss_line << i << "_" << j;
                    transformPointCloud(*mpModel,
                                        *mpOutputModel,
                                        rototranslations[i]);
                    visualization::PointCloudColorHandlerCustom<PointXYZ>
                            outputModel(mpOutputModel,
                                        255,
                                        0,
                                        0);
                    mpOutputViewer->addPointCloud(mpOutputModel,
                                                  outputModel,
                                                  ss_line.str());
                }
            }
            transformPointCloud(*mpModel,
                                *mpModel,
                                Eigen::Vector3f(-1, 0, 0),
                                Eigen::Quaternionf(1, 0, 0, 0));
            transformPointCloud(*mpModelKeypoints,
                                *mpModelKeypoints,
                                Eigen::Vector3f(-1, 0, 0),
                                Eigen::Quaternionf(1, 0, 0, 0));
            visualization::PointCloudColorHandlerCustom<PointXYZ>
                    colorModel(mpModel,
                               255,
                               0,
                               0);
            mpOutputViewer->addPointCloud<PointXYZ>(mpModel,
                                                    colorModel,
                                                    "colorModel");
            mpOutputViewer->resetCamera();
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
        emit ui->AreaRemoveSignal(indices);
    }
}


/**
 * @brief PCLObjectExtractor::widget1Focus
 */
void PCLObjectExtractor::widget1Focus()
{
    if(mPreviousWidgetFocus != 1 &&
            mPreviousWidgetFocus != 0)
    {
        ResetStyleSheet(1);
    }
    mUi->groupBox_1->setStyleSheet(QString("QGroupBox") + "{\n    border:" +
                                   "2px solid red;\n    border-radius:" +
                                   "9px;\n    margin-top: 0.5em;\n}\n\n" +
                                   "QGroupBox::title {\n    subcontrol-or" +
                                   "igin: margin;\n    left: 10px;\n    " +
                                   "padding: 0 3px 0 3px;\n}");
    mPreviousWidgetFocus = 1;
}


/**
 * @brief PCLObjectExtractor::widget2Focus
 */
void PCLObjectExtractor::widget2Focus()
{
    if(mPreviousWidgetFocus != 2 &&
            mPreviousWidgetFocus != 0)
    {
        ResetStyleSheet(2);
    }
    mUi->groupBox_2->setStyleSheet(QString("QGroupBox") + "{\n    border:" +
                                   "2px solid red;\n    border-radius:" +
                                   "9px;\n    margin-top: 0.5em;\n}\n\n" +
                                   "QGroupBox::title {\n    subcontrol-or" +
                                   "igin: margin;\n    left: 10px;\n    " +
                                   "padding: 0 3px 0 3px;\n}");
    mPreviousWidgetFocus = 2;
}


/**
 * @brief PCLObjectExtractor::ResetStyleSheet
 * @param currentWidgetFocus
 */
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
}


/**
 * @brief PCLObjectExtractor::UpdateSelectedPoints
 */
void PCLObjectExtractor::UpdateSelectedPoints()
{
    mpSelectionViewer->updatePointCloud(mpSelected, "");
    PointCloud<PointXYZ>::iterator it;
    int mNumPointsSelected = 0;
    for(it = mpSelected->points.begin();
        it != mpSelected->points.end();
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
    if(mpSelected->points.size() > 0)
    {
        mUi->saveCloudButton->setEnabled(true);
        mUi->saveCloudButton->setText(QString("Save"));
    }
    else if(mpSelected->points.size() == 0)
    {
        mUi->saveCloudButton->setEnabled(false);
        mUi->saveCloudButton->setText(QString("Save"));
    }
}


/**
 * @brief PCLObjectExtractor::ComputeCloudResolution
 * @param cloud
 * @return
 */
double PCLObjectExtractor::ComputeCloudResolution(
                            const PointCloud<PointXYZ>::ConstPtr & cloud)
{
    double res = 0.0;
    int n_points = 0;
    int nres;
    std::vector<int> indices (2);
    std::vector<float> sqr_distances (2);
    search::KdTree<PointXYZ> tree;
    tree.setInputCloud (cloud);

    for (size_t i = 0; i < cloud->size (); ++i)
    {
      if (! pcl_isfinite ((*cloud)[i].x))
      {
        continue;
      }
      //Considering the second neighbor since the first is the point itself.
      nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
      if (nres == 2)
      {
        res += sqrt (sqr_distances[1]);
        ++n_points;
      }
    }
    if (n_points != 0)
    {
      res /= n_points;
    }
    return res;
}
