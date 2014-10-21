#ifndef PCLOBJECTEXTRACTOR_H
#define PCLOBJECTEXTRACTOR_H

#include <QMainWindow>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/impl/instantiate.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <vtkRenderWindow.h>
#include <vtkEventQtSlotConnect.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <algorithm>
#include <QFileDialog>
#include <QMessageBox>
#include <QIcon>
#include <QVTKWidget.h>
#include <QFocusEvent>

namespace Ui
{
    class PCLObjectExtractor;
}

class PCLObjectExtractor : public QMainWindow
{
    Q_OBJECT

public:
    explicit PCLObjectExtractor(QWidget *parent = 0);
    ~PCLObjectExtractor();

signals:
    void PointHighlightSignal(int pointIndex);
    void AreaHighlightSignal(std::vector<int> pointIndices);
    void PointRemoveSignal(int pointIndex);
    void AreaRemoveSignal(std::vector<int> pointIndices);

private slots:
    void PointHighlightSlot(int pointIndex);
    void AreaHighlightSlot(std::vector<int> pointIndices);
    void PointRemoveSlot(int pointIndex);
    void AreaRemoveSlot(std::vector<int> pointIndices);
    void on_actionHelp_triggered();
    void on_actionExit_triggered();
    void on_loadCloudButton_clicked();
    void on_saveCloudButton_clicked();
    void on_loadModelButton_clicked();
    void on_loadSceneButton_clicked();
    void on_recognizeButton_clicked();
    void widget1Focus();
    void widget2Focus();

private:
    static void PointSelectionCallback(
                            const pcl::visualization::PointPickingEvent& event,
                            void* args);
    static void AreaSelectionCallback(
                            const pcl::visualization::AreaPickingEvent& event,
                            void* args);
    static void PointRemoveCallback(
                            const pcl::visualization::PointPickingEvent& event,
                            void* args);
    static void AreaRemoveCallback(
                            const pcl::visualization::AreaPickingEvent& event,
                            void* args);
    void ResetStyleSheet(int currentWidgetFocus);
    void UpdateSelectedPoints();
    Ui::PCLObjectExtractor *mUi;
    pcl::visualization::PCLVisualizer::Ptr mpCloudViewer;
    pcl::visualization::PCLVisualizer::Ptr mpSelectionViewer;
    pcl::visualization::PCLVisualizer::Ptr mpModelViewer;
    pcl::visualization::PCLVisualizer::Ptr mpSceneViewer;
    pcl::visualization::PCLVisualizer::Ptr mpOutputViewer;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mpLoaded;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mpSelected;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mpModel;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mpScene;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mpOutput;
    pcl::PointCloud<pcl::SHOT352>::Ptr mpModelDescriptors;
    pcl::PointCloud<pcl::SHOT352>::Ptr mpSceneDescriptors;
    pcl::PointCloud<pcl::Normal>::Ptr mpModelNormals;
    pcl::PointCloud<pcl::Normal>::Ptr mpSceneNormals;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mpModelKeypoints;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mpSceneKeypoints;
    QFileDialog mFileDialog;
    pcl::PCDReader mPCDReader;
    int mNumPointsSelected;
    int mPreviousWidgetFocus;
};

#endif // PCLOBJECTEXTRACTOR_H
