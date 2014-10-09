#ifndef PCLOBJECTEXTRACTOR_H
#define PCLOBJECTEXTRACTOR_H

#include <QMainWindow>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/impl/instantiate.hpp>
#include <vtkRenderWindow.h>
#include <vtkEventQtSlotConnect.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <algorithm>
#include <QFileDialog>
#include <QMessageBox>
#include <QIcon>


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
    void on_loadButton_clicked();
    void on_saveButton_clicked();

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
    Ui::PCLObjectExtractor *mUi;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> mpLoadedCloudViewer;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> mpSelectedCloudViewer;
    pcl::PCLPointCloud2Ptr mpLoadedCloud;
    pcl::PCLPointCloud2Ptr mpSelectedCloud;
    std::string mLoadedCloudType;
    std::string mSelectedCloudType;
    QFileDialog mFileDialog;
    pcl::PCDReader mPCDReader;
    pcl::PCDWriter mPCDWriter;
    int mNumPointsSelected;
};

#endif // PCLOBJECTEXTRACTOR_H
