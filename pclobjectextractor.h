#ifndef PCLOBJECTEXTRACTOR_H
#define PCLOBJECTEXTRACTOR_H

#include <QMainWindow>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>
#include <vtkEventQtSlotConnect.h>
#include <pcl/filters/filter.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

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
    void AreaHighlightSignal(std::vector<int> pointIndecies);

private slots:
    void PointHighlightSlot(int pointIndex);
    void AreaHighlightSlot(std::vector<int> pointIndecies);

private:
    static void PointSelectionCallback(
                            const pcl::visualization::PointPickingEvent& event,
                            void* args);
    static void AreaSelectionCallback(
                            const pcl::visualization::AreaPickingEvent& event,
                            void* args);
    Ui::PCLObjectExtractor *ui;

protected:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> mViewer;
    PointCloudT::Ptr mCloud;
};

#endif // PCLOBJECTEXTRACTOR_H
