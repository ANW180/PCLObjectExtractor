#ifndef PCLOBJECTEXTRACTOR_H
#define PCLOBJECTEXTRACTOR_H

#include <QMainWindow>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>

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

private:
    Ui::PCLObjectExtractor *ui;

protected:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> mViewer;
    PointCloudT::Ptr mCloud;
};

#endif // PCLOBJECTEXTRACTOR_H
