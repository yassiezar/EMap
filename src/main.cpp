#include <vector>
#include <thread>
#include <chrono>
#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/visualization/pcl_visualizer.h>

const int WIDTH = 100;
const int HEIGHT = 100;

using PointT = pcl::PointXYZRGB;
using PointCloud = pcl::PointCloud<PointT>;

void add_new_data(PointCloud::Ptr in_cloud)
{
    std::cout << "Adding data" << std::endl;
    PointT zero{0, 255, 0};
    PointCloud::Ptr new_cloud {new PointCloud(10, 10, zero)};
    PointCloud::Ptr out_cloud{new PointCloud(in_cloud->width+new_cloud->width, in_cloud->width+new_cloud->width)};

    for(int i = 0; i < 10; ++i)
    {
        for(int j = 0; j < 10; ++j)
        {
            new_cloud->points[j*10+ i].x = rand() % 100 + 1;
            new_cloud->points[j*10 + i].y = rand() % 100 + 1;
            new_cloud->points[j*10 + i].z = 10;
        }
    }

    *out_cloud = *in_cloud + *new_cloud;

    *in_cloud = std::move(*out_cloud);
}

int main(int argc, char** argv)
{
    using namespace std::chrono_literals;

    PointT zero{255, 0, 0};
    PointCloud::Ptr sheet {new PointCloud{WIDTH, HEIGHT, zero}};
    for(int i = 0; i < WIDTH; ++i)
    {
        for(int j = 0; j < HEIGHT; ++j)
        {
            sheet->points[j*WIDTH + i].x = i;
            sheet->points[j*WIDTH + i].y = j;
            sheet->points[j*WIDTH + i].z = 0;
        }
    }

    pcl::visualization::PCLVisualizer::Ptr viewer {new pcl::visualization::PCLVisualizer{"Cloud Viewer"}};
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<PointT>(sheet, "sheet");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while(!viewer->wasStopped())
    {
        add_new_data(sheet);
        viewer->updatePointCloud(sheet, "sheet");
        viewer->spinOnce(1000);
    }

    return 0;
}