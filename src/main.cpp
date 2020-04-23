#include <vector>
#include <thread>
#include <chrono>
#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/features/normal_3d.h>

#include <pcl/surface/gp3.h>

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

pcl::PolygonMesh fit_surface(const PointCloud::Ptr in_cloud)
{
    pcl::NormalEstimation<PointT, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (in_cloud);
    n.setInputCloud (in_cloud);
    n.setSearchMethod (tree);
    n.setKSearch (20);
    n.compute (*normals);
    //* normals should not contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::concatenateFields (*in_cloud, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals

    // Create search tree*
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    tree2->setInputCloud (cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
    pcl::PolygonMesh triangles;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (5);

    // Set typical values for the parameters
    gp3.setMu (2.5);
    gp3.setMaximumNearestNeighbors (9);
    gp3.setMaximumSurfaceAngle(M_PI/2); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);

    return triangles;
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

    pcl::PolygonMesh mesh = fit_surface(sheet);

    pcl::visualization::PCLVisualizer::Ptr viewer {new pcl::visualization::PCLVisualizer{"Cloud Viewer"}};
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<PointT>(sheet, "sheet");
    viewer->addPolygonMesh(mesh);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while(!viewer->wasStopped())
    {
        // add_new_data(sheet);
        // viewer->updatePointCloud(sheet, "sheet");
        viewer->spinOnce(1000);
    }

    return 0;
}