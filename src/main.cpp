#include <vector>
#include <thread>
#include <chrono>
#include <iostream>
#include <cmath>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/search/flann_search.h>
#include <pcl/search/impl/flann_search.hpp>

#include <pcl/features/normal_3d.h>

#include <pcl/surface/gp3.h>

#include <Eigen/Sparse>

#include <sheet.hpp>

const int WIDTH = 3;
const int HEIGHT = 3;

using PointT = pcl::PointXYZRGB;
using PointCloud = pcl::PointCloud<PointT>;

template <typename T> 
int sgn(T val) 
{
    return (T(0) < val) - (val < T(0));
}

std::vector<float> get_point_displacements(const PointCloud::Ptr input, const PointCloud::Ptr base_cloud, std::vector<int> ind, std::vector<float>dists)
{
     int num_iter = 500;
     float epsilon = pow(10, -4);

     float k = 10.f;

     std::vector<float> d;
     d.resize(HEIGHT*WIDTH);

     Eigen::Matrix<float, HEIGHT*WIDTH, 1> b = Eigen::Matrix<float, HEIGHT*WIDTH, 1>::Ones(HEIGHT*WIDTH, 1);
     b *= 0.5*k;
    //  Eigen::SparseMatrix<float> b{HEIGHT*WIDTH, 1};
     Eigen::Matrix<float, HEIGHT*WIDTH, HEIGHT*WIDTH> A = Eigen::Matrix<float, HEIGHT*WIDTH, HEIGHT*WIDTH>::Identity(HEIGHT*WIDTH, HEIGHT*WIDTH);
     A.diagonal(0) *= 0.5f*k;
     for(int i = 0; i < A.cols()*A.rows(); ++i)
     {
         if(i != A.cols()-1)   A(i+1, i) = 0.5*k;
         if(i != 0)                       A(i-1, i) = 0.5*k;
         if(i == A.cols() - 1) break;
     }
    //  A.diagonal(-1) = A.diagonal(0);
    //  A.diagonal(0) += 0.5*k;
    //  Eigen::SparseMatrix<float> A{HEIGHT*WIDTH, HEIGHT*WIDTH};
    //  A.setIdentity();
    // A.diagonal(1) = Eigen::VectorXf(0.5*k);
    // A.diagonal(-1) = Eigen::VectorXf(0.5*k);
     Eigen::Matrix<float, HEIGHT*WIDTH, 1> x = Eigen::Matrix<float, HEIGHT*WIDTH, 1>::Ones(HEIGHT*WIDTH, 1);

    // for(auto i : ind)
    for(int i = 0; i < dists.size(); ++i)
    {
        // b.coeffRef(i, 0) = dists[i];
        // b(i, 0) = dists[i];
        b(ind[i]) = dists[i];
    }
    // A.makeCompressed();
    // b.makeCompressed();

    Eigen::Matrix<float, HEIGHT*WIDTH, 2> r = Eigen::Matrix<float, HEIGHT*WIDTH, 2>::Zero(HEIGHT*WIDTH, 2);
    r.col(0) = b - A*x;
    Eigen::Matrix<float, HEIGHT*WIDTH, 1> p = r.col(0);//b - A*x;
    float alpha = 0;
    float beta = 0;

    // while(k < num_iter)     // Add error convergence criteria
    // for(int i = 0; i < x.cols()-1; ++i)
    for(int i = 0; i < num_iter; ++i)
    {
        float alpha1 = (r.col(0).transpose()*r.col(0));
        float alpha2 = (p.transpose()*A*p);
        alpha = alpha1/alpha2;
        // Eigen::Matrix<float, HEIGHT*HEIGHT, 1> tmp = x.col(i) + (alpha*p.col(i));
        // auto tmp = p*alpha;
        x += p*alpha;
        // x = x + tmp;//alpha*p;
        r.col(1) = r.col(0) - alpha*A*p;
        float norm = r.col(1).squaredNorm();
        if(norm < epsilon)
        {
            std::cout << "Converged in " << i << " iterations" << std::endl;
            break;
        }
        float beta1 = (r.col(1).transpose()*r.col(1));
        float beta2 = (r.col(0).transpose()*r.col(0));
        beta = beta1/beta2;
        p = r.col(1) + beta*p;
        r.col(0) = r.col(1);
    }

    Eigen::VectorXf::Map(&d[0], x.size()) = x;
    std::cout << x << std::endl;

    return d;
}

void add_new_data(PointCloud::Ptr in_cloud)
{
    std::cout << "Adding data" << std::endl;
    PointT zero{0, 255, 0};
    int ndata = 1;
    PointCloud::Ptr new_cloud {new PointCloud(ndata, ndata, zero)};

    for(int i = 0; i < ndata; ++i)
    {
        for(int j = 0; j < ndata; ++j)
        {
            new_cloud->points[j*ndata+ i].x = 2;//rand() % WIDTH + 1;
            new_cloud->points[j*ndata + i].y = 2;//rand() % HEIGHT + 1;
            new_cloud->points[j*ndata + i].z = 5;//sin((i*j)/(HEIGHT*WIDTH));
        }
    }

    // PointCloud::Ptr out_cloud{new PointCloud(in_cloud->width+new_cloud->width, in_cloud->width+new_cloud->width)};
    // *out_cloud = *in_cloud + *new_cloud;

    pcl::search::FlannSearch<PointT> search;
    search.setInputCloud(in_cloud);
    int K = 1;

    std::vector<int> idx(K);
    std::vector<int> ind(ndata*ndata);
    std::vector<float> dist(K);
    std::vector<float> dists(ndata*ndata);

    int i = 0;
    for(auto p : new_cloud->points)
    {
        int num = search.nearestKSearch(p, K, idx, dist);
        ind[i] = idx[0];
        dists[i] = dist[0];
        ++i;
    }

    std::vector<float> point_displacement = get_point_displacements(new_cloud, in_cloud, ind, dists);

    for(int i = 0; i < point_displacement.size(); ++i)
    {
        in_cloud->points[i].z = in_cloud->points[i].z + sqrt(point_displacement[i]);        // TODO: check sign to make sure whether to add or subtract displacement
    }

    // *in_cloud = std::move(*out_cloud);
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
    Sheet sheet{WIDTH, HEIGHT};
    for(int i = 0; i < HEIGHT; ++i)
    {
        for(int j = 0; j < WIDTH; ++j)
        {
            if(i != 0) sheet.update_connection(i*HEIGHT + j, (i - 1)*HEIGHT + j);
            if(j != 0) sheet.update_connection(i*HEIGHT + j, i*HEIGHT + j - 1);
            if(i != HEIGHT - 1) sheet.update_connection(i*HEIGHT + j, (i + 1)*HEIGHT + j);
            if(j != WIDTH - 1) sheet.update_connection(i*HEIGHT + j, i*HEIGHT + j + 1);
        }
    }

    Eigen::MatrixXf model = sheet.get_model();
    // pcl::PolygonMesh mesh = fit_surface(sheet);

    pcl::visualization::PCLVisualizer::Ptr viewer {new pcl::visualization::PCLVisualizer{"Cloud Viewer"}};
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<PointT>(sheet, "sheet");
    // viewer->addPolygonMesh(mesh);
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