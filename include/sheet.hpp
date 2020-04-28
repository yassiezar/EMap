#ifndef _CERES_SURFACE_BUILDING_SHEET_HPP
#define _CERES_SURFACE_BUILDING_SHEET_HPP 

#include <vector>
#include <iostream>

#include <Eigen/Core>

#include <spring_node.hpp>

struct NodeCoord
{
    int x, y;
};

class Sheet
{
private:
    int sheet_width_, sheet_height_;

    std::vector<std::vector<SpringNode>> sheet_;
    constexpr NodeCoord get_node(int);

    const float k_ = 10;

public:
    Sheet(int, int);

    Eigen::MatrixXf get_model();

    void update_connection(int, int);
};

#endif