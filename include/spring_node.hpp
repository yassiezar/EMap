#ifndef _CERES_SURFACE_BUILDING_SPRING_NODE_HPP
#define _CERES_SURFACE_BUILDING_SPRING_NODE_HPP

#include <vector>
#include <cassert>

class SpringNode
{
private:
    std::vector<int> connected_node_idx_;
    int idx_;
    int connection_count_;

public:
    SpringNode();
    void add_connection(int, int);

    std::vector<int> get_connections() const;
    int get_index() const;
};

#endif