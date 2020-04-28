#include <spring_node.hpp>

SpringNode::SpringNode() : connection_count_{0}, idx_{0}, connected_node_idx_{std::vector<int>(4, -1)}
{
}

void SpringNode::add_connection(int idx, int connection)
{
    assert(connection_count_ != 4);
    connected_node_idx_[connection_count_] = connection;
    idx_ = idx;
    ++connection_count_;
}

int SpringNode::get_index() const 
{
    return idx_;
}

std::vector<int> SpringNode::get_connections() const
{
    return connected_node_idx_;
}