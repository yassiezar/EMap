#include <sheet.hpp>

Sheet::Sheet(int width,int height) : sheet_width_{width}, sheet_height_{height}
{
    sheet_.resize(sheet_height_, std::vector<SpringNode>(sheet_width_, SpringNode()));
}

void Sheet::update_connection(int idx, int connection)
{
    int col = idx % sheet_width_;
    int row = idx / sheet_height_;
    sheet_[row][col].add_connection(idx, connection);
}

Eigen::MatrixXf Sheet::get_model()
{
    Eigen::MatrixXf A{sheet_height_*sheet_width_, sheet_width_*sheet_height_};
    A.setZero();

    int cur_row = 0;
    for(auto row : sheet_)
    {
        for(auto node : row)
        {
            std::vector<int> connections = node.get_connections();
            for(auto c : connections)
            {
                if(c > -1)
                {
                    NodeCoord coord2 = get_node(c);
                    A(cur_row, c % (sheet_width_*sheet_height_)) = 0.5*k_;
                }
            }
            ++cur_row;
            std::cout << A << "\n" << std::endl;
        }
    }

    return A;
}

constexpr NodeCoord Sheet::get_node(int idx) 
{
    return NodeCoord{idx%sheet_width_, idx/sheet_height_};
}