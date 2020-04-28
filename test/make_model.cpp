#include <gtest/gtest.h>

#include <sheet.hpp>
#include <spring_node.hpp>

TEST(test_model_suite, test_model)
{
    int WIDTH = 3;
    int HEIGHT = 3;
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

    Eigen::MatrixXf model_in = sheet.get_model();

    Eigen::MatrixXf model_out{};
    model_out << 0, 5, 0, 5, 0, 0, 0, 0, 0,
                 5, 0, 5, 0, 5, 0, 0, 0, 0,
                 0, 5, 0, 0, 0, 5, 0, 0, 0,
                 5, 0, 0, 0, 5, 0, 5, 0, 0,
                 0, 5, 0, 5, 0, 5, 0, 5, 0,
                 0, 0, 5, 0, 5, 0, 0, 0, 5,
                 0, 0, 0, 5, 0, 0, 0, 5, 0,
                 0, 0, 0, 0, 5, 0, 5, 0, 5,
                 0, 0, 0, 0, 0, 5, 0, 5, 0;

    EXPECT_EQ(model_in, model_out);
}