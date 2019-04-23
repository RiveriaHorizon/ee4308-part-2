#ifndef MAZE_UTILITY_H
#define MAZE_UTILITY_H

namespace maze_utility
{
// Maze constants
const double POINT_A_X = -1.5;
const double POINT_A_Y = 1.5;
const double POINT_B_X = 2;
const double POINT_B_Y = 0.0;
const double POINT_C_X = -1.5;
const double POINT_C_Y = -1.5;
const int CARTESIAN_LIMIT = 3;
const int CARTESIAN_MULTIPLIER = 8; //Boundary array limit is related to this by multiplying this constant by 6, followed by adding 1
                                    //Ensure this is an even number (TODO: variable %2)
const int BOUNDARY_ARRAY_LIMIT = 49;
const int OBSTACLE_RADIUS = 4;

void obstacle_area(int obstacle_x, int obstacle_y, int obstacle_radius,
                   int (*maze)[BOUNDARY_ARRAY_LIMIT][BOUNDARY_ARRAY_LIMIT],
                   int (*traceback)[BOUNDARY_ARRAY_LIMIT][BOUNDARY_ARRAY_LIMIT],
                   int (*G_cost)[BOUNDARY_ARRAY_LIMIT][BOUNDARY_ARRAY_LIMIT])
{
    for (int i = -obstacle_radius; i <= obstacle_radius; i++)
    {
        for (int j = -obstacle_radius; j <= obstacle_radius; j++)
        {
            (*maze)[obstacle_y + i][obstacle_x + j] = 99;
            (*traceback)[obstacle_y + i][obstacle_x + j] = 99;
            (*G_cost)[obstacle_y + i][obstacle_x + j] = 99;
        }
    }
}

void reset_maze(int boundary_limit,
                int (*maze)[BOUNDARY_ARRAY_LIMIT][BOUNDARY_ARRAY_LIMIT],
                int (*traceback)[BOUNDARY_ARRAY_LIMIT][BOUNDARY_ARRAY_LIMIT],
                int (*G_cost)[BOUNDARY_ARRAY_LIMIT][BOUNDARY_ARRAY_LIMIT],
                int (*H_cost)[BOUNDARY_ARRAY_LIMIT][BOUNDARY_ARRAY_LIMIT],
                bool (*check_cost)[BOUNDARY_ARRAY_LIMIT][BOUNDARY_ARRAY_LIMIT])
{
    for (int i = 0; i < boundary_limit; i++)
    {
        for (int j = 0; j < boundary_limit; j++)
        {
            if ((*maze)[i][j] != 99)
            {
                (*maze)[i][j] = 0;
                (*traceback)[i][j] = 0;
                (*G_cost)[i][j] = 0;
                (*H_cost)[i][j] = 0;
                (*check_cost)[i][j] = 0;
            }
        }
    }
}

void print_maze(int boundary_limit,
                int (*maze)[BOUNDARY_ARRAY_LIMIT][BOUNDARY_ARRAY_LIMIT],
                int (*traceback)[BOUNDARY_ARRAY_LIMIT][BOUNDARY_ARRAY_LIMIT])
{
    for (int i = 0; i < boundary_limit; i++)
    {
        for (int j = 0; j < boundary_limit; j++)
        {
            std::cout << "	" << (*maze)[i][j];
        }
        std::cout << std::endl;
    }

    std::cout << std::endl;

    for (int i = 0; i < boundary_limit; i++)
    {
        for (int j = 0; j < boundary_limit; j++)
        {
            std::cout << "	" << (*traceback)[i][j];
        }
        std::cout << std::endl;
    }
}

int decode_coordinate(double input, int cartesian_limit, int cartesian_multiplier)
{
    return (cartesian_limit - (double)input) * cartesian_multiplier;
}

double encode_coordinate(int input, int cartesian_limit, int cartesian_multiplier)
{
    return ((double)input / cartesian_multiplier - cartesian_limit) * -1;
}
} // namespace maze_utility
#endif