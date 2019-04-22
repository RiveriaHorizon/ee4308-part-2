#ifndef MAZE_UTILITY_H
#define MAZE_UTILITY_H

#include <iostream>
#include <iomanip>
#include <fstream>
#include <math.h>

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
                int (*H_cost)[BOUNDARY_ARRAY_LIMIT][BOUNDARY_ARRAY_LIMIT])
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

float calc_heading(int i_next, int j_next, int i, int j)
{
    //North-West	North	 	North-East
    //West	          +				  East
    //South-West	South	    South-East

    // Next path on the East
    if (i_next == i && j_next > j)
    {
        return 0;
    }

    // Next path on the the North-East
    else if (i_next < i && j_next > j)
    {
        return M_PI / 4;
    }

    // Next path on the North
    else if (i_next < i && j_next == j)
    {
        return M_PI / 2;
    }

    // Next path on the North-West
    else if (i_next < i && j_next < j)
    {
        return (3 * M_PI) / 4;
    }

    // Next path on the West
    else if (i_next == i && j_next < j)
    {
        return M_PI;
    }

    // Next path on the South-West
    else if (i_next > i && j_next < j)
    {
        return (5 * M_PI) / 4;
    }

    // Next path on the South
    else if (i_next > i && j_next == j)
    {
        return (3 * M_PI) / 2;
    }

    // Next path on the South-East
    else if (i_next > i && j_next > j)
    {
        return (7 * M_PI) / 4;
    }
}

void write_path(std::ofstream &outfile,
                float pose_x, float pose_y, float pose_z,
                float vel_x, float vel_y, float vel_z,
                float acc_x, float acc_y, float acc_z,
                float heading, float ang_vel)
{
    outfile << std::setprecision(2) << pose_x << " " << pose_y << " " << pose_z << " " << //x,y,z position
        vel_x << " " << vel_y << " " << vel_z << " " <<                                   //x,y,z velocity
        acc_x << " " << acc_y << " " << acc_z << " " <<                                   //x,y,z acceleration
        heading << " " << ang_vel <<                                                      //heading
        std::endl;
}

void take_off(std::ofstream &outfile)
{
    double incr_vel = 0.0;
    for (int i = 0; i < 501; i++)
    {
        maze_utility::write_path(outfile,
                                 maze_utility::POINT_A_X, maze_utility::POINT_A_Y, incr_vel,
                                 0.0, 0.0, 0.3,
                                 0.0, 0.0, 0.0,
                                 0.0, 0.0);
        incr_vel = incr_vel + 0.002;
    }

    for (int i = 0; i < 101; i++)
    {
        maze_utility::write_path(outfile,
                                 maze_utility::POINT_A_X, maze_utility::POINT_A_Y, 1.00,
                                 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0,
                                 0.0, 0.0);
    }
}

void landing(std::ofstream &outfile)
{
    double desc_vel = 1.0;
    for (int i = 0; i < 71; i++)
    {
        maze_utility::write_path(outfile, maze_utility::POINT_C_X, maze_utility::POINT_C_Y, desc_vel,
                                 0.0, 0.0, -0.4,
                                 0.0, 0.0, 0.0,
                                 0.0, 0.0);
        desc_vel = desc_vel - 0.01;
    }
    desc_vel = 0.3;
    for (int i = 0; i < 31; i++)
    {
        maze_utility::write_path(outfile, maze_utility::POINT_C_X, maze_utility::POINT_C_Y, desc_vel,
                                 0.0, 0.0, -0.4,
                                 0.0, 0.0, 0.0,
                                 0.0, 0.0);
        desc_vel = desc_vel - 0.01;
    }

    for (int i = 0; i < 11; i++)
    {
        maze_utility::write_path(outfile, maze_utility::POINT_C_X, maze_utility::POINT_C_Y, 0.0,
                                 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0,
                                 0.0, 0.0);
    }
}

} // namespace maze_utility
#endif