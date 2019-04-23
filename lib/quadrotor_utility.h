#ifndef QUADROTOR_UTILITY_H
#define QUADROTOR_UTILITY_H

#define _USE_MATH_DEFINES
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>

#include "maze_utility.h"

namespace quadrotor_utility
{
void write_path(std::ofstream &outfile,
                double pose_x, double pose_y, double pose_z,
                double vel_x, double vel_y, double vel_z,
                double acc_x, double acc_y, double acc_z,
                double heading, double ang_vel)
{
    outfile << std::setprecision(2) << pose_x << " " << pose_y << " " << pose_z << " " << //x,y,z position
        vel_x << " " << vel_y << " " << vel_z << " " <<                                   //x,y,z velocity
        acc_x << " " << acc_y << " " << acc_z << " " <<                                   //x,y,z acceleration
        heading << " " << ang_vel <<                                                      //heading
        std::endl;
}

void hover(std::ofstream &outfile)
{
    int pause = 0;
    //pause quadrotor if serial input is true
    std::cout << "Please enter 1 if you want to have a pause when arriving at checkpoint." << std::endl;
    std::cin >> pause;

    if (pause)
    {
        for (int i = 0; i < 150; i++)
        {
            write_path(outfile, maze_utility::POINT_B_X, maze_utility::POINT_B_Y, 1.0,
                       0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0,
                       0.0, 0.0);
        }
    }
}

void take_off(std::ofstream &outfile)
{
    double incr_vel = 0.0;
    for (int i = 0; i < 201; i++)
    {
        write_path(outfile,
                   maze_utility::POINT_A_X, maze_utility::POINT_A_Y, incr_vel,
                   0.0, 0.0, 0.3,
                   0.0, 0.0, 0.0,
                   0.0, 0.0);
        incr_vel = incr_vel + 0.005;
    }

    for (int i = 0; i < 101; i++)
    {
        write_path(outfile,
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
        write_path(outfile, maze_utility::POINT_C_X, maze_utility::POINT_C_Y, desc_vel,
                   0.0, 0.0, -0.4,
                   0.0, 0.0, 0.0,
                   0.0, 0.0);
        desc_vel = desc_vel - 0.01;
    }
    desc_vel = 0.3;
    for (int i = 0; i < 31; i++)
    {
        write_path(outfile, maze_utility::POINT_C_X, maze_utility::POINT_C_Y, desc_vel,
                   0.0, 0.0, -0.4,
                   0.0, 0.0, 0.0,
                   0.0, 0.0);
        desc_vel = desc_vel - 0.01;
    }

    for (int i = 0; i < 11; i++)
    {
        write_path(outfile, maze_utility::POINT_C_X, maze_utility::POINT_C_Y, 0.0,
                   0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0,
                   0.0, 0.0);
    }
}
} // namespace quadrotor_utility
#endif