#ifndef TRAJECTORY_GEN_H
#define TRAJECTORY_GEN_H

#include <algorithm>
#include <iostream>
#include <cmath>
#include <vector>

namespace trajectory_utility
{
// Creates the minimum jerk trajectory that accepts the start position, desired position,
// the frequency of the control system and a time parameter that indicates how long it
// should take to get from the start position to the desired position
std::vector<double> position_trajectory_gen(double start_pose, double desired_pose, double frequency, double move_time)
{
    std::vector<double> trajectory;
    double time_frequency = move_time * frequency;
    for (int time = 1; time <= time_frequency; time++)
    {
        trajectory.push_back(start_pose + (desired_pose - start_pose) *
                                              (10.0 * pow((time / time_frequency), 3.0) -
                                               15.0 * pow((time / time_frequency), 4.0) +
                                               6.0 * pow((time / time_frequency), 5.0)));
    }
    return trajectory;
}

std::vector<double> velocity_trajectory_gen(double start_pose, double desired_pose, double frequency, double move_time)
{
    std::vector<double> trajectory_derivative;
    double time_frequency = move_time * frequency;

    for (int time = 1; time <= time_frequency; time++)
    {
        trajectory_derivative.push_back(frequency * (desired_pose - start_pose) *
                                        (30.0 * pow(time, 2.0) * pow(1.0 / time_frequency, 3.0) -
                                         60.0 * pow(time, 3.0) * pow(1.0 / time_frequency, 4.0) +
                                         30.0 * pow(time, 4.0) * pow(1.0 / time_frequency, 5.0)));
    }
    return trajectory_derivative;
}

std::vector<double> acceleration_trajectory_gen(double start_pose, double desired_pose, double frequency, double move_time)
{
    std::vector<double> trajectory_derivative_two;
    double time_frequency = move_time * frequency;

    for (int time = 1; time <= time_frequency; time++)
    {
        trajectory_derivative_two.push_back(frequency * (desired_pose - start_pose) *
                                            (60.0 * pow(time, 1.0) * pow(1.0 / time_frequency, 3.0) -
                                             180.0 * pow(time, 2.0) * pow(1.0 / time_frequency, 4.0) +
                                             120.0 * pow(time, 3.0) * pow(1.0 / time_frequency, 5.0)));
    }
    return trajectory_derivative_two;
}

std::vector<std::vector<double>> calc_matrix_inverse(int n, std::vector<std::vector<double>> matrix)
{
    int i, j, k;
    std::vector<std::vector<double>> a;
    float t;
    for (i = 0; i < n; i++)
        for (j = 0; j < n; j++)
            a[i][j] = matrix[i][j];
    for (i = 0; i < n; i++)
    {
        for (j = n; j < 2 * n; j++)
        {
            if (i == j - n)
                a[i][j] = 1;
            else
                a[i][j] = 0;
        }
    }
    for (i = 0; i < n; i++)
    {
        t = a[i][i];
        for (j = i; j < 2 * n; j++)
            a[i][j] = a[i][j] / t;
        for (j = 0; j < n; j++)
        {
            if (i != j)
            {
                t = a[j][i];
                for (k = 0; k < 2 * n; k++)
                    a[j][k] = a[j][k] - t * a[i][k];
            }
        }
    }
    std::cout << "\n\nInverse matrix\n\n";
    for (i = 0; i < n; i++)
    {
        for (j = n; j < 2 * n; j++)
            std::cout << "\t" << a[i][j];
        std::cout << "\n";
    }
    return a;
}

int calc_path_steps(int row, int col,
                    int (*traceback)[maze_utility::BOUNDARY_ARRAY_LIMIT][maze_utility::BOUNDARY_ARRAY_LIMIT])
{
    int count = 0;
    //Count then number of steps needed to reach the destination
    for (row = 0; row < maze_utility::BOUNDARY_ARRAY_LIMIT; row++)
    {
        for (col = 0; col < maze_utility::BOUNDARY_ARRAY_LIMIT; col++)
        {
            if ((*traceback)[row][col] != 99 && (*traceback)[row][col] != 11111)
            {
                count++;
            }
        }
    }
    return count;
}

void trajectory_smoothing(std::ofstream &outfile,
                          int (*traceback)[maze_utility::BOUNDARY_ARRAY_LIMIT][maze_utility::BOUNDARY_ARRAY_LIMIT])
{
    int loop = 0, count = 0,
        row = 0, col = 0,
        check = 0;
    int i = 0, j = 0, i_next = 0, j_next = 0;
    float x = 0.0, y = 0.0;
    double direction = 0.0;
    double total_time = 0.0;
    double frequency = 20;
    std::vector<double> row_position, col_position,
        row_velocity, col_velocity,
        row_acceleration, col_acceleration;
    std::vector<double> row_pose_list, col_pose_list,
        row_vel_list, col_vel_list,
        row_acc_list, col_acc_list;
    std::vector<Vector> waypointlist;
    int iteration = 0;
    double prev_direction = std::numeric_limits<int>::max();
    double prev_row = std::numeric_limits<int>::max();
    double prev_col = std::numeric_limits<int>::max();

    //Count then number of steps needed to reach the destination
    calc_path_steps(row, col, traceback);

    while (loop <= count)
    {
        // Search through the traceback array to get the path starting from 0.
        // Record the coordinates with reference to the array and cartesian at current iteration and next iteration
        for (row = 0; row < maze_utility::BOUNDARY_ARRAY_LIMIT; row++)
        {
            for (col = 0; col < maze_utility::BOUNDARY_ARRAY_LIMIT; col++)
            {
                if (loop != count)
                {
                    // If the number is within the planned path, we record the coordinate
                    if ((*traceback)[row][col] == check)
                    {
                        i = row;
                        j = col;
                        x = maze_utility::encode_coordinate(i, maze_utility::CARTESIAN_LIMIT, maze_utility::CARTESIAN_MULTIPLIER);
                        y = maze_utility::encode_coordinate(j, maze_utility::CARTESIAN_LIMIT, maze_utility::CARTESIAN_MULTIPLIER);
                        Vector coordinate = Vector(x, y, 0.0);
                        waypointlist.push_back(coordinate);
                    }
                    if ((*traceback)[row][col] == check + 1)
                    {
                        i_next = row;
                        j_next = col;
                    }
                }
            }
        }

        direction = maze_utility::calc_heading(i_next, j_next, i, j);
        x = waypointlist[loop].x;
        y = waypointlist[loop].y;

        if (prev_direction != direction && prev_direction != std::numeric_limits<int>::max())
        {
            std::cout << "Direction: " << direction << "\t"
                      << "Previous Direction: " << prev_direction << std::endl;
            std::cout << "Loop iteration: " << loop << std::endl;
            std::cout << "Previous x: " << prev_row << "\t"
                      << "Previous y: " << prev_col << "\t"
                      << "Current x: " << x << "\t"
                      << "Current y: " << y << std::endl;

            prev_direction = direction;
            double average_velocity = 0.5;
            double time_taken = sqrt(pow(x - prev_row, 2.0) + pow(y - prev_col, 2.0)) / average_velocity;

            row_position = trajectory_utility::position_trajectory_gen(prev_row, x, frequency, time_taken);
            col_position = trajectory_utility::position_trajectory_gen(prev_col, y, frequency, time_taken);
            row_velocity = trajectory_utility::velocity_trajectory_gen(prev_row, x, frequency, time_taken);
            col_velocity = trajectory_utility::velocity_trajectory_gen(prev_col, y, frequency, time_taken);
            row_acceleration = trajectory_utility::acceleration_trajectory_gen(prev_row, x, frequency, time_taken);
            col_acceleration = trajectory_utility::acceleration_trajectory_gen(prev_col, y, frequency, time_taken);

            for (int iteration = 0; iteration < row_position.size(); iteration++)
            {
                row_pose_list.push_back(row_position[iteration]);
            }
            for (int iteration = 0; iteration < col_position.size(); iteration++)
            {
                col_pose_list.push_back(col_position[iteration]);
            }
            for (int iteration = 0; iteration < row_velocity.size(); iteration++)
            {
                row_vel_list.push_back(row_velocity[iteration]);
            }
            for (int iteration = 0; iteration < col_velocity.size(); iteration++)
            {
                col_vel_list.push_back(col_velocity[iteration]);
            }
            for (int iteration = 0; iteration < row_acceleration.size(); iteration++)
            {
                row_acc_list.push_back(row_acceleration[iteration]);
            }
            for (int iteration = 0; iteration < col_acceleration.size(); iteration++)
            {
                col_acc_list.push_back(col_acceleration[iteration]);
            }

            prev_row = x;
            prev_col = y;
            total_time = time_taken + total_time;
        }
        else if (prev_direction == std::numeric_limits<int>::max())
        {
            prev_direction = direction;
            prev_row = x;
            prev_col = y;
        }

        check++;
        loop++;
    }

    // Curve *curve = new BSpline();
    // curve->set_steps(3);

    // while (waypointlist.size() > 0)
    // {
    // 	curve->add_way_point(waypointlist.front());
    // 	waypointlist.pop_front();
    // }

    for (int i = 1; i < row_pose_list.size(); ++i)
    {
        maze_utility::write_path(outfile, row_pose_list[i], col_pose_list[i], 1.0,
                                 row_vel_list[i], col_vel_list[i], 0.0,
                                 row_acc_list[i], col_acc_list[i], 0.0,
                                 0.0, 0.0);
    }

    // vector<double> xaxis;
    // for (int i = 1; i < (int)round(total_time * frequency); i++)
    // {
    // 	xaxis.push_back(i / frequency);
    // }
    // matplotlibcpp::named_plot("X-Position", xaxis, row_pose_list);
    // matplotlibcpp::named_plot("Y-Position", xaxis, col_pose_list);
    // matplotlibcpp::named_plot("X-Velocity", xaxis, row_vel_list);
    // matplotlibcpp::named_plot("Y-Velocity", xaxis, col_vel_list);
    // matplotlibcpp::named_plot("X-Acceleration", xaxis, row_acc_list);
    // matplotlibcpp::named_plot("Y-Acceleration", xaxis, col_acc_list);
    // matplotlibcpp::title("Minimum Jerk Trajectory");
    // matplotlibcpp::xlabel("Time [s]");
    // matplotlibcpp::ylabel("Distance [m], Velocity [m/s] and Acceleration [m/s2]");

    // for (int i = 0; i < row_pose_list.size(); i++)
    // {
    //     row_pose_list[i] = -row_pose_list[i];
    // }
    // matplotlibcpp::named_plot("Path", col_pose_list, row_pose_list);
    // matplotlibcpp::xlabel("X-Coordinate");
    // matplotlibcpp::ylabel("Y-Coordinate");
    // matplotlibcpp::legend();
    // matplotlibcpp::show();

    int pause = 0;
    //pause quadrotor if serial input is true
    std::cout << "Please enter 1 if you want to have a pause when arriving at checkpoint." << std::endl;
    std::cin >> pause;

    if (pause)
    {
        for (int i = 0; i < 150; i++)
        {
            maze_utility::write_path(outfile, maze_utility::POINT_B_X, maze_utility::POINT_B_Y, 1.0,
                                     0.0, 0.0, 0.0,
                                     0.0, 0.0, 0.0,
                                     0.0, 0.0);
        }
    }
}

void calc_coeff(std::vector<std::vector<double>> inv_matrix, double t0, double tf, double q0, double qf, double vel0, double velf, double acc0, double accf)
{
    std::vector<double> coeff_vector;

    coeff_vector.push_back((inv_matrix[0][0] * q0) + (inv_matrix[0][1] * vel0 * t0) + (inv_matrix[0][2] * acc0 * t0 * t0) +
                           (inv_matrix[0][3] * qf * t0 * t0 * t0) + (inv_matrix[0][4] * velf * t0 * t0 * t0 * t0) + (inv_matrix[0][5] * accf * t0 * t0 * t0 * t0 * t0)); // a0

    coeff_vector.push_back((inv_matrix[1][0] * q0) + (inv_matrix[1][1] * vel0) + (inv_matrix[1][2] * acc0 * t0) +
                           (inv_matrix[1][3] * qf * t0 * t0) + (inv_matrix[1][4] * velf * t0 * t0 * t0) + (inv_matrix[1][5] * accf * t0 * t0 * t0 * t0)); //a1

    coeff_vector.push_back((inv_matrix[2][0] * q0) + (inv_matrix[2][1] * vel0) + (inv_matrix[2][2] * acc0) +
                           (inv_matrix[2][3] * qf * t0) + (inv_matrix[2][4] * velf * t0 * t0) + (inv_matrix[2][5] * accf * t0 * t0 * t0)); //a2

    coeff_vector.push_back((inv_matrix[3][0] * q0) + (inv_matrix[3][1] * vel0 * tf) + (inv_matrix[3][2] * acc0 * tf * tf) +
                           (inv_matrix[3][3] * qf * tf * tf * tf) + (inv_matrix[3][4] * velf * tf * tf * tf * tf) + (inv_matrix[3][5] * accf * tf * tf * tf * tf * tf)); // a4

    coeff_vector.push_back((inv_matrix[4][0] * q0) + (inv_matrix[4][1] * vel0) + (inv_matrix[4][2] * acc0 * tf) +
                           (inv_matrix[4][3] * qf * tf * tf) + (inv_matrix[4][4] * velf * tf * tf * tf) + (inv_matrix[4][5] * accf * tf * tf * tf * tf)); //a5

    coeff_vector.push_back((inv_matrix[5][0] * q0) + (inv_matrix[5][1] * vel0) + (inv_matrix[5][2] * acc0) +
                           (inv_matrix[5][3] * qf * tf) + (inv_matrix[5][4] * velf * tf * tf) + (inv_matrix[5][5] * accf * tf * tf * tf)); //a6
}
}; // namespace trajectory_utility
#endif