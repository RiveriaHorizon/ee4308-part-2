#ifndef TRAJECTORY_UTILITY_H
#define TRAJECTORY_UTILITY_H

#define _USE_MATH_DEFINES
#include <algorithm>
#include <iostream>
#include <cmath>
#include <vector>
#include <deque>

#include "quadrotor_utility.h"

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

double calc_heading(int i_next, int j_next, int i, int j)
{
    //North-West  North     North-East
    //West            +          East
    //South-West  South      South-East

    // Next path on the East
    if (i_next == i && j_next > j)
    {
        return 0;
    }

    // Next path on the the North-East
    else if (i_next < i && j_next > j)
    {

        double height = abs(i - i_next);
        double base = abs(j_next - j);
        double pythagoras = sqrt((height * height) + (base * base));
        double angle = asin(height / pythagoras);

        return angle;
    }

    // Next path on the North
    else if (i_next < i && j_next == j)
    {
        return M_PI / 2;
    }

    // Next path on the North-West
    else if (i_next < i && j_next < j)
    {
        double height = abs(i - i_next);
        double base = abs(j - j_next);
        double pythagoras = sqrt((height * height) + (base * base));
        double angle = M_PI - asin(height / pythagoras);

        return angle;
    }

    // Next path on the West
    else if (i_next == i && j_next < j)
    {
        return M_PI;
    }

    // Next path on the South-West
    else if (i_next > i && j_next < j)
    {
        double height = abs(i_next - i);
        double base = abs(j - j_next);
        double pythagoras = sqrt((height * height) + (base * base));
        double angle = M_PI + asin(height / pythagoras);

        return angle;
    }

    // Next path on the South
    else if (i_next > i && j_next == j)
    {
        return (3 * M_PI) / 2;
    }

    // Next path on the South-East
    else if (i_next > i && j_next > j)
    {
        double height = abs(i_next - i);
        double base = abs(j_next - j);
        double pythagoras = sqrt((height * height) + (base * base));
        double angle = (2 * M_PI) - asin(height / pythagoras);

        return angle;
    }
    else if (i_next == i && j_next == j)
    {
        return 0; // TODO: Should change to previous direction
    }
}

int calc_path_steps(int (*traceback)[maze_utility::BOUNDARY_ARRAY_LIMIT][maze_utility::BOUNDARY_ARRAY_LIMIT])
{
    int count = 0;
    //Count then number of steps needed to reach the destination
    for (int row = 0; row < maze_utility::BOUNDARY_ARRAY_LIMIT; row++)
    {
        for (int col = 0; col < maze_utility::BOUNDARY_ARRAY_LIMIT; col++)
        {
            if ((*traceback)[row][col] != 99 && (*traceback)[row][col] != 11111)
            {
                count++;
            }
        }
    }
    return count;
}

std::vector<std::deque<Vector>> create_coordinate_lists(int (*traceback)[maze_utility::BOUNDARY_ARRAY_LIMIT][maze_utility::BOUNDARY_ARRAY_LIMIT])
{
    std::vector<std::deque<Vector>> compiled_list;
    std::deque<Vector> array_coordinate_list, cartesian_coordinate_list;
    int loop = 0, count = 0, check = 0;

    while (loop <= count)
    {
        count = calc_path_steps(traceback);
        // Search through the traceback array to get the path starting from 0.
        // Record the coordinates with reference to the array and cartesian at current iteration and next iteration
        int i_coordinate = 0, j_coordinate = 0,
            i_next_coordinate = 0, j_next_coordinate = 0,
            x_coordinate = 0, y_coordinate = 0;
        for (int row = 0; row < maze_utility::BOUNDARY_ARRAY_LIMIT; row++)
        {
            for (int col = 0; col < maze_utility::BOUNDARY_ARRAY_LIMIT; col++)
            {
                if (loop != count)
                {
                    // If the number is within the planned path, we record the coordinate
                    if ((*traceback)[row][col] == check)
                    {
                        i_coordinate = row;
                        j_coordinate = col;
                        array_coordinate_list.push_back(Vector(i_coordinate, j_coordinate, 0));
                        x_coordinate = maze_utility::encode_coordinate(i_coordinate, maze_utility::CARTESIAN_LIMIT, maze_utility::CARTESIAN_MULTIPLIER);
                        y_coordinate = maze_utility::encode_coordinate(j_coordinate, maze_utility::CARTESIAN_LIMIT, maze_utility::CARTESIAN_MULTIPLIER);
                        cartesian_coordinate_list.push_back(Vector(x_coordinate, y_coordinate, 0.0));
                    }
                    if ((*traceback)[row][col] == check + 1)
                    {
                        i_next_coordinate = row;
                        j_next_coordinate = col;
                    }
                    if (check == count - 1)
                    {
                        i_next_coordinate = i_coordinate;
                        j_next_coordinate = j_coordinate;
                    }
                }
            }
        }
        check++;
        loop++;
    }

    // Compress colated coordinate lists into a single class and return it
    compiled_list.push_back(array_coordinate_list);
    compiled_list.push_back(cartesian_coordinate_list);
    return compiled_list;
}

std::deque<Vector> create_line_segmented_path(int count, std::deque<Vector> array_coordinate_list)
{
    // Constants to be used in array_coordinate_list
    const int FIRST = 0,
              LAST = array_coordinate_list.size() - 1;
    // Define variables that will be used in the generation of line segments
    std::deque<Vector> line_segmented_path;
    double direction_ls = 0;
    double prev_direction_ls = std::numeric_limits<double>::infinity();
    double calculated_constraint = 0.0;

    line_segmented_path.push_back(Vector((int)array_coordinate_list[FIRST].x, (int)array_coordinate_list[FIRST].y, 0.0));
    line_segmented_path.push_back(Vector((int)array_coordinate_list[FIRST].x, (int)array_coordinate_list[FIRST].y, 0.0));
    line_segmented_path.push_back(Vector((int)array_coordinate_list[FIRST].x, (int)array_coordinate_list[FIRST].y, 0.0));
    for (int outer_iteration = 0; outer_iteration < count; outer_iteration++)
    {
        for (int inner_iteration = outer_iteration + 1; inner_iteration < count; inner_iteration++)
        {
            int i_coordinate = array_coordinate_list[outer_iteration].x;
            int j_coordinate = array_coordinate_list[outer_iteration].y;
            int i_coordinate_next = array_coordinate_list[inner_iteration].x;
            int j_coordinate_next = array_coordinate_list[inner_iteration].y;

            direction_ls = calc_heading(i_coordinate_next, j_coordinate_next, i_coordinate, j_coordinate);
            if (direction_ls > M_PI)
            {
                direction_ls = direction_ls - M_PI;
            }

            if (prev_direction_ls != direction_ls && prev_direction_ls != std::numeric_limits<double>::infinity())
            {
                calculated_constraint = std::fabs((direction_ls - prev_direction_ls) / prev_direction_ls) * 100;

                if (calculated_constraint > 30 && calculated_constraint < 60)
                {
                    line_segmented_path.push_back(Vector(array_coordinate_list[inner_iteration].x,
                                                         array_coordinate_list[inner_iteration].y,
                                                         0.0));
                    prev_direction_ls = calc_heading(array_coordinate_list[inner_iteration].x,
                                                     array_coordinate_list[inner_iteration].y,
                                                     array_coordinate_list[outer_iteration].x,
                                                     array_coordinate_list[outer_iteration].y);
                }
                else
                {
                    prev_direction_ls = direction_ls;
                    outer_iteration = inner_iteration;
                    break;
                }
            }
            else
            {
                prev_direction_ls = direction_ls;
            }
        }
        if (outer_iteration == count - 1)
        {
            break;
        }
    }
    line_segmented_path.push_back(Vector((int)array_coordinate_list[LAST].x, (int)array_coordinate_list[LAST].y, 0.0));
    line_segmented_path.push_back(Vector((int)array_coordinate_list[LAST].x, (int)array_coordinate_list[LAST].y, 0.0));
    line_segmented_path.push_back(Vector((int)array_coordinate_list[LAST].x, (int)array_coordinate_list[LAST].y, 0.0));

    return line_segmented_path;
}

double calc_distance(double x1, double y1, double x2, double y2)
{
    return sqrt(pow(x1 - x2, 2.0) + pow(y1 - y2, 2.0));
}

void trajectory_smoothing(std::ofstream &outfile,
                          int (*traceback)[maze_utility::BOUNDARY_ARRAY_LIMIT][maze_utility::BOUNDARY_ARRAY_LIMIT],
                          bool C)
{
    int iteration = 0;
    int count = calc_path_steps(traceback);
    std::vector<double> row_pose_list, col_pose_list,
        row_vel_list, col_vel_list,
        row_acc_list, col_acc_list;

    std::deque<Vector> array_coordinate_list = create_coordinate_lists(traceback)[0];
    std::deque<Vector> cartesian_coordinate_list = create_coordinate_lists(traceback)[1];
    std::deque<Vector> line_segmented_path = create_line_segmented_path(count, array_coordinate_list);

    for (int i = 0; i < line_segmented_path.size(); i++)
    {
        line_segmented_path[i].x = maze_utility::encode_coordinate(line_segmented_path[i].x, maze_utility::CARTESIAN_LIMIT, maze_utility::CARTESIAN_MULTIPLIER);
        line_segmented_path[i].y = maze_utility::encode_coordinate(line_segmented_path[i].y, maze_utility::CARTESIAN_LIMIT, maze_utility::CARTESIAN_MULTIPLIER);
    }

    // Create position, velocity and acceleration profiles at each direction change defined in the line segments
    double total_time = 0.0;
    double frequency = 3;
    for (int i = 0; i < line_segmented_path.size(); i++)
    {
        double x_next_coordinate = line_segmented_path[i + 1].x;
        double y_next_coordinate = line_segmented_path[i + 1].y;
        double x_coordinate = line_segmented_path[i].x;
        double y_coordinate = line_segmented_path[i].y;
        double average_velocity = 0.5;
        double time_taken = calc_distance(x_coordinate, x_next_coordinate, y_coordinate, y_next_coordinate) / average_velocity;

        // std::vector<double> row_position = position_trajectory_gen(x_coordinate, x_next_coordinate, frequency, time_taken);
        // std::vector<double> col_position = position_trajectory_gen(y_coordinate, y_next_coordinate, frequency, time_taken);
        std::vector<double> row_velocity = velocity_trajectory_gen(x_coordinate, x_next_coordinate, frequency, time_taken);
        std::vector<double> col_velocity = velocity_trajectory_gen(y_coordinate, y_next_coordinate, frequency, time_taken);
        std::vector<double> row_acceleration = acceleration_trajectory_gen(x_coordinate, x_next_coordinate, frequency, time_taken);
        std::vector<double> col_acceleration = acceleration_trajectory_gen(y_coordinate, y_next_coordinate, frequency, time_taken);

        // for (int iteration = 0; iteration < row_position.size(); iteration++)
        // {
        //     row_pose_list.push_back(row_position[iteration]);
        // }
        // for (int iteration = 0; iteration < col_position.size(); iteration++)
        // {
        //     col_pose_list.push_back(col_position[iteration]);
        // }
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

        total_time = time_taken + total_time;
    }

    Curve *curve = new BSpline();
    if (C)
    {
        curve->set_steps(80);
    }
    else
    {
        curve->set_steps(40);
    }

    while (line_segmented_path.size() > 0)
    {
        curve->add_way_point(line_segmented_path.front());
        std::cout << "X: " << line_segmented_path.front().x << "\t"
                  << "Y: " << line_segmented_path.front().y << std::endl;
        line_segmented_path.pop_front();
    }

    std::cout << "Number of items in row_vel_list: " << row_vel_list.size() << "\t"
              << "Number of items in row_acc_list: " << row_acc_list.size() << "\t"
              << "Total time taken: " << (int)(total_time * frequency) << "\t"
              << "Number of items in Spline: " << curve->node_count() << std::endl;

    std::vector<double> x_spline, y_spline;
    for (int i = 0; i < curve->node_count(); i++)
    {
        quadrotor_utility::write_path(outfile, curve->node(i).x, curve->node(i).y, 1.0,
                                      row_vel_list[i], col_vel_list[i], 0.0,
                                      row_acc_list[i], col_acc_list[i], 0.0,
                                      0.0, 0.0);
    }

    // std::vector<double> x_axis_tj;
    // for (int i = 1; i <= (int)(total_time * frequency); i++)
    // {
    //     x_axis_tj.push_back(i / frequency);
    // }
    // matplotlibcpp::named_plot("X-Position", x_axis_tj, row_pose_list);
    // matplotlibcpp::named_plot("Y-Position", x_axis_tj, col_pose_list);
    // matplotlibcpp::named_plot("X-Velocity", x_axis_tj, row_vel_list);
    // matplotlibcpp::named_plot("Y-Velocity", x_axis_tj, col_vel_list);
    // matplotlibcpp::named_plot("X-Acceleration", x_axis_tj, row_acc_list);
    // matplotlibcpp::named_plot("Y-Acceleration", x_axis_tj, col_acc_list);
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

    quadrotor_utility::hover(outfile);
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

std::vector<std::vector<double>> calc_matrix_inverse(int n, std::vector<std::vector<double>> matrix)
{
    int i, j, k;
    std::vector<std::vector<double>> a;
    double t;
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

}; // namespace trajectory_utility
#endif