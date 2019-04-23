#ifndef TRAJECTORY_UTILITY_H
#define TRAJECTORY_UTILITY_H

#define _USE_MATH_DEFINES
#include <algorithm>
#include <cmath>
#include <deque>
#include <iostream>
#include <vector>

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

    std::cout << "Time frequency: " << time_frequency << "\t" << move_time << "\t" << frequency << std::endl;
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
            i_next_coordinate = 0, j_next_coordinate = 0;
        double x_coordinate = 0, y_coordinate = 0;
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

    line_segmented_path.push_back(Vector(array_coordinate_list[FIRST].x, array_coordinate_list[FIRST].y, 0.0));
    line_segmented_path.push_back(Vector(array_coordinate_list[FIRST].x, array_coordinate_list[FIRST].y, 0.0));
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

                if (calculated_constraint > 30 && calculated_constraint < 62)
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
    line_segmented_path.push_back(Vector(array_coordinate_list[LAST].x, array_coordinate_list[LAST].y, 0.0));
    line_segmented_path.push_back(Vector(array_coordinate_list[LAST].x, array_coordinate_list[LAST].y, 0.0));
    line_segmented_path.push_back(Vector(array_coordinate_list[LAST].x, array_coordinate_list[LAST].y, 0.0));

    return line_segmented_path;
}

double calc_distance(double x1, double x2, double y1, double y2)
{
    return sqrt(pow(x1 - x2, 2.0) + pow(y1 - y2, 2.0));
}

void plot_pose_vel_acc(Curve *curve, double total_time, double frequency,
                       std::vector<double> row_velocity, std::vector<double> col_velocity,
                       std::vector<double> row_acceleration, std::vector<double> col_acceleration,
                       std::string name)
{
    std::vector<double> x_axis_tj;
    for (int i = 1; i <= (int)(total_time * frequency); i++)
    {
        x_axis_tj.push_back(i / frequency);
    }

    matplotlibcpp::clf();
    matplotlibcpp::named_plot("X-Velocity", x_axis_tj, row_velocity);
    matplotlibcpp::named_plot("Y-Velocity", x_axis_tj, col_velocity);
    matplotlibcpp::named_plot("X-Acceleration", x_axis_tj, row_acceleration);
    matplotlibcpp::named_plot("Y-Acceleration", x_axis_tj, col_acceleration);
    matplotlibcpp::title("Minimum Jerk Trajectory");
    matplotlibcpp::xlabel("Time [s]");
    matplotlibcpp::ylabel("Velocity [m/s] and Acceleration [m/s2]");
    matplotlibcpp::legend();
    matplotlibcpp::save("graph_plots/vel_acc_" + name + ".png");
    matplotlibcpp::close();
}

void plot_path(Curve *curve, std::string name)
{
    std::vector<double> x_spline, y_spline;
    for (int i = 0; i < curve->node_count(); i++)
    {
        x_spline.push_back(curve->node(i).x);
        y_spline.push_back(curve->node(i).y);
    }

    matplotlibcpp::clf();
    matplotlibcpp::named_plot("Path", y_spline, x_spline);
    matplotlibcpp::xlim(3, -3);
    matplotlibcpp::ylim(-3, 3);
    matplotlibcpp::xlabel("Y-Coordinate");
    matplotlibcpp::ylabel("X-Coordinate");

    matplotlibcpp::legend();
    matplotlibcpp::save("graph_plots/position_" + name + ".png");
    matplotlibcpp::close();
}

void trajectory_smoothing(std::ofstream &outfile,
                          int (*traceback)[maze_utility::BOUNDARY_ARRAY_LIMIT][maze_utility::BOUNDARY_ARRAY_LIMIT],
                          bool is_going_to_C)
{
    // Constants initialization
    const int FREQUENCY_A_B = 11;       // Frequency from A to B for Vel-Acc profiling
    const int FREQUENCY_B_C = 12;       // Frequency from B to C for Vel-Acc profiling
    const double STEP_MULTIPLIER = 1.3; // Constant to reduce spline step size

    // Variable initialization
    double total_time = 0.0;
    double frequency;
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
    if (is_going_to_C)
    {
        frequency = FREQUENCY_B_C;
    }
    else
    {
        frequency = FREQUENCY_A_B;
    }

    double x_next_coordinate, y_next_coordinate,
        x_coordinate, y_coordinate;
    for (int i = 0; i < cartesian_coordinate_list.size(); i++)
    {
        if (i != cartesian_coordinate_list.size() - 1)
        {
            x_next_coordinate = cartesian_coordinate_list[i + 1].x;
            y_next_coordinate = cartesian_coordinate_list[i + 1].y;
            x_coordinate = cartesian_coordinate_list[i].x;
            y_coordinate = cartesian_coordinate_list[i].y;
        }
        else
        {
            x_next_coordinate = cartesian_coordinate_list[i].x;
            y_next_coordinate = cartesian_coordinate_list[i].y;
            x_coordinate = cartesian_coordinate_list[i].x;
            y_coordinate = cartesian_coordinate_list[i].y;
        }
        double average_velocity = 0.5;
        double time_taken = calc_distance(x_coordinate, x_next_coordinate, y_coordinate, y_next_coordinate) / average_velocity;
        total_time = time_taken + total_time;
        std::cout << total_time << "\t" << x_coordinate << "\t" << x_next_coordinate << "\t" << y_coordinate << "\t" << y_next_coordinate << "\t" << i << std::endl;
    }
    std::cout << "Total time taken: " << total_time << std::endl;
    std::vector<double> row_velocity = velocity_trajectory_gen(cartesian_coordinate_list[0].x, cartesian_coordinate_list[cartesian_coordinate_list.size() - 1].x, frequency, total_time);
    std::vector<double> col_velocity = velocity_trajectory_gen(cartesian_coordinate_list[0].y, cartesian_coordinate_list[cartesian_coordinate_list.size() - 1].y, frequency, total_time);
    std::vector<double> row_acceleration = acceleration_trajectory_gen(cartesian_coordinate_list[0].x, cartesian_coordinate_list[cartesian_coordinate_list.size() - 1].x, frequency, total_time);
    std::vector<double> col_acceleration = acceleration_trajectory_gen(cartesian_coordinate_list[0].y, cartesian_coordinate_list[cartesian_coordinate_list.size() - 1].y, frequency, total_time);

    Curve *curve = new BSpline();
    curve->set_steps(total_time * frequency / (STEP_MULTIPLIER * cartesian_coordinate_list.size())); //Number of intermediary steps from one point to another
    std::cout << total_time * frequency / (STEP_MULTIPLIER * cartesian_coordinate_list.size()) << std::endl;

    while (cartesian_coordinate_list.size() > 0)
    {
        curve->add_way_point(cartesian_coordinate_list.front());
        cartesian_coordinate_list.pop_front();
    }

    std::cout << "Number of items in row_vel_list: " << row_velocity.size() << "\t"
              << "Number of items in row_acc_list: " << row_acceleration.size() << "\t"
              << "Total time taken: " << (int)(total_time * frequency) << "\t"
              << "Number of items in Spline: " << curve->node_count() << std::endl;

    for (int i = 0; i < curve->node_count(); i++)
    {
        quadrotor_utility::write_path(outfile, curve->node(i).x, curve->node(i).y, 1.0,
                                      row_velocity[i], col_velocity[i], 0.0,
                                      row_acceleration[i], col_acceleration[i], 0.0,
                                      0.0, 0.0);
    }
    if (is_going_to_C)
    {
        plot_pose_vel_acc(curve, total_time, frequency,
                          row_velocity, col_velocity,
                          row_acceleration, col_acceleration, "B-C");
        plot_path(curve, "B-C");
    }
    else
    {
        plot_pose_vel_acc(curve, total_time, frequency,
                          row_velocity, col_velocity,
                          row_acceleration, col_acceleration, "A-B");
        plot_path(curve, "A-B");
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