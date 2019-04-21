#include "trajectory_gen.h"
#include <iomanip>

using namespace std;

vector<double> position_trajectory_gen(double start_pose, double desired_pose, double frequency, double move_time)
{
    vector<double> trajectory;
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

vector<double> velocity_trajectory_gen(double start_pose, double desired_pose, double frequency, double move_time)
{
    vector<double> trajectory_derivative;
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

vector<double> acceleration_trajectory_gen(double start_pose, double desired_pose, double frequency, double move_time)
{
    vector<double> trajectory_derivative_two;
    double time_frequency = move_time * frequency;

    for (int time = 1; time <= time_frequency; time++)
    {
        trajectory_derivative_two.push_back(frequency * (desired_pose - start_pose) *
                                            (60.0 * (time)*pow(1.0 / time_frequency, 3.0) -
                                             180.0 * pow(time, 2.0) * pow(1.0 / time_frequency, 4.0) +
                                             120.0 * pow(time, 3.0) * (1.0 / time_frequency, 5.0)));
    }
    return trajectory_derivative_two;
}

vector<vector<double>> calc_matrix_inverse(int n, vector<vector<double>> matrix)
{
    int i, j, k;
    vector<vector<double>> a;
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
    cout << "\n\nInverse matrix\n\n";
    for (i = 0; i < n; i++)
    {
        for (j = n; j < 2 * n; j++)
            cout << "\t" << a[i][j];
        cout << "\n";
    }
    return a;
}

void calc_coeff(vector<vector<double>> inv_matrix, double t0, double tf, double q0, double qf, double vel0, double velf, double acc0, double accf)
{
    vector<double> coeff_vector;

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