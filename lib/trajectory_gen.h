#ifndef TRAJECTORY_GEN_H
#define TRAJECTORY_GEN_H

#include <algorithm>
#include <iostream>
#include <cmath>
#include <vector>

using namespace std;

// Creates the minimum jerk trajectory that accepts the start position, desired position,
// the frequency of the control system and a time parameter that indicates how long it
// should take to get from the start position to the desired position
vector<double> position_trajectory_gen(double start_pose, double desired_pose, double frequency, double move_time);
vector<double> velocity_trajectory_gen(double start_pose, double desired_pose, double frequency, double move_time);
vector<double> acceleration_trajectory_gen(double start_pose, double desired_pose, double frequency, double move_time);
vector<double> calc_matrix_inverse(int n, float matrix[100][200]);
void calc_coeff(vector<vector<double>> inv_matrix, double t0, double tf, double q0, double qf, double vel0, double velf, double acc0, double accf);

#endif