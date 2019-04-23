#define _USE_MATH_DEFINES

//From cpp-spline
#include "BSpline.h"
#include "Curve.h"
#include "Vector.h"
//From matplotlibcpp
#include "matplotlibcpp.h"
//For path-planning
#include "maze_algorithm.h"
#include "maze_utility.h"
#include "trajectory_gen.h"

void trajectory_smoothing(std::ofstream &outfile);

int maze[maze_utility::BOUNDARY_ARRAY_LIMIT][maze_utility::BOUNDARY_ARRAY_LIMIT];
int traceback[maze_utility::BOUNDARY_ARRAY_LIMIT][maze_utility::BOUNDARY_ARRAY_LIMIT];
int G_cost[maze_utility::BOUNDARY_ARRAY_LIMIT][maze_utility::BOUNDARY_ARRAY_LIMIT];
int H_cost[maze_utility::BOUNDARY_ARRAY_LIMIT][maze_utility::BOUNDARY_ARRAY_LIMIT];
bool Boolean_array[maze_utility::BOUNDARY_ARRAY_LIMIT][maze_utility::BOUNDARY_ARRAY_LIMIT];

int main()
{
	//start of decode obstacles.txt//
	std::ifstream infile;
	infile.open("io/obs.txt");

	double obstacle_x, obstacle_y;
	while (!infile.eof())
	{
		infile >> obstacle_y >> obstacle_x;
		maze_utility::obstacle_area(maze_utility::decode_coordinate(obstacle_x, maze_utility::CARTESIAN_LIMIT, maze_utility::CARTESIAN_MULTIPLIER),
									maze_utility::decode_coordinate(obstacle_y, maze_utility::CARTESIAN_LIMIT, maze_utility::CARTESIAN_MULTIPLIER),
									maze_utility::OBSTACLE_RADIUS, &maze, &traceback, &G_cost);
		// std::cout << obstacle_x << endl;
	}
	//end of decode obstacles.txt//

	//start of generate droth path//
	std::ofstream outfile;
	outfile.open("io/path.txt", std::fstream::trunc);
	std::cout.setf(std::ios::showpoint);
	std::cout.precision(3);
	outfile.setf(std::ios::showpoint);
	outfile.setf(std::ios::fixed);
	outfile.precision(3);
	//end of generate droth path//

	//start of generating take-off path from A//
	//takeoff(POINT_A_X, POINT_A_Y, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, outfile);
	maze_utility::take_off(outfile);
	//end of generating take-off path from A//

	maze_utility::reset_maze(maze_utility::BOUNDARY_ARRAY_LIMIT, &maze, &traceback, &G_cost, &H_cost);

	//start generating path from A to B//
	// floodfill(decode_coordinate(POINT_A_Y), decode_coordinate(POINT_A_X), decode_coordinate(POINT_B_Y), decode_coordinate(POINT_B_X));
	maze_algorithm::Astar(maze_utility::decode_coordinate(maze_utility::POINT_A_Y, maze_utility::CARTESIAN_LIMIT, maze_utility::CARTESIAN_MULTIPLIER),
						  maze_utility::decode_coordinate(maze_utility::POINT_A_X, maze_utility::CARTESIAN_LIMIT, maze_utility::CARTESIAN_MULTIPLIER),
						  maze_utility::decode_coordinate(maze_utility::POINT_B_Y, maze_utility::CARTESIAN_LIMIT, maze_utility::CARTESIAN_MULTIPLIER),
						  maze_utility::decode_coordinate(maze_utility::POINT_B_X, maze_utility::CARTESIAN_LIMIT, maze_utility::CARTESIAN_MULTIPLIER),
						  &maze, &traceback, &G_cost, &H_cost);
	//end of generating path from A to B//

	//prints array to console for debugging
	// maze_utility::print_maze(maze_utility::BOUNDARY_ARRAY_LIMIT, &maze, &traceback);

	//start interpolating path planned and outputs to path.txt
	trajectory_utility::trajectory_smoothing(outfile, &traceback, false);
	//end trajectory_smoothing function

	maze_utility::reset_maze(maze_utility::BOUNDARY_ARRAY_LIMIT, &maze, &traceback, &G_cost, &H_cost);

	//start generating path from B to C//
	// floodfill(decode_coordinate(POINT_B_Y), decode_coordinate(POINT_B_X), decode_coordinate(POINT_C_Y), decode_coordinate(POINT_C_X));
	maze_algorithm::Astar(maze_utility::decode_coordinate(maze_utility::POINT_B_Y, maze_utility::CARTESIAN_LIMIT, maze_utility::CARTESIAN_MULTIPLIER),
						  maze_utility::decode_coordinate(maze_utility::POINT_B_X, maze_utility::CARTESIAN_LIMIT, maze_utility::CARTESIAN_MULTIPLIER),
						  maze_utility::decode_coordinate(maze_utility::POINT_C_Y, maze_utility::CARTESIAN_LIMIT, maze_utility::CARTESIAN_MULTIPLIER),
						  maze_utility::decode_coordinate(maze_utility::POINT_C_X, maze_utility::CARTESIAN_LIMIT, maze_utility::CARTESIAN_MULTIPLIER),
						  &maze, &traceback, &G_cost, &H_cost);
	//end of generating path from B to C//

	//prints array to console for debugging
	// maze_utility::print_maze(maze_utility::BOUNDARY_ARRAY_LIMIT, &maze, &traceback);

	//start interpolating path planned and outputs to path.txt
	trajectory_utility::trajectory_smoothing(outfile, &traceback, true);
	//end reset function

	//pause quadrotor if serial input is true
	//end

	//start of generating landing path from C//
	maze_utility::landing(outfile);
	//end of generating landing path from C//

	return 0;
}
