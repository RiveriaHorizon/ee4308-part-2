#define _USE_MATH_DEFINES

#include "BSpline.h"
#include "Vector.h"
#include "trajectory_gen.h"
#include <array>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <math.h>

using namespace std;

void floodfill(int start_x, int start_y, int end_x, int end_y);
void Astar(int start_x, int start_y, int end_x, int end_y);
int distance(int x1, int y1, int x2, int y2);
void obstacle_area(int obstacle_x, int obstacle_y);
void reset_array();
void print_array();
int decode_coordinate(double input);
float calc_heading(int i_new, int j_new, int i, int j);
void spline(ofstream &outfile);
void write_path(ofstream &outfile, float pose_x, float pose_y, float pose_z, float vel_x, float vel_y, float vel_z, float acc_x, float acc_y, float acc_z, float heading, float ang_vel);

// Maze constants
const double POINT_A_X = -1.5;
const double POINT_A_Y = 1.5;
const double POINT_B_X = 2;
const double POINT_B_Y = 0.0;
const double POINT_C_X = -1.5;
const double POINT_C_Y = -1.5;
const int CARTESIAN_LIMIT = 3;
const int CARTESIAN_ARRAY_CONSTANT = 8; //Boundary array limit is related to this by multiplying this constant by 6, followed by adding 1
										//Ensure this is an even number (TODO: variable %2)
const int BOUNDARY_ARRAY_LIMIT = 49;
const int OBSTACLE_RADIUS = 4;

int maze[BOUNDARY_ARRAY_LIMIT][BOUNDARY_ARRAY_LIMIT];
int traceback[BOUNDARY_ARRAY_LIMIT][BOUNDARY_ARRAY_LIMIT];
int G_cost[BOUNDARY_ARRAY_LIMIT][BOUNDARY_ARRAY_LIMIT];
int H_cost[BOUNDARY_ARRAY_LIMIT][BOUNDARY_ARRAY_LIMIT];
bool Boolean_array[BOUNDARY_ARRAY_LIMIT][BOUNDARY_ARRAY_LIMIT];

// Used in the Path function
int iteration = 0;
int prev_direction = -1;
double prev_row = -1.0;
double prev_col = -1.0;

int main()
{
	cout << BOUNDARY_ARRAY_LIMIT << endl;
	//start of decode obstacles.txt//
	ifstream infile;
	infile.open("io/obs.txt");

	double obstacle_x, obstacle_y;
	while (!infile.eof())
	{
		infile >> obstacle_y >> obstacle_x;
		obstacle_area(decode_coordinate(obstacle_x), decode_coordinate(obstacle_y));
		// cout << obstacle_x << endl;
	}
	//end of decode obstacles.txt//

	//start of generate droth path//
	ofstream outfile;
	outfile.open("io/path.txt", fstream::trunc);
	cout.setf(ios::showpoint);
	cout.precision(3);
	outfile.setf(ios::showpoint);
	outfile.setf(ios::fixed);
	outfile.precision(3);
	//end of generate droth path//

	//start of generating take-off path from A//
	//takeoff(POINT_A_X, POINT_A_Y, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, outfile);
	double incr_vel = 0.0;
	for (int i = 0; i < 501; i++)
	{
		write_path(outfile, POINT_A_X, POINT_A_Y, incr_vel,
				   0.0, 0.0, 0.3,
				   0.0, 0.0, 0.0,
				   0.0, 0.0);
		incr_vel = incr_vel + 0.002;
	}

	for (int i = 0; i < 101; i++)
	{
		write_path(outfile, POINT_A_X, POINT_A_Y, 1.00,
				   0.0, 0.0, 0.0,
				   0.0, 0.0, 0.0,
				   0.0, 0.0);
	}
	//end of generating take-off path from A//

	//start generating path from A to B//
	// floodfill(decode_coordinate(POINT_A_Y), decode_coordinate(POINT_A_X), decode_coordinate(POINT_B_Y), decode_coordinate(POINT_B_X));
	Astar(decode_coordinate(POINT_A_Y), decode_coordinate(POINT_A_X), decode_coordinate(POINT_B_Y), decode_coordinate(POINT_B_X));
	//end of generating path from A to B//

	//prints array to console for debugging
	// print_array();

	//start interpolating path planned and outputs to path.txt
	spline(outfile);
	//end spline function

	//start generating path from B to C//
	// floodfill(decode_coordinate(POINT_B_Y), decode_coordinate(POINT_B_X), decode_coordinate(POINT_C_Y), decode_coordinate(POINT_C_X));
	Astar(decode_coordinate(POINT_B_Y), decode_coordinate(POINT_B_X), decode_coordinate(POINT_C_Y), decode_coordinate(POINT_C_X));
	//end of generating path from B to C//

	//prints array to console for debugging
	// print_array();

	//start interpolating path planned and outputs to path.txt
	spline(outfile);
	//end reset function

	//pause quadrotor if serial input is true
	//end

	//start of generating landing path from C//
	double desc_vel = 1.0;
	for (int i = 0; i < 71; i++)
	{
		write_path(outfile, POINT_C_X, POINT_C_Y, desc_vel,
				   0.0, 0.0, -0.4,
				   0.0, 0.0, 0.0,
				   0.0, 0.0);
		desc_vel = desc_vel - 0.01;
	}
	desc_vel = 0.3;
	for (int i = 0; i < 31; i++)
	{
		write_path(outfile, POINT_C_X, POINT_C_Y, desc_vel,
				   0.0, 0.0, -0.4,
				   0.0, 0.0, 0.0,
				   0.0, 0.0);
		desc_vel = desc_vel - 0.01;
	}

	for (int i = 0; i < 11; i++)
	{
		write_path(outfile, POINT_C_X, POINT_C_Y, 0.0,
				   0.0, 0.0, 0.0,
				   0.0, 0.0, 0.0,
				   0.0, 0.0);
	}
	//end of generating landing path from C//

	return 0;
}

void floodfill(int start_x, int start_y, int end_x, int end_y)
{
	//resets the path planning array for next journey
	reset_array();
	//end reset function

	int number = 1;

	for (int i = -1; i <= 1; i++) //initial flood fill the surrounding
	{
		for (int j = -1; j <= 1; j++)
		{
			if (maze[start_y + i][start_x + j] == 0 && (start_x + i) >= 0 && (start_y + i) >= 0 && (start_x + i) < BOUNDARY_ARRAY_LIMIT && (start_y + i) < BOUNDARY_ARRAY_LIMIT)
			{
				maze[start_y + i][start_x + j] = 1;
			}
		}
	}

	for (int z = 0; z < 99; z++) //flood fill the rest maze
	{
		for (int i = 0; i < BOUNDARY_ARRAY_LIMIT; i++)
		{
			for (int j = 0; j < BOUNDARY_ARRAY_LIMIT; j++)
			{
				if (maze[i][j] == number)
				{
					for (int a = -1; a <= 1; a++)
					{
						for (int b = -1; b <= 1; b++)
						{
							if (maze[i + a][j + b] == 0 && ((i + a) >= 0) && ((j + b) >= 0) && ((i + a) < BOUNDARY_ARRAY_LIMIT) && ((j + b) < BOUNDARY_ARRAY_LIMIT))
							{
								maze[i + a][j + b] = number + 1;
							}
						}
					}
				}
			}
		}
		number++;
	}

	maze[start_y][start_x] = 0; //set back the initial start point

	for (int i = 0; i < BOUNDARY_ARRAY_LIMIT; i++)
	{
		for (int j = 0; j < BOUNDARY_ARRAY_LIMIT; j++)
		{
			if (traceback[i][j] != 99)
			{
				traceback[i][j] = 11111;
			}
		}
	}

	int count = traceback[end_y][end_x] = maze[end_y][end_x];
	while (count != 0)
	{
		for (int i = -1; i <= 1; i++)
		{
			for (int j = -1; j <= 1; j++)
			{
				if (maze[end_y + i][end_x + j] < maze[end_y][end_x])
				{
					traceback[end_y + i][end_x + j] = --count;
					end_y = end_y + i;
					end_x = end_x + j;
					i = j = 2;
				}
			}
		}
	}
}

void Astar(int start_x, int start_y, int end_x, int end_y)
{
	//resets the path planning array for next journey
	reset_array();
	//end reset function

	int count = 0;
	Boolean_array[start_y][start_x] = true;
	/*
		Generating all the 8 successor of this cell

			N.W   N   N.E
			  \   |   /
			   \  |  /
			W----Cell----E
				/  | \
			   /   |  \
			S.W    S   S.E

		Cell-->Popped Cell (i, j)
		N -->  North       (i-1, j)
		S -->  South       (i+1, j)
		E -->  East        (i, j+1)
		W -->  West           (i, j-1)
		N.E--> North-East  (i-1, j+1)
		N.W--> North-West  (i-1, j-1)
		S.E--> South-East  (i+1, j+1)
		S.W--> South-West  (i+1, j-1)*/

	G_cost[start_y][start_x] = 0;
	//Start of generate G cost for each cells//
	while (count < BOUNDARY_ARRAY_LIMIT)
	{
		for (int outer_y = -count; outer_y <= count; outer_y++) //count outer y boundary
		{
			for (int outer_x = -count; outer_x <= count; outer_x++) //count outer x boundary
			{
				if (G_cost[outer_y + start_y][outer_x + start_x] != 99 &&
					(outer_y + start_y) >= 0 &&
					(outer_x + start_x) >= 0 &&
					(outer_y + start_y) < BOUNDARY_ARRAY_LIMIT &&
					(outer_x + start_x) < BOUNDARY_ARRAY_LIMIT) //check for obstacle & boundary
				{
					for (int inner_y = -1; inner_y <= 1; inner_y++) //count NSEW of outer y boundary
					{
						for (int inner_x = -1; inner_x <= 1; inner_x++) //count NSEW of outer x boundary
						{
							if (G_cost[outer_y + inner_y + start_y][outer_x + inner_x + start_x] != 99 &&
								(outer_y + inner_y + start_y) >= 0 &&
								(outer_x + inner_x + start_x) >= 0 &&
								(outer_y + inner_y + start_y) < BOUNDARY_ARRAY_LIMIT &&
								(outer_x + inner_x + start_x) < BOUNDARY_ARRAY_LIMIT) //check for obstacle & boundary
							{
								if ((inner_y == -1 && inner_x == 1) ||
									(inner_y == -1 && inner_x == -1) ||
									(inner_y == 1 && inner_x == 1) ||
									(inner_y == 1 && inner_x == -1)) //check for NE, NW, SE, SW condition
								{
									if ((G_cost[outer_y + inner_y + start_y][outer_x + inner_x + start_x] == 0 ||
										 G_cost[outer_y + inner_y + start_y][outer_x + inner_x + start_x] > G_cost[outer_y + start_y][outer_x + start_x] + 14)) //check for lowest G cost condition
									{
										G_cost[outer_y + inner_y + start_y][outer_x + inner_x + start_x] = G_cost[outer_y + start_y][outer_x + start_x] + 14; //input value
									}
								}
								else if ((inner_y == -1 && inner_x == 0) ||
										 (inner_y == 1 && inner_x == 0) ||
										 (inner_y == 0 && inner_x == 1) ||
										 (inner_y == 0 && inner_x == -1)) //check for N, S, E, W condition
								{
									if ((G_cost[outer_y + inner_y + start_y][outer_x + inner_x + start_x] == 0 || G_cost[outer_y + inner_y + start_y][outer_x + inner_x + start_x] > G_cost[outer_y + start_y][outer_x + start_x] + 10)) //check for lowest G cost condition
									{
										G_cost[outer_y + inner_y + start_y][outer_x + inner_x + start_x] = G_cost[outer_y + start_y][outer_x + start_x] + 10; //input value
									}
								}
							}
						}
					}
				}
			}
		}
		count++;
	}

	G_cost[start_y][start_x] = 0;

	//End of generate G cost for each cells//

	//Start of generate H cost for each cells//
	for (int i = 0; i < BOUNDARY_ARRAY_LIMIT; i++)
	{
		for (int j = 0; j < BOUNDARY_ARRAY_LIMIT; j++)
		{
			H_cost[i][j] = distance(end_y, end_x, i, j); //euclidean distance to approximate the H cost
		}
	}
	//End of generate H cost for each cells//

	//Start of generate F cost for each cells//
	for (int i = 0; i < BOUNDARY_ARRAY_LIMIT; i++)
	{
		for (int j = 0; j < BOUNDARY_ARRAY_LIMIT; j++)
		{
			if (maze[i][j] != 99 || (i == start_y && j == start_x))
			{
				maze[i][j] = G_cost[i][j] + H_cost[i][j]; //F=G+H
			}
		}
	}
	maze[start_y][start_x] = traceback[start_y][start_x] = 0;
	//End of generate F cost for each cells//

	//Start of A* traceback//
	for (int i = 0; i < BOUNDARY_ARRAY_LIMIT; i++)
	{
		for (int j = 0; j < BOUNDARY_ARRAY_LIMIT; j++)
		{
			if (traceback[i][j] != 99 || (i == start_y && j == start_x))
			{
				traceback[i][j] = 11111;
			}
		}
	}

	int lowest_F, lowest_G, new_x = 99, new_y = 99, new_end_x = end_x, new_end_y = end_y;

	traceback[end_y][end_x] = count = 1000; //set end point 1000 due to flipping
	while ((new_end_x != start_x) || (new_end_y != start_y))
	{
		lowest_F = lowest_G = 999;
		for (int i = -1; i <= 1; i++)
		{
			for (int j = -1; j <= 1; j++)
			{
				if (maze[new_end_y + i][new_end_x + j] != 99 && (i != 0 || j != 0) && (new_end_x + j) >= 0 && (new_end_y + i) >= 0 && (new_end_x + j) < BOUNDARY_ARRAY_LIMIT && (new_end_y + i) < BOUNDARY_ARRAY_LIMIT) //check for obstacle & boundary
				{
					if (maze[new_end_y + i][new_end_x + j] < lowest_F) //check for lowest F cost
					{
						lowest_F = maze[new_end_y + i][new_end_x + j];
						lowest_G = G_cost[new_end_y + i][new_end_x + j];
						new_x = new_end_x + j;
						new_y = new_end_y + i;
					}
					else if (maze[new_end_y + i][new_end_x + j] == lowest_F && G_cost[new_end_y + i][new_end_x + j] < lowest_G) //check for lowest G cost if F cost the same
					{
						lowest_F = maze[new_end_y + i][new_end_x + j];
						lowest_G = G_cost[new_end_y + i][new_end_x + j];
						new_x = new_end_x + j;
						new_y = new_end_y + i;
					}
				}
			}
		}

		traceback[new_y][new_x] = ++count;
		new_end_x = new_x;
		new_end_y = new_y;
	}

	int max_count = traceback[start_y][start_x] - 1000; //minus 1000 due to flipping
	count = traceback[start_y][start_x] = 0;

	while (count != max_count) //increment count to maximum count;
	{
		for (int i = -1; i <= 1; i++)
		{
			for (int j = -1; j <= 1; j++)
			{
				if (traceback[start_y + i][start_x + j] != 99 && traceback[start_y + i][start_x + j] != 11111 && (start_x + j) >= 0 && (start_y + i) >= 0 && (start_x + j) < BOUNDARY_ARRAY_LIMIT && (start_y + i) < BOUNDARY_ARRAY_LIMIT) //check for boundary & obstacle
				{
					if (traceback[start_y + i][start_x + j] > count) //check for reverse count
					{
						traceback[start_y + i][start_x + j] = ++count;
						start_y = start_y + i;
						start_x = start_x + j;
						i = j = 2;
					}
				}
			}
		}
	}
	//End of A* traceback//
}

int distance(int x1, int y1, int x2, int y2)
{
	return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2)) * 10;
}

double takeoff(double x_pos, double y_pos, double z_pos, double x_vel, double y_vel, double z_vel, double x_acc, double y_acc, double z_acc, double head_angle, double angular_rate, ofstream &outfile)
{
	outfile << x_pos << " " << y_pos << " " << 22 << " " << z_acc << endl;
	return 0;
}

void obstacle_area(int obstacle_x, int obstacle_y)
{
	for (int i = -OBSTACLE_RADIUS; i <= OBSTACLE_RADIUS; i++)
	{
		for (int j = -OBSTACLE_RADIUS; j <= OBSTACLE_RADIUS; j++)
		{
			maze[obstacle_y + i][obstacle_x + j] = 99;
			traceback[obstacle_y + i][obstacle_x + j] = 99;
			G_cost[obstacle_y + i][obstacle_x + j] = 99;
		}
	}
}

void reset_array()
{
	for (int i = 0; i < BOUNDARY_ARRAY_LIMIT; i++)
	{
		for (int j = 0; j < BOUNDARY_ARRAY_LIMIT; j++)
		{
			if (maze[i][j] != 99)
			{
				maze[i][j] = 0;
				traceback[i][j] = 0;
				G_cost[i][j] = 0;
				H_cost[i][j] = 0;
			}
			Boolean_array[i][j] = false;
		}
	}
}

void print_array()
{
	for (int i = 0; i < BOUNDARY_ARRAY_LIMIT; i++)
	{
		for (int j = 0; j < BOUNDARY_ARRAY_LIMIT; j++)
		{
			cout << "	" << Boolean_array[i][j];
		}
		cout << endl;
	}

	cout << endl;

	for (int i = 0; i < BOUNDARY_ARRAY_LIMIT; i++)
	{
		for (int j = 0; j < BOUNDARY_ARRAY_LIMIT; j++)
		{
			cout << "	" << G_cost[i][j];
		}
		cout << endl;
	}
}

int decode_coordinate(double input)
{
	return (CARTESIAN_LIMIT - (double)input) * CARTESIAN_ARRAY_CONSTANT;
}

double encode_coordinate(int input)
{
	return ((double)input / CARTESIAN_ARRAY_CONSTANT - CARTESIAN_LIMIT) * -1;
}

float calc_heading(int i_new, int j_new, int i, int j)
{
	//North-West	North	 	North-East
	//West	          +				  East
	//South-West	South	    South-East

	// Next path on the East
	if (i_new == i && j_new > j)
	{
		return 0;
	}

	// Next path on the the North-East
	else if (i_new < i && j_new > j)
	{
		return M_PI / 4;
	}

	// Next path on the North
	else if (i_new < i && j_new == j)
	{
		return M_PI / 2;
	}

	// Next path on the North-West
	else if (i_new < i && j_new < j)
	{
		return (3 * M_PI) / 4;
	}

	// Next path on the West
	else if (i_new == i && j_new < j)
	{
		return M_PI;
	}

	// Next path on the South-West
	else if (i_new > i && j_new < j)
	{
		return (5 * M_PI) / 4;
	}

	// Next path on the South
	else if (i_new > i && j_new == j)
	{
		return (3 * M_PI) / 2;
	}

	// Next path on the South-East
	else if (i_new > i && j_new > j)
	{
		return (7 * M_PI) / 4;
	}
}

void write_path(ofstream &outfile, float pose_x, float pose_y, float pose_z, float vel_x, float vel_y, float vel_z, float acc_x, float acc_y, float acc_z, float heading, float ang_vel)
{
	outfile << setprecision(2) << pose_x << " " << pose_y << " " << pose_z << " " << //x,y,z position
		vel_x << " " << vel_y << " " << vel_z << " " <<								 //x,y,z velocity
		acc_x << " " << acc_y << " " << acc_z << " " <<								 //x,y,z acceleration
		heading << " " << ang_vel <<												 //heading
		endl;
}

void spline(ofstream &outfile)
{
	int loop, count,
		row, col,
		check = 0;
	int i, j, i_new, j_new = 0;
	float x, y = 0.0;
	double direction = 0.0;

	double average_velocity = 0.5;
	double frequency = 20;
	vector<double> row_position, col_position,
		row_velocity, col_velocity,
		row_acceleration, col_acceleration;
	deque<double> row_pose_list, col_pose_list,
		row_vel_list, col_vel_list,
		row_acc_list, col_acc_list;

	//Count then number of steps needed to reach the destination
	for (row = 0; row < BOUNDARY_ARRAY_LIMIT; row++)
	{
		for (col = 0; col < BOUNDARY_ARRAY_LIMIT; col++)
		{
			if (traceback[row][col] != 99 && traceback[row][col] != 11111)
			{
				count++;
			}
		}
	}

	// create vector list
	deque<Vector> waypointlist;

	while (loop <= count)
	{
		for (row = 0; row < BOUNDARY_ARRAY_LIMIT; row++)
		{
			for (col = 0; col < BOUNDARY_ARRAY_LIMIT; col++)
			{
				//cout << col << endl;
				if (loop != count)
				{
					// If the number is within the planned path, we record the coordinate
					if (traceback[row][col] == check)
					{
						i = row;
						j = col;
						x = encode_coordinate(i);
						y = encode_coordinate(j);
						Vector coordinate = Vector(x, y, 0.0);
						waypointlist.push_back(coordinate);
					}
					if (traceback[row][col] == check + 1)
					{
						i_new = row;
						j_new = col;
					}
				}
			}
		}

		direction = calc_heading(i_new, j_new, i, j);

		if (prev_direction != direction)
		{
			prev_direction = direction;
			double time_x = abs(x - prev_row) / average_velocity;
			double time_y = abs(y - prev_col) / average_velocity;

			// row_position = position_trajectory_gen(prev_row, x, frequency, time_x);
			// col_position = position_trajectory_gen(prev_col, y, frequency, time_y);
			row_velocity = velocity_trajectory_gen(prev_row, x, frequency, time_x);
			col_velocity = velocity_trajectory_gen(prev_col, y, frequency, time_y);
			row_acceleration = acceleration_trajectory_gen(prev_row, x, frequency, time_x);
			col_acceleration = acceleration_trajectory_gen(prev_col, y, frequency, time_y);

			// for (int iteration = 0; iteration < row_position.size(); iteration++)
			// {
			// 	row_pose_list.push_back(row_position[iteration]);
			// }
			// for (int iteration = 0; iteration < col_position.size(); iteration++)
			// {
			// 	col_pose_list.push_back(col_position[iteration]);
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
		}

		if (prev_row != x)
		{
			prev_row = x;
		}
		if (prev_col != y)
		{
			prev_col = y;
		}

		loop++;
		check++;
	}

	Curve *curve = new BSpline();
	curve->set_steps(6);

	while (waypointlist.size() > 0)
	{
		curve->add_way_point(waypointlist.front());
		waypointlist.pop_front();
	}
	// cout << "Number of items in row_pose_list: " << row_pose_list.size() << endl;
	// cout << "Number of items in col_pose_list: " << col_pose_list.size() << endl;
	cout << "Number of items in row_vel_list: " << row_vel_list.size() << endl;
	cout << "Number of items in col_vel_list: " << col_vel_list.size() << endl;
	cout << "Number of items in row_acc_list: " << row_acc_list.size() << endl;
	cout << "Number of items in col_acc_list: " << col_acc_list.size() << endl;
	cout << "Number of items in curve: " << curve->node_count() << endl;

	for (int i = 1; i < curve->node_count(); ++i)
	{
		write_path(outfile, curve->node(i).x, curve->node(i).y, 1.0,
				   row_vel_list[i], col_vel_list[i], 0.0,
				   row_acc_list[i], col_acc_list[i], 0.0,
				   0.0, 0.0);
	}

	int pause = 0;
	//pause quadrotor if serial input is true
	cin >> pause;
	if (pause == 1)
	{
		for (int i = 0; i < 150; i++)
		{
			write_path(outfile, POINT_B_X, POINT_B_Y, 1.0,
					   0.0, 0.0, 0.0,
					   0.0, 0.0, 0.0,
					   0.0, 0.0);
		}
	}
}
