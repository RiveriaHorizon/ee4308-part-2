#include "maze_utility.h"

namespace maze_algorithm
{
int distance(int x1, int y1, int x2, int y2)
{
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2)) * 10;
}

void floodfill(int start_x, int start_y, int end_x, int end_y,
               int (*maze)[maze_utility::BOUNDARY_ARRAY_LIMIT][maze_utility::BOUNDARY_ARRAY_LIMIT],
               int (*traceback)[maze_utility::BOUNDARY_ARRAY_LIMIT][maze_utility::BOUNDARY_ARRAY_LIMIT])
{
    int number = 1;

    for (int i = -1; i <= 1; i++) //initial flood fill the surrounding
    {
        for (int j = -1; j <= 1; j++)
        {
            if ((*maze)[start_y + i][start_x + j] == 0 &&
                (start_x + i) >= 0 && (start_y + i) >= 0 &&
                (start_x + i) < maze_utility::BOUNDARY_ARRAY_LIMIT &&
                (start_y + i) < maze_utility::BOUNDARY_ARRAY_LIMIT)
            {
                (*maze)[start_y + i][start_x + j] = 1;
            }
        }
    }

    for (int z = 0; z < 99; z++) //flood fill the rest maze
    {
        for (int i = 0; i < maze_utility::BOUNDARY_ARRAY_LIMIT; i++)
        {
            for (int j = 0; j < maze_utility::BOUNDARY_ARRAY_LIMIT; j++)
            {
                if ((*maze)[i][j] == number)
                {
                    for (int a = -1; a <= 1; a++)
                    {
                        for (int b = -1; b <= 1; b++)
                        {
                            if ((*maze)[i + a][j + b] == 0 &&
                                ((i + a) >= 0) && ((j + b) >= 0) &&
                                ((i + a) < maze_utility::BOUNDARY_ARRAY_LIMIT) &&
                                ((j + b) < maze_utility::BOUNDARY_ARRAY_LIMIT))
                            {
                                (*maze)[i + a][j + b] = number + 1;
                            }
                        }
                    }
                }
            }
        }
        number++;
    }

    (*maze)[start_y][start_x] = 0; //set back the initial start point

    for (int i = 0; i < maze_utility::BOUNDARY_ARRAY_LIMIT; i++)
    {
        for (int j = 0; j < maze_utility::BOUNDARY_ARRAY_LIMIT; j++)
        {
            if ((*traceback)[i][j] != 99)
            {
                (*traceback)[i][j] = 11111;
            }
        }
    }

    int count = (*traceback)[end_y][end_x] = (*maze)[end_y][end_x];
    while (count != 0)
    {
        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                if (maze[end_y + i][end_x + j] < maze[end_y][end_x])
                {
                    (*traceback)[end_y + i][end_x + j] = --count;
                    end_y = end_y + i;
                    end_x = end_x + j;
                    i = j = 2;
                }
            }
        }
    }
}

void Astar(int start_x, int start_y, int end_x, int end_y,
           int (*maze)[maze_utility::BOUNDARY_ARRAY_LIMIT][maze_utility::BOUNDARY_ARRAY_LIMIT],
           int (*traceback)[maze_utility::BOUNDARY_ARRAY_LIMIT][maze_utility::BOUNDARY_ARRAY_LIMIT],
           int (*G_cost)[maze_utility::BOUNDARY_ARRAY_LIMIT][maze_utility::BOUNDARY_ARRAY_LIMIT],
           int (*H_cost)[maze_utility::BOUNDARY_ARRAY_LIMIT][maze_utility::BOUNDARY_ARRAY_LIMIT],
           bool (*check_cost)[maze_utility::BOUNDARY_ARRAY_LIMIT][maze_utility::BOUNDARY_ARRAY_LIMIT])
{
    int count = 0;
    /*
		Generating all the 8 successor of this cell

			N.W   N   N.E
			  \   |   /
			   \  |  /
			W----Cell----E
				/  | \
			   /   |  \
			S.W    S   S.E

		Cell-->Popped Cell (y, x)
		N -->  North       (y-1, x)
		S -->  South       (y+1, x)
		E -->  East        (y, x+1)
		W -->  West        (y, x-1)
		N.E--> North-East  (y-1, x+1)
		N.W--> North-West  (y-1, x-1)
		S.E--> South-East  (y+1, x+1)
		S.W--> South-West  (y+1, x-1)*/

    //Start of generate G cost for each cells//
    (*G_cost)[start_y][start_x] = 0;
    (*check_cost)[start_y][start_x] = true;
    while (count < maze_utility::BOUNDARY_ARRAY_LIMIT)
    {
        for (int outer_y = -count; outer_y <= count; outer_y++) //count outer y boundary
        {
            for (int outer_x = -count; outer_x <= count; outer_x++) //count outer x boundary
            {
                if ((*G_cost)[outer_y + start_y][outer_x + start_x] != 99 &&
                    (outer_y + start_y) >= 0 &&
                    (outer_x + start_x) >= 0 &&
                    (outer_y + start_y) < maze_utility::BOUNDARY_ARRAY_LIMIT &&
                    (outer_x + start_x) < maze_utility::BOUNDARY_ARRAY_LIMIT) //check for obstacle & boundary
                {
                    if ((*check_cost)[outer_y + start_y][outer_x + start_x]) //check for path taken to prevent open new loop
                    {
                        for (int inner_y = -1; inner_y <= 1; inner_y++) //count NSEW of outer y boundary
                        {
                            for (int inner_x = -1; inner_x <= 1; inner_x++) //count NSEW of outer x boundary
                            {
                                if ((*G_cost)[outer_y + inner_y + start_y][outer_x + inner_x + start_x] != 99 &&
                                    (outer_y + inner_y + start_y) >= 0 &&
                                    (outer_x + inner_x + start_x) >= 0 &&
                                    (outer_y + inner_y + start_y) < maze_utility::BOUNDARY_ARRAY_LIMIT &&
                                    (outer_x + inner_x + start_x) < maze_utility::BOUNDARY_ARRAY_LIMIT) //check for obstacle & boundary
                                {
                                    if ((inner_y == -1 && inner_x == 1) ||
                                        (inner_y == -1 && inner_x == -1) ||
                                        (inner_y == 1 && inner_x == 1) ||
                                        (inner_y == 1 && inner_x == -1)) //check for NE, NW, SE, SW condition
                                    {
                                        if (((*G_cost)[outer_y + inner_y + start_y][outer_x + inner_x + start_x] == 0 ||
                                             (*G_cost)[outer_y + inner_y + start_y][outer_x + inner_x + start_x] > (*G_cost)[outer_y + start_y][outer_x + start_x] + 14)) //check for lowest G cost condition
                                        {
                                            (*G_cost)[outer_y + inner_y + start_y][outer_x + inner_x + start_x] = (*G_cost)[outer_y + start_y][outer_x + start_x] + 14; //input value
                                        }
                                    }
                                    else if ((inner_y == -1 && inner_x == 0) ||
                                             (inner_y == 1 && inner_x == 0) ||
                                             (inner_y == 0 && inner_x == 1) ||
                                             (inner_y == 0 && inner_x == -1)) //check for N, S, E, W condition
                                    {
                                        if (((*G_cost)[outer_y + inner_y + start_y][outer_x + inner_x + start_x] == 0 ||
                                             (*G_cost)[outer_y + inner_y + start_y][outer_x + inner_x + start_x] > (*G_cost)[outer_y + start_y][outer_x + start_x] + 10)) //check for lowest G cost condition
                                        {
                                            (*G_cost)[outer_y + inner_y + start_y][outer_x + inner_x + start_x] = (*G_cost)[outer_y + start_y][outer_x + start_x] + 10; //input value
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        count++;

        for (int check_y = 0; check_y < maze_utility::BOUNDARY_ARRAY_LIMIT; check_y++)
        {
            for (int check_x = 0; check_x < maze_utility::BOUNDARY_ARRAY_LIMIT; check_x++)
            {
                if ((*G_cost)[check_y][check_x] != 99 && (*G_cost)[check_y][check_x] != 0 || (check_y == start_y && check_x == start_x)) //check for none obstacles, none zeros and starting point
                {
                    (*check_cost)[check_y][check_x] = true;
                }
            }
        }
    }
    (*G_cost)[start_y][start_x] = 0;
    //End of generate G cost for each cells//

    //Start of generate H cost for each cells//
    for (int i = 0; i < maze_utility::BOUNDARY_ARRAY_LIMIT; i++)
    {
        for (int j = 0; j < maze_utility::BOUNDARY_ARRAY_LIMIT; j++)
        {
            (*H_cost)[i][j] = distance(end_y, end_x, i, j); //euclidean distance to approximate the H cost
        }
    }
    //End of generate H cost for each cells//

    //Start of generate F cost for each cells//
    for (int i = 0; i < maze_utility::BOUNDARY_ARRAY_LIMIT; i++)
    {
        for (int j = 0; j < maze_utility::BOUNDARY_ARRAY_LIMIT; j++)
        {
            if ((*maze)[i][j] != 99 || (i == start_y && j == start_x))
            {
                (*maze)[i][j] = (*G_cost)[i][j] + (*H_cost)[i][j]; //F=G+H
            }
        }
    }
    (*maze)[start_y][start_x] = (*traceback)[start_y][start_x] = 0;
    //End of generate F cost for each cells//

    //Start of A* traceback//
    for (int i = 0; i < maze_utility::BOUNDARY_ARRAY_LIMIT; i++)
    {
        for (int j = 0; j < maze_utility::BOUNDARY_ARRAY_LIMIT; j++)
        {
            if ((*traceback)[i][j] != 99 || (i == start_y && j == start_x))
            {
                (*traceback)[i][j] = 11111;
            }
        }
    }

    int lowest_F, lowest_G, new_x = 99, new_y = 99, new_end_x = end_x, new_end_y = end_y;

    (*traceback)[end_y][end_x] = count = 10000; //set end point 1000 due to flipping
    while ((new_end_x != start_x) || (new_end_y != start_y))
    {
        lowest_F = lowest_G = 999;
        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                if ((*maze)[new_end_y + i][new_end_x + j] != 99 && (i != 0 || j != 0) && (new_end_x + j) >= 0 && (new_end_y + i) >= 0 && (new_end_x + j) < maze_utility::BOUNDARY_ARRAY_LIMIT && (new_end_y + i) < maze_utility::BOUNDARY_ARRAY_LIMIT) //check for obstacle & boundary
                {
                    if ((*maze)[new_end_y + i][new_end_x + j] < lowest_F) //check for lowest F cost
                    {
                        lowest_F = (*maze)[new_end_y + i][new_end_x + j];
                        lowest_G = (*G_cost)[new_end_y + i][new_end_x + j];
                        new_x = new_end_x + j;
                        new_y = new_end_y + i;
                    }
                    else if ((*maze)[new_end_y + i][new_end_x + j] == lowest_F && (*G_cost)[new_end_y + i][new_end_x + j] < lowest_G) //check for lowest G cost if F cost the same
                    {
                        lowest_F = (*maze)[new_end_y + i][new_end_x + j];
                        lowest_G = (*G_cost)[new_end_y + i][new_end_x + j];
                        new_x = new_end_x + j;
                        new_y = new_end_y + i;
                    }
                }
            }
        }
        (*traceback)[new_y][new_x] = ++count;
        new_end_x = new_x;
        new_end_y = new_y;
    }

    int max_count = (*traceback)[start_y][start_x] - 10000; //minus 1000 due to flipping
    count = (*traceback)[start_y][start_x] = 0;

    while (count != max_count) //increment count to maximum count;
    {
        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                if ((*traceback)[start_y + i][start_x + j] != 99 && (*traceback)[start_y + i][start_x + j] != 11111 && (start_x + j) >= 0 && (start_y + i) >= 0 && (start_x + j) < maze_utility::BOUNDARY_ARRAY_LIMIT && (start_y + i) < maze_utility::BOUNDARY_ARRAY_LIMIT) //check for boundary & obstacle
                {
                    if ((*traceback)[start_y + i][start_x + j] > count) //check for reverse count
                    {
                        (*traceback)[start_y + i][start_x + j] = ++count;
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
}; // namespace maze_algorithm
