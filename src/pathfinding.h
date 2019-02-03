/**
 * @file pathfinding.h
 * @author Leander Feppon and Bilal Lahmami
 * @date 10 Dec 2018
 * @brief Finds a navigable path using the A* algorithm
 * @see https://www.bedroomlan.org/projects/libastar/
 */

#ifndef SWAGBOY_PATHFINDING_H
#define SWAGBOY_PATHFINDING_H

#include <stdbool.h>
#include "astar/astar.h"

#define WIDTH 19 //1 square = 10cm2
#define HEIGHT 27

#define NB_PUCKS 9

#define PUCK 1

#define get_map( x, y ) field[ ( y ) * WIDTH + ( x ) ]
#define set_map( x, y, val ) field[ ( y ) * WIDTH + ( x ) ] = ( val )

#define pose_to_w_index( x ) (int)( x / 10 + 9 )
#define pose_to_h_index( y ) (int)( 26 - ( y / 10 ) )


extern char field[ WIDTH * HEIGHT ];


typedef struct {

    float x;
    float y;
    bool captured;
} puck_t;

typedef struct {

    astar_t* pathfinder;
    direction_t* directions;

    uint32_t x0;
    uint32_t y0;
} PATHFINDER;

extern puck_t pucks[ NB_PUCKS ];
extern PATHFINDER *pf;

/**
 * @brief User-defined function giving the cost for one square of the grid
 * @param x
 * @param y
 * @return the cost of the square
 */
extern uint8_t get_costs (uint32_t x, uint32_t y);

/**
 * @brief Allocate enough memory for the pf pointer and set the origin of the map to (0,0) and of the robot to
 * WIDHT/2 and HEIGHT-1
 * @return true if the pointer has been correctly set
 */
bool init_pathfinder();

/**
 * @brief Frees the memory taken by pf
 */
void uninit_pathfinder();

/**
 * @brief Calculates a navigable path, from (pf->x0, pf->y0) to (w, h)
 * @param w wth tile of field
 * @param h hth tile of field
 * @return true if a path has been found (partial or complete)
 */
bool calculate_path_to( uint32_t w, uint32_t h );

#endif //SWAGBOY_PATHFINDING_H
