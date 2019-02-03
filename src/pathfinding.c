/**
 * @file pathfinding.c
 * @author Leander Feppon and Bilal Lahmami
 * @date 10 Dec 2018
 * @brief Finds a navigable path using the A* algorithm
 * @see https://www.bedroomlan.org/projects/libastar/
 */
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>

#include "astar/astar.h"


#include "pathfinding.h"



PATHFINDER* pf = NULL;

puck_t pucks[ NB_PUCKS ] =
        {
            {.x = -50, .y = 60, .captured = false },
            {.x = 0, .y = 60, .captured = false },
            {.x = 50, .y = 60, .captured = false },
            {.x = -50, .y = 120, .captured = false },
            {.x = 0, .y = 120, .captured = false },
            {.x = 50, .y = 120, .captured = false },
            {.x = -50, .y = 180, .captured = false },
            {.x = 0, .y = 180, .captured = false },
            {.x = 50, .y = 180, .captured = false },
        };

char field[ WIDTH * HEIGHT ] =
        {
                1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                1, 0, 0, 0, PUCK, 0, 0, 0, 0, PUCK, 0, 0, 0, 0, PUCK, 0, 0, 0, 1,
                1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                1, 0, 0, 0, PUCK, 0, 0, 0, 0, PUCK, 0, 0, 0, 0, PUCK, 0, 0, 0, 1,
                1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                1, 0, 0, 0, PUCK, 0, 0, 0, 0, PUCK, 0, 0, 0, 0, PUCK, 0, 0, 0, 1,
                1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1
        };


uint8_t get_costs ( uint32_t x, uint32_t y ) {

    return (uint8_t) (get_map(x, y ) == PUCK ? COST_BLOCKED : 1);
}

bool init_pathfinder() {

    if ( pf != NULL )
        return false;

    pf = malloc( sizeof( PATHFINDER ) );

    errno = 0;

    if ( pf == NULL )
        fprintf( stderr,  "Impossible d'allouer l'espace mémoire nécessaire pour le pf\n" ),
        exit( EXIT_SUCCESS );

    pf->pathfinder = astar_new( WIDTH, HEIGHT, get_costs, NULL );
    astar_set_origin( pf->pathfinder, 0, 0 );

    pf->x0 = WIDTH / 2 ;
    pf->y0 = HEIGHT - 1;

    return true;
}

void uninit_pathfinder() {

    // NULL check necessary because
    // astar assert(directions != NULL)
    if ( pf->directions != NULL)
        astar_free_directions( pf->directions );

    astar_destroy( pf->pathfinder );

    free( pf );
    pf = NULL;
}
bool calculate_path_to( uint32_t w, uint32_t h ) {

    if ( NULL == pf )
        return false;

    astar_set_origin( pf->pathfinder, w, h );
    astar_run(pf->pathfinder, pf->x0, pf->y0, (const uint32_t) w, (const uint32_t) h);

    return (bool) ( astar_have_route(pf->pathfinder ) );
}