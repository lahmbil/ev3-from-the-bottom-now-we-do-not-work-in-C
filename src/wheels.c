/**
 * @file wheels.c
 * @author Leander Feppon and Bilal Lahmami
 * @date 10 Dec 2018
 * @brief Manages the movements of the robot and its position
 *
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <pthread.h>
#include <unistd.h>

#include "ev3.h"
#include "ev3_tacho.h"
#include "ev3_port.h"

#include "pathfinding.h"

#include "wheels.h"


static void* _move_listener(void *param);
static void _set_command_attr(COMMAND* cmd, int command, float distance, int duration, int angle);
static bool _set_command(WHEELS* m, int command, float distance, int duration, int angle);
static void _run_forever(WHEELS* m);
static void _run_timed(WHEELS* m, int ms);
static void _rotate(WHEELS* m, int angle);
static void _set_speed(WHEELS* m, int speed);


pthread_t thread_id;
pthread_mutex_t lock_motors;
bool running;


static void* _move_listener( void* param ) {

    WHEELS* w = (WHEELS*)param;

    while ( running ) {

        if ( w->cmd->hasChanged ) {

            pthread_mutex_lock(&lock_motors);

            w->cmd->hasChanged = false;


            if (w->cmd->command == ROTATING) {

                w->pose->v += w->cmd->angle < 0 ? w->cmd->angle + 360 : w->cmd->angle;
                w->pose->v %= 360;

            } else if ( w->cmd->command == AUTONAV || w->cmd->command == MOVE_FORWARD || w->cmd->command == MOVE_BACKWARD) {

                bool reverse = w->cmd->command == MOVE_BACKWARD ? true : false;

                float old_x = w->pose->x;
                float old_y = w->pose->y;

                float distance = reverse ? DIST_PER_SEC * -1 : DIST_PER_SEC;
                distance /= 2;
                float distance_traveled = 0;
                float h = distance / sqrtf( 2.0f );

                while (  ! w->cmd->hasChanged && is_moving( w ) ) {

                    pthread_mutex_lock(&lock_motors);

                    if ( w->cmd->distance <= distance_traveled ) {
                        multi_set_tacho_command_inx( w->motors, TACHO_STOP );
                        break;
                    }

                    if (0 == w->pose->v)
                        w->pose->y += distance;

                    else if (180 == w->pose->v)
                        w->pose->y -= distance;

                    else if (90 == w->pose->v)
                        w->pose->x += distance;

                    else if (270 == w->pose->v)
                        w->pose->x -= distance;

                    else {
                        if (45 == w->pose->v)
                            w->pose->x += h, w->pose->y += h;

                        else if (135 == w->pose->v)
                            w->pose->x += h, w->pose->y -= h;

                        else if (225 == w->pose->v)
                            w->pose->x -= h, w->pose->y -= h;

                        else if (315 == w->pose->v)
                            w->pose->x -= h, w->pose->y += h;
                    }

                    distance_traveled += distance;

                    pthread_mutex_unlock(&lock_motors);
                    usleep( 500 * 1000 ); // position update every 500 ms

                }

                // the position is corrected at the end of the movement

                if ( w->cmd->hasChanged && w->cmd->command == STOPPED ) {

                    w->cmd->distance = distance_traveled;
                    break;
                }


                if ( w->pose->v % 90 != 0 ) {

                    float hypothenuse = distance_traveled / sqrtf( 2 );

                    if ( 45 == w->pose->v )
                        w->pose->x = old_x + hypothenuse, w->pose->y = old_y + hypothenuse;

                    else if ( 135 == w->pose->v )
                        w->pose->x = old_x + hypothenuse, w->pose->y = old_y - hypothenuse;

                    else if ( 225 == w->pose->v )
                        w->pose->x = old_x - hypothenuse, w->pose->y = old_y - hypothenuse;

                    else if ( 315 == w->pose->v )
                        w->pose->x = old_x - hypothenuse, w->pose->y = old_y + hypothenuse;
                }

                else {

                    if ( 0 == w->pose->v )
                        w->pose->y = old_y + w->cmd->distance;

                    else if ( 90 == w->pose->v )
                        w->pose->x = old_x + w->cmd->distance;

                    else if ( 180 == w->pose->v )
                        w->pose->y = old_y - w->cmd->distance;

                    else if ( 270 == w->pose->v )
                        w->pose->x = old_x - w->cmd->distance;
                }
            }


            w->cmd->hasChanged = false;


/*            printf("\n");
            printf( "Pose (x): %f\n", w->pose->x );
            printf( "Pose (y): %f\n", w->pose->y );
            printf( "Pose (v): %d\n\n", w->pose->v );*/



        }

        pthread_mutex_unlock(&lock_motors);
        sched_yield();
    }

    pthread_exit( 0 );
}

WHEELS* init_motors( uint8_t left_motor_port, uint8_t right_motor_port ) {

    if ( ev3_tacho_init() < 1 ) exit( EXIT_FAILURE );

    WHEELS* m;

    errno = 0;
    if ( m = malloc( sizeof( WHEELS ) ), NULL == m )
        fprintf( stderr,  "Impossible d'allouer l'espace mémoire nécessaire: %s\n", strerror( errno ) ),
                exit( EXIT_FAILURE );

    if ( m->cmd = malloc(sizeof(COMMAND)), NULL == m )
        fprintf( stderr,  "Impossible d'allouer l'espace mémoire nécessaire: %s\n", strerror( errno ) ),
                exit( EXIT_FAILURE );

    if ( m->pose = malloc(sizeof(POSE)), NULL == m )
        fprintf( stderr,  "Impossible d'allouer l'espace mémoire nécessaire: %s\n", strerror( errno ) ),
                exit( EXIT_FAILURE );


    bool lSuccess = ev3_search_tacho_plugged_in( left_motor_port, EXT_PORT__NONE_, &m->motors[L], 0 );
    bool rSuccess = ev3_search_tacho_plugged_in( right_motor_port, EXT_PORT__NONE_, &m->motors[R], 0 );

    errno = 0;

    if ( !lSuccess || !rSuccess )
        fprintf( stderr,  "Impossible d'initialiser les moteurs\n" ),
                uninit_motors( &m ),
                exit( EXIT_FAILURE );


    set_speed( m, 0.3f );

    running = true;

    if ( pthread_create( &thread_id, NULL, _move_listener, m ) )
        fprintf( stderr,  "Impossible de créer un nouveau thread pour le move listener: %s\n", strerror( errno ) ),
                uninit_motors( &m ),
                exit( EXIT_FAILURE );

    if ( pthread_mutex_init(&lock_motors, NULL) != 0 )
        fprintf( stderr,  "Premier mutex: initialisation impossible: %s\n", strerror( errno ) ),
                uninit_motors( &m ),
                exit( EXIT_FAILURE );


    multi_set_tacho_ramp_down_sp( m->motors, 500 );

    m->pose->x = 0;
    m->pose->y = 0;
    m->pose->v = 0;

    m->cmd->command = STOPPED;
    m->cmd->hasChanged = false;
    m->cmd->distance = 0;
    m->cmd->duration = 0;
    m->cmd->angle = 0;

    return m;
}

void uninit_motors(WHEELS** w) {

    pthread_mutex_lock( &lock_motors );

    printf( "CLOSING MOTORS\n" );

    free( (*w)->pose );
    (*w)->pose = NULL;
    free( (*w)->cmd );
    (*w)->cmd = NULL;
    free( *w );
    *w = NULL;

    running = false;

/*

    if ( pthread_join( thread_id, 0 ) )
        fprintf( stderr,  "Impossible de terminer le thread\n" ),
        exit( EXIT_FAILURE );
*/


    pthread_mutex_destroy( &lock_motors );
}

static void _set_command_attr( COMMAND* cmd, int command, float distance, int duration, int angle ) {

    cmd->command = command;
    cmd->distance = distance;
    cmd->duration = duration;
    cmd->angle = angle % 360;
}

static bool _set_command( WHEELS* m, int command, float distance, int duration, int angle ) {

    COMMAND** cmd = &m->cmd;
    bool couldSet = true;

    if ( command != STOPPED && ( (*cmd)->duration != 0 || (*cmd)->command == ROTATING ) ) {

        while ( is_moving( m ) )
            sched_yield();

        multi_set_tacho_command_inx( m->motors, TACHO_STOP );
        usleep( 500 * 1000 );
    }

    pthread_mutex_lock( &lock_motors );

    switch ( command ) {

        case MOVE_FORWARD:

            if ( 0 == distance && 0 == duration ) {

                _set_command_attr( *cmd, m->pose->speed < 0 ? MOVE_BACKWARD : MOVE_FORWARD, 0, 0, 0 );

                _run_forever( m );
            }

            else if ( distance > 0 && 0 == duration) {

                int time = (int)( (970 * distance) / DIST_PER_SEC ); // 970 and not 100 seems better and more precise
//                printf("time=%d\n", time);
                _set_command_attr( *cmd, m->pose->speed < 0 ? MOVE_BACKWARD : MOVE_FORWARD, distance, time, 0 );

                _run_timed( m, time );
            }

            else if ( 0 == distance ) {

                int dist_timed = (int) ( ( DIST_PER_SEC * duration ) / 1000.0f );
                _set_command_attr( *cmd, m->pose->speed < 0 ? MOVE_BACKWARD : MOVE_FORWARD, dist_timed, duration, 0 );

                _run_timed( m, duration );
            }

            else
                couldSet = false;

            break;

        case ROTATING:

            //printf( "ANGLE=%d\n", angle );
            _set_command_attr( *cmd, ROTATING, 0, 0, angle );

            _rotate( m, angle );

            break;

        case STOPPED:

            _set_command_attr( *cmd, STOPPED, 0, 0, 0 );

            multi_set_tacho_command_inx( m->motors, TACHO_STOP );

            break;

        default:

            couldSet = false;

            break;
    }

    m->cmd->hasChanged = couldSet;
    pthread_mutex_unlock( &lock_motors );

    return couldSet;
}

void set_speed( WHEELS* w, float speed ) {

    if ( speed <= 0 || speed > 1 ) {

        fprintf( stderr,  "La vitesse doit être comprise entre 0 et 100%%" );
        return;
    }

    int real_speed = ( int )( get_max_speed( w ) * speed );

    _set_speed( w, real_speed );
}

static void _set_speed( WHEELS* m, int speed ) {

    set_tacho_speed_sp( m->motors[L], speed );
    set_tacho_speed_sp( m->motors[R], speed );

    m->pose->speed = speed;
}

bool is_moving( WHEELS* w ) {

    FLAGS_T flag;

    get_tacho_state_flags( w->motors[L], &flag );
    if ( flag != TACHO_STATE__NONE_ ) return true;

    get_tacho_state_flags( w->motors[R], &flag );
    if ( flag != TACHO_STATE__NONE_ ) return true;

    return false;
}

void run_forever( WHEELS* w ) {

    _set_command( w, MOVE_FORWARD, 0, 0, 0 );
}

static void _run_forever( WHEELS* m ) {

    multi_set_tacho_command_inx( m->motors, TACHO_RUN_FOREVER );
}

void run_timed( WHEELS* w, int ms ) {

    _set_command( w, MOVE_FORWARD, 0, ms, 0 );
}

static void _run_timed( WHEELS* m, int ms ) {

    multi_set_tacho_time_sp( m->motors, ms );
    multi_set_tacho_command_inx( m->motors, TACHO_RUN_TIMED );
}

void travel( WHEELS* w, float distance ) {

    _set_command( w, MOVE_FORWARD, distance, 0, 0 );
}

void rotate_to( WHEELS* w, int angle ) {

    if ( ( angle % 360 == 0 ) ) {
        if ( 0 == w->pose->v )
            return;
        angle = 360;
    }

    int diff;

    if ( is_moving( w ) ) return;

    diff = ( ( angle - w->pose->v + 180 ) % 360 ) - 180;
    diff = diff < -180 ? 360 + diff : diff;

//    printf("v=%d,a=%d,diff=%d\n", w->pose->v, angle, diff);

    if ( 0 == diff ) return;

    _set_command(w, ROTATING, 0, 0, diff);
}

void rotate( WHEELS* w, int angle ) {

    _set_command(w, ROTATING, 0, 0, angle);
}

static void _rotate( WHEELS* m, int angle ) {

    int angular_speed = get_max_speed( m );

    int real_angle;

    if (abs(angle) > 0 && abs( angle ) <= 45)
        real_angle = angle > 0 ? MID_LEFT_TURN : MID_RIGHT_TURN;

    else if (abs( angle ) == 90 )
        real_angle = angle > 0 ? LEFT_TURN : RIGHT_TURN;

    else if (abs( angle ) == 180 )
        real_angle = angle > 0 ? HALF_TURN : -HALF_TURN;

    else if ( abs( angle ) == 360 || angle == 0 )
        real_angle = angle > 0 ? FULL_TURN : -FULL_TURN;

    else
        real_angle = angle * 2 + 10 * angle / 90;
    angular_speed /= ( real_angle == abs(MID_LEFT_TURN) ? 8 : 6 );


    int old_speed = m->pose->speed;
    _set_speed( m, angular_speed );


    set_tacho_position_sp( m->motors[L], real_angle );
    set_tacho_position_sp( m->motors[R], -real_angle );

    multi_set_tacho_command_inx( m->motors, TACHO_RUN_TO_REL_POS );

    _set_speed( m, old_speed );
}

int get_max_speed( WHEELS* w ) {

    int s;
    get_tacho_max_speed( w->motors[L], &s );

    return s;
}

void stop_wheels( WHEELS *w ) {

    _set_command( w, STOPPED, 0, 0, 0 );
    usleep( 500 * 1000 );
}

void follow_path( WHEELS* w, PATHFINDER* pf ) {

    int32_t steps = (int32_t) astar_get_directions( pf->pathfinder, &pf->directions );
    direction_t* dir = pf->directions;
    int32_t path[ 32 ][ 2 ];
    memset( path, -99, sizeof( path[ 0 ][ 0 ] ) * 32 * 2 );


    for ( int32_t i = 0 ; i < steps ; i++, dir++ ) {

        path[ i ][ 0 ] = astar_get_dx( pf->pathfinder, *dir );
        path[ i ][ 1 ] = astar_get_dy( pf->pathfinder, *dir );
    }

    int32_t  x0, y0, x1, y1;
    bool mid_turn;
    float distance_to_travel;



    printf("Nb steps: %d, pf->x=%d, pf->y=%d\n\n", steps, pf->x0, pf->y0);
    for ( int32_t i = 0 ; w->cmd->command != STOPPED &&  i < steps - 1 ; i ++ ) {


        x0 = x1 = path[ i ][ 0 ];
        y0 = y1 = path[ i ][ 1 ];
        mid_turn = x0 != 0 && y0 != 0;
        distance_to_travel = mid_turn ? 14.14f : 10;

        while ( x0 == x1 && y0 == y1 ) {

            if ( i >= steps - 1 ) break;

            x1 = path[ i + 1 ][ 0 ];
            y1 = path[ i + 1 ][ 1 ];

            distance_to_travel += mid_turn ? 14.14f : 10;
            i++;
        }


        if ( x0 == -1 && y0 == - 1 ) //NW
            rotate_to( w, -45 );

        else if ( x0 == -1 && y0 == 1 ) //SW
            rotate_to( w, 225 );

        else if ( x0 == 1 && y0 == - 1 ) //NE
            rotate_to( w, 45 );

        else if ( x0 == 1 && y0 == 1 ) //SE
            rotate_to( w, 135 );

        else if ( x0 == 0 && y0 == 1 ) //S
            rotate_to( w, 180 );

        else if ( x0 == 0 && y0 == -1 ) //N
            rotate_to( w, 0 );

        else if ( x0 == -1 && y0 == 0 ) //W
            rotate_to( w, 270 );

        else if ( x0 == 1 && y0 == 0 ) //E
            rotate_to( w, 90 );
        else
            break;

        travel( w, distance_to_travel );
        //printf("Distance to travel: %f\n", distance_to_travel );
        //sched_yield();
    }
}

void go_to( WHEELS* w, PATHFINDER* pf, float x, float y ) {

    uint32_t i = (uint32_t) pose_to_w_index( w->pose->x );
    uint32_t j = (uint32_t) pose_to_h_index( w->pose->y );
    uint32_t x1 = (uint32_t) pose_to_w_index( x );
    uint32_t y1 = (uint32_t) pose_to_h_index( y );

    pf->x0 = i;
    pf->y0 = j;

    _set_command_attr( w->cmd, AUTONAV, 0, 0, 0 );
    //w->cmd->hasChanged = true;

    calculate_path_to( x1, y1 );
    w->pose->autonav = true;
    follow_path( w, pf );
    w->pose->autonav = false;
}

int nearest_puck( WHEELS* w ) {

    int index = 0;
    //puck_t nearest = pucks[ 0 ];

    float distance, smallest = 9999;
    for ( int i = 0 ; i < NB_PUCKS ; i++ ) {

        if ( pucks[ i ].captured )
            continue;

        distance = sqrtf( powf( w->pose->x - pucks[ i ].x, 2 ) + powf( w->pose->y - pucks[ i ].y, 2 ) );

        if ( distance < smallest ) {

            smallest = distance;
            index = i;
        }

    }

    return index;
}