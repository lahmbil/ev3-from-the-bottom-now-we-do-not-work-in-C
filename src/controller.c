/**
 * @file controller.c
 * @author Leander Feppon and Bilal Lahmami
 * @date 10 Dec 2018
 * @brief Manages the puck grabber's logic and decisions
 */

#include <unistd.h>
#include <math.h>

#include "ev3.h"
#include "ev3_port.h"

#include "wheels.h"
#include "claw.h"
#include "sensors.h"
#include "pathfinding.h"

#include "controller.h"

#include "coroutine.h"


#define SLEEP( msec ) usleep( ( msec ) * 1000 )

#define LEFT_WHEEL_PORT     OUTPUT_A
#define RIGHT_WHEEL_PORT    OUTPUT_B
#define CLAW_PORT           OUTPUT_D

int coeff_reverted;
bool alive = false;
bool timeout = false;
bool holding_puck = false;

int color;
int distance;
bool pressed;

WHEELS* w;
CLAW* c;

CORO_CONTEXT( COLOR_SENSOR );
CORO_CONTEXT( SONIC_SENSOR );
CORO_CONTEXT( TOUCH_SENSOR );
CORO_CONTEXT( CHECK_KEYS );

CORO_DEFINE( COLOR_SENSOR ) {


    CORO_BEGIN();

    for ( ; ; ) {

        color = get_color();

        if ( YELLOW == color )
            w->pose->x = -50 * coeff_reverted;

        else if ( RED == color )
            w->pose->x = 50 * coeff_reverted;

        else if ( BLUE == color )
            w->pose->y = coeff_reverted == 1 ? 60 : 180;

        else if ( GREEN == color )
            w->pose->y = coeff_reverted == 1 ? 180 : 60;

        else if ( WHITE == color )
            w->pose->y = 240;

        CORO_YIELD();
    }

    CORO_END();
}


CORO_DEFINE( SONIC_SENSOR ) {

    CORO_BEGIN();

    for ( ; ; ) {

        distance = get_distance();

        CORO_YIELD();
    }

    CORO_END();
}

CORO_DEFINE( TOUCH_SENSOR ) {

    CORO_BEGIN();

    for ( ; ; ) {

        pressed = is_pressed();


        if ( pressed && ! holding_puck) {

            stop_wheels( w );
            holding_puck = true;

            open_claw( c, false );
            while ( is_claw_moving( c ) )
                ;
            close_claw( c, false);

            rotate_to( w, 0 );
            travel ( w, 240 - w->pose->y );

            while ( is_moving( w ) )
                ;

            open_claw( c , false);
            holding_puck = false;

            rotate_to( w, 180 );

            printf("PUCK? %f, %f: %d\n", w->pose->x, w->pose->y, get_map( pose_to_w_index( round(w->pose->x) ), pose_to_h_index( round(w->pose->y )) ) );
            set_map( pose_to_w_index( w->pose->x ), pose_to_h_index( w->pose->y ), 8 );


        }


        CORO_YIELD();
    }

    CORO_END();
}

CORO_DEFINE( CHECK_KEYS ) {

    CORO_LOCAL uint8_t key;

    CORO_BEGIN();

    for( ; ; ) {

        SLEEP(500); // if a key is not yet released, key takes the value of the key still being pressed
        CORO_WAIT( ev3_read_keys( &key ) && key != EV3_KEY__NONE_ );

        if ( key == EV3_KEY_BACK )
            alive = false;

        else
            timeout = true;

        CORO_YIELD();
    }

    CORO_END();
}
void controller() {

    w = init_motors( LEFT_WHEEL_PORT, RIGHT_WHEEL_PORT );
    c = init_claw( CLAW_PORT );
    bool sensorsInit = init_sensors();
    bool pathInit = init_pathfinder();

    alive = ( w != NULL ) && ( c != NULL ) && sensorsInit && pathInit;

    uint8_t key;

    //close_claw( c, true );

    printf("WAITING FOR KEY PRESS\n");
    while ( ev3_read_keys( &key ),  key != EV3_KEY_DOWN && key != EV3_KEY_UP )
        SLEEP( 100 );


    coeff_reverted = key == EV3_KEY_DOWN ? -1 : 1;

    // first blood
    ////////////////////////
    set_speed( w, 0.8 );

    multi_set_tacho_ramp_up_sp( w->motors, 500 );
    multi_set_tacho_position_sp( w->motors,5000 );
    multi_set_tacho_command_inx( w->motors, TACHO_RUN_TO_REL_POS );

    SLEEP( 200 );
    open_claw( c, false );
    while ( is_claw_moving( c ) )
        ;
    close_claw( c, false );

    while ( is_moving( w ) )
        ;

    open_claw( c, false );
    while ( is_claw_moving( c ) )
        ;

    close_claw( c, false );
    multi_set_tacho_command_inx( w->motors, TACHO_STOP );

    set_speed( w, 0.3 );

    rotate_to( w, 180 );
    w->pose->x = -10;
    w->pose->y = 250;
    ////////////////////////

    go_to( w, pf, 0, 180 );

    main:
    if ( alive && ! timeout )
        printf("MAIN\n");
    while ( alive && ! timeout ) {

        CORO_CALL( COLOR_SENSOR );
        CORO_CALL( SONIC_SENSOR );
        CORO_CALL( TOUCH_SENSOR );
        CORO_CALL( CHECK_KEYS );


        for ( int i = 0 ; i < NB_PUCKS ; i++ ) {

            if ( round(w->pose->x) == pucks[ i ].x && round(w->pose->y) == pucks[ i ].y
                && pucks[ i ].captured == false && pressed == false ) {

                printf("PUCK ALREADY CAPTURED: %f, %f\n", pucks[ i ].x, pucks[ i ].y );
                pucks[ i ].captured = true;
            }
        }

        if ( holding_puck &&  w->cmd->command != AUTONAV ) {

            printf("Holding puck? %d\n\n", holding_puck);

            stop_wheels( w );
            go_to( w, pf, w->pose->x, 240 );
            w->cmd->command = AUTONAV;

            while ( is_moving( w ) && get_color() != WHITE )
                ;
            //open_claw( c, false );
            while ( is_claw_moving( c ) )
                ;
            rotate_to( w, 180 );
            holding_puck = false;
        }

        else if ( ! holding_puck && w->cmd->command != AUTONAV ) {

            open_claw( c, false );
            printf("NOT HOLDING ANYTHING\n");
            int i = nearest_puck( w );
            go_to( w, pf, pucks[ i ].x, pucks[ i ].y );
            w->cmd->command = AUTONAV;
            if ( pf->pathfinder->result == 0)
                continue;

            printf("Nearest puck: %d\n", i);
            printf("Going to %f, %f\n", pucks[i].x, pucks[i].y);
            //printf("Waiting for movement to complete\n\n");
        }

        SLEEP( 200 );

    }


    if ( timeout ) {

        printf("TIMEOUT\n");

        SLEEP(500);
        while ( ev3_read_keys( &key ), key != EV3_KEY_RIGHT && key != EV3_KEY_LEFT && key != EV3_KEY_BACK )
            SLEEP( 200 );

        timeout = false;

        if ( key == EV3_KEY_BACK )
            alive = false;

        w->pose->x = 0;
        w->pose->y = 0;
        w->pose->v = 0;

        goto main; // ugly but it works
    }

    printf("END\n");

    if ( c->closed == false )
        close_claw( c, true );

    uninit_motors( &w );
    uninit_claw( &c );
    uninit_sensors();
    uninit_pathfinder();

}