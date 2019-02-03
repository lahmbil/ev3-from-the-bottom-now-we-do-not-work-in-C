/**
 * @file claw.c
 * @author Leander Feppon and Bilal Lahmami
 * @date 10 Dec 2018
 * @brief Manages the movements of the claw associated to a medium EV3 motor
 */

#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <sched.h>
#include <string.h>

#include "ev3.h"
#include "ev3_port.h"
#include "claw.h"


#define DEFAULT_TIME 1500 // ms


static int _get_max_speed( CLAW c );

CLAW* init_claw( uint8_t port ) {

    if ( ev3_tacho_init() < 1 ) exit( EXIT_FAILURE );

    CLAW* c = malloc( sizeof( CLAW ) );

    errno = 0;

    if ( NULL == c )
        fprintf( stderr,  "Impossible d'allouer l'espace mémoire nécessaire pour la pince: %s\n", strerror( errno ) ),
        exit(EXIT_FAILURE);

    if ( false == ev3_search_tacho_plugged_in( port, EXT_PORT__NONE_, &c->motor, 0 ) )
        fprintf( stderr,  "Impossible de trouver la pince: %s\n", strerror( errno ) ),
        free( c ),
        exit( EXIT_FAILURE );

    c->speed = (int)( _get_max_speed( *c ) * 0.6f );
    set_tacho_speed_sp( c->motor, c->speed );

    c->closed = true;

    return c;
}

void uninit_claw( CLAW** c ) {

   free( *c );
   c = NULL;
}

static int _get_max_speed( CLAW c ) {

    int s;
    get_tacho_max_speed( c.motor, &s );

    return s;
}

static void _run( CLAW* c, bool forward, bool force ) {

    if ( !force && ( forward == c->closed ) )
        return;

    while ( is_claw_moving( c ) )
        sched_yield();

    set_tacho_command_inx( c->motor, TACHO_STOP );


    if ( (forward && c->speed > 0) || (!forward && c->speed < 0) ) {

        c->speed *= -1;
        set_tacho_speed_sp( c->motor, c->speed );
    }

    set_tacho_time_sp(c->motor, DEFAULT_TIME);
    set_tacho_command_inx( c->motor, TACHO_RUN_TIMED );

    c->closed = forward;
}

void open_claw( CLAW *c, bool force ) {

    _run( c, false, force );
}


void close_claw( CLAW* c, bool force ) {

    _run( c, true, force );
}

bool is_claw_moving(CLAW *c) {

    FLAGS_T flag;

    get_tacho_state_flags( c->motor, &flag );

    return flag != TACHO_STATE__NONE_;
}
