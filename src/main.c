#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#include "ev3.h"
#include "ev3_port.h"
#include "ev3_sensor.h"
#include "ev3_tacho.h"
#include "coroutine.h"

#include "controller.h"


// WIN32 /////////////////////////////////////////
#ifdef __WIN32__

#include <windows.h>

// UNIX //////////////////////////////////////////
#else

#include <unistd.h>
#define SLEEP( msec ) usleep(( msec ) * 1000 )

//////////////////////////////////////////////////
#endif

int main() {

    if ( ev3_init() < 1 ) return EXIT_FAILURE;
    if ( ev3_tacho_init() < 1 ) return EXIT_FAILURE;
    if ( ev3_port_init() < 1 ) return EXIT_FAILURE;
    if ( ev3_sensor_init() < 1 ) return EXIT_FAILURE;


//    mwStatistics( 2 );
//    mwAutoCheck( 1 );
    controller();

    ev3_uninit();

//    mwTerm();
    //SLEEP(5000);

    return EXIT_SUCCESS;

}