/**
 * @file sensors.c
 * @author Leander Feppon and Bilal Lahmami
 * @date 10 Dec 2018
 * @brief Manages the sensors of the robot: color sensor, ultrasonic sensor, and touch sensor
 * @see http://in4lio.github.io/ev3dev-c/group__ev3__sensor.html
 */


#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include "sensors.h"


const char *colors[] = { "?", "BLACK", "BLUE", "GREEN", "YELLOW", "RED", "WHITE", "BROWN" };

COLOR_SENSOR* color_sensor;
TOUCH_SENSOR* touch_sensor;
US_SENSOR* sonic_sensor;

static bool _init_color_sensor();
static bool _init_touch_sensor();
static bool _init_sonic_sensor();

static bool _init_color_sensor() {

    if ( color_sensor != NULL )
        return false;

    if ( ev3_sensor_init() < 1 )
        fprintf( stderr,  "Impossible d'initialiser les capteurs: %s\n", strerror( errno ) ),
        exit( EXIT_FAILURE );

    color_sensor = malloc( sizeof( COLOR_SENSOR ) );
    if ( NULL == color_sensor )
        fprintf( stderr,  "Impossible d'allouer l'espace mémoire nécessaire pour le capteur de couleur: %s\n", strerror( errno ) ),
        exit( EXIT_FAILURE );

    if ( ! ev3_search_sensor( LEGO_EV3_COLOR, &color_sensor->sn, 0 ) )
        fprintf( stderr,  "Aucun capteur de couleur trouvé: %s\n", strerror( errno ) ),
        free( color_sensor ),
        exit( EXIT_FAILURE );

    set_sensor_mode( color_sensor->sn, "COL-COLOR" );

    return true;
}

static bool _init_touch_sensor() {

    if ( touch_sensor != NULL )
        return false;


    errno = 0;
    if ( ev3_sensor_init() < 1 )
        fprintf( stderr,  "Impossible d'initialiser les capteurs: %s\n", strerror( errno ) ),
                exit( EXIT_FAILURE );

    touch_sensor = malloc( sizeof( TOUCH_SENSOR ) );
    if ( NULL == touch_sensor )
        fprintf( stderr,  "Impossible d'allouer l'espace mémoire nécessaire pour le capteur de touche: %s\n", strerror( errno ) ),
                exit( EXIT_FAILURE );

    if ( ! ev3_search_sensor( LEGO_EV3_TOUCH, &touch_sensor->sn, 0 ) )
        fprintf( stderr,  "Aucun capteur de touche trouvé: %s\n", strerror( errno ) ),
                free( touch_sensor ),
                exit( EXIT_FAILURE );

    set_sensor_mode( touch_sensor->sn, "LEGO_EV3_TOUCH_TOUCH" );

    return true;
}

static bool _init_sonic_sensor() {


    if ( sonic_sensor != NULL )
        return false;

    errno = 0;
    if ( ev3_sensor_init() < 1 )
        fprintf( stderr,  "Impossible d'initialiser les capteurs: %s\n", strerror( errno ) ),
                exit( EXIT_FAILURE );

    sonic_sensor = malloc( sizeof( US_SENSOR ) );

    if ( NULL == sonic_sensor )
        fprintf( stderr,  "Impossible d'allouer l'espace mémoire nécessaire pour le capteur d'ultrasons: %s\n", strerror( errno ) ),
                exit( EXIT_FAILURE );

    if ( ! ev3_search_sensor( LEGO_EV3_US, &sonic_sensor->sn, 0 ) )
        fprintf( stderr,  "Aucun capteur d'ultrasons trouvé: %s\n", strerror( errno ) ),
                free( sonic_sensor ),
                exit( EXIT_FAILURE );

    set_sensor_mode( sonic_sensor->sn, "LEGO_EV3_US_DIST_CM" );

    return true;
}

bool init_sensors() {

    if ( ! _init_color_sensor() ) return false;
    if ( ! _init_touch_sensor() ) return false;
    if ( ! _init_sonic_sensor() ) return false;

    return true;
}

void uninit_sensors() {

    free( color_sensor );
    free( touch_sensor );
    free( sonic_sensor );

    color_sensor = NULL;
    touch_sensor = NULL;
    sonic_sensor = NULL;
}

int get_color() {

    if ( NULL == color_sensor )
        _init_color_sensor();


    get_sensor_value( 0, color_sensor->sn, &color_sensor->color );

    if ( color_sensor->color < 0 || color_sensor->color >= COLOR_COUNT ) {
        //color_sensor->color = 0;
        printf( "UNKN: %d\n\n", color_sensor->color );
        return 0;
    }

    //printf("color_sensor->sn=%d\n", color_sensor->sn);

//        sleep( 2 );


    return color_sensor->color;
}

bool is_pressed() {

    if ( NULL == touch_sensor)
        _init_touch_sensor();

    //printf("touch_sensor->sn=%d\n", touch_sensor->sn);
    get_sensor_value( 0, touch_sensor->sn, &touch_sensor->pressed );

    return (bool)( touch_sensor->pressed) ;
}

int get_distance() {

    if ( NULL == sonic_sensor )
        _init_sonic_sensor();

    //printf("sonic_sensor->sn=%d\n", sonic_sensor->sn);
    get_sensor_value( 0, sonic_sensor->sn, &sonic_sensor->distance );

    return sonic_sensor->distance;
}


