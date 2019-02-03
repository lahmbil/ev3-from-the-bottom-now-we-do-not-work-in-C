/**
 * @file sensors.h
 * @author Leander Feppon and Bilal Lahmami
 * @date 10 Dec 2018
 * @brief Manages the sensors of the robot: color sensor, ultrasonic sensor, and touch sensor
 * @see http://in4lio.github.io/ev3dev-c/group__ev3__sensor.html
 */

#ifndef SWAGBOY_SENSORS_H
#define SWAGBOY_SENSORS_H

#include "ev3.h"
#include "ev3_sensor.h"

/**
 * Used to return a string to which color was found
 */
extern const char* colors[];
#define COLOR_COUNT  (( int )( sizeof( colors ) / sizeof( colors[ 0 ])))

enum Colors{

    UNKNOWN = 0,
    BLACK,
    BLUE,
    GREEN,
    YELLOW,
    RED,
    WHITE,
    BROWN
};

typedef struct {

    uint8_t sn;
    int color;
} COLOR_SENSOR;

typedef struct {

    uint8_t sn;
    int pressed;
} TOUCH_SENSOR;

typedef struct {

    uint8_t sn;
    int distance;
} US_SENSOR;

extern COLOR_SENSOR* color_sensor;
extern TOUCH_SENSOR* touch_sensor;
extern US_SENSOR* sonic_sensor;

/**
 * @brief Tries to find an ultrasonic sensor, a touch sensor, and a color sensor and to initialize them.
 * Tries to allocate enough memory for sonic_sensor_, touch_sensor, and color_sensor
 * @return
 */
bool init_sensors();

/**
 * @brief Frees the memory taken by sonic_sensor, touch_sensor, and color_sensor
 */
void uninit_sensors();

/**
 * @return the color detected by the color sensor
 * @see Colors
 */
int get_color();

/**
 *
 * @return true if the touch button is pressed
 */
bool is_pressed();

/**
 *
 * @return the distance measured by the ultrasonic sensor. A distance of 255 should be considered as a distance which
 * could not have been calculated
 */
int get_distance();

#endif //SWAGBOY_SENSORS_H
