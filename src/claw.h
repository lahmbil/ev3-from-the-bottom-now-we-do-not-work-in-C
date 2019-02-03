/**
 * @file claw.h
 * @author Leander Feppon and Bilal Lahmami
 * @date 10 Dec 2018
 * @brief Manages the movements of the claw associated to a medium EV3 motor
 */

#ifndef SWAGBOY_CLAW_H
#define SWAGBOY_CLAW_H

#include "ev3_tacho.h"

/**
 * A structure for a quick acces to a claw's properties
 */
typedef struct {

    uint8_t motor;
    bool closed; /**< true if closed*/
    int speed;
} CLAW;

/**
 * @brief Searches if a medium motor is plugged in and try to allocate enough memory for CLAW structure.
 * Its speed is set to 60%.
 * @param port a number to a medium EV3 motor
 * @return A pointer to a CLAW structure
 * @see https://github.com/in4lio/ev3dev-c/blob/master/source/ev3/ev3_port.h
 */
CLAW* init_claw( uint8_t port );

/**
 * @brief Frees the memory taken by a pointer to a CLAW structure
 * @param c
 */
void uninit_claw( CLAW** c );

/**
 * @brief Opens the claw if it is not already is
 * @param c
 * @param force Should only be set to true if the program exited without having closed the claw
 */
void open_claw( CLAW *c, bool force );

/**
 * @brief Closes the claw if it is not already is
 * @param c
 * @param force Should only be set to true if the program exited without having closed the claw
 */
void close_claw( CLAW* c, bool force );

/**
 * @brief Indicates if the claw is still in movement
 * @param c
 * @return true if the claw is still in movement
 */
bool is_claw_moving( CLAW *c );


#endif //SWAGBOY_CLAW_H
