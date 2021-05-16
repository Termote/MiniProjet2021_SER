#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "motors.h"
#include "movement_control.h"
#include <math.h>
#include "selector.h"
#include "chprintf.h"
#include "sensors/proximity.h"
#include <leds.h>
#include "obstacle_avoid.h"


struct movement{                                    // Struct with all information about the e-puck movement state and general information

    enum {
        NO = 0,
        YES = 1
    } on_right_track;

    uint8_t obstacle_detection[2];                  // 1 if there is an obstacle on : [FRONT,SIDE]
    uint8_t front_sensor;                           // Determines current front sensor
    uint8_t side_sensor;                            // Determines current side sensor
    uint8_t diag_sensor;                            // Determines current diagonal sensor

    enum {
            UNDER=0,
			FORWARD=1,
			DIVERGING=2,
			BACKWARD = 3,
			CONVERGING = 4,
			OVER = 5
        } orientation;

    enum {
        STOPED=0,
        ADVANCING=1,
        TURNING=2
    } state;                                        // Determines current movement state

    enum {
        LEFT=-1,
        CENTER=0,
		RIGHT=1
    } turn_direction, obstacle_avoiding_side;       // Determines current turning direction, Determines general side to get arround the obstacle

} movement_info;

/***************************INTERNAL FUNCTIONS************************************/

void halt(void){                                        // Stops the motors
    movement_info.state = STOPED;
    left_motor_set_speed(0);
    right_motor_set_speed(0);
}

void advance_distance(uint16_t distance){           // Makes the e-puck go forward a certain amount of mm

    go_forward();
    left_motor_set_pos(0);
    right_motor_set_pos(0);

    while (left_motor_get_pos() < (distance/CONVERSION_CM_MM)* NSTEP_ONE_TURN / WHEEL_PERIMETER) {
    }
    halt();
}

void turn_to(int angle){                            // Turns the e-puck a derired angle

    movement_info.state = TURNING;

    left_motor_set_pos(0);
    right_motor_set_pos(0);

    // The e-puck will pivot on itself
    left_motor_set_speed(movement_info.turn_direction*MOTOR_SPEED_LIMIT/2);
    right_motor_set_speed(-movement_info.turn_direction*MOTOR_SPEED_LIMIT/2);
    // Turns until the desired angle is reached
    while ((abs(left_motor_get_pos()) < abs((angle/FULL_PERIMETER_DEG)*NSTEP_ONE_TURN*CORRECTION_FACTOR))
    	&& (abs(right_motor_get_pos()) < abs((angle/FULL_PERIMETER_DEG)*NSTEP_ONE_TURN*CORRECTION_FACTOR))) {
	}
    halt();
}

void update_orientation(void) {                          // Updates orientation from current turning side (-1 or 1)

	movement_info.orientation += movement_info.turn_direction*movement_info.obstacle_avoiding_side;
		if (movement_info.orientation == UNDER) {
			movement_info.orientation = CONVERGING;
		}
			else if (movement_info.orientation == OVER) {
				movement_info.orientation = FORWARD;
			}
}

uint8_t status_on_diag(void) {                          //Returns TRUE if there is an object detected on diagonal

    if (get_prox(movement_info.diag_sensor) > DETECTION_DISTANCE*CLOSE_COEFF
        || get_prox(movement_info.diag_sensor-movement_info.obstacle_avoiding_side) > DETECTION_DISTANCE*CLOSE_COEFF) {
    	return TRUE;
    }
	else {
		return FALSE;
	}
}

uint8_t status_on_front(void){                          //Returns TRUE if there is an object detected on front

    if (get_prox(movement_info.front_sensor) > DETECTION_DISTANCE) {
    	return TRUE;
    }
	else {
		return FALSE;
	}
}

uint8_t status_on_side(void) {                           //Returns TRUE if there is an object detected on object side

    if ((get_prox(movement_info.side_sensor) > DETECTION_DISTANCE - ERROR_TOLERANCE) || 
	    (get_prox(movement_info.diag_sensor) > DETECTION_DISTANCE/FAR_COEFF)) {
    	return TRUE;
    }
	else {
		return FALSE;
	}
}

void find_turning_side (void){                          // Determines witch side to turn to, from proximity sensors

    if((movement_info.obstacle_avoiding_side == RIGHT)) {
    	if (movement_info.obstacle_detection[0] == TRUE) {
    		movement_info.turn_direction = RIGHT;
    	}
    	else {
    		movement_info.turn_direction = LEFT;
    	}
    }

    if((movement_info.obstacle_avoiding_side == LEFT)) {
        	if (movement_info.obstacle_detection[0] == TRUE) {
        		movement_info.turn_direction = LEFT;
        	}
        	else {
        		movement_info.turn_direction = RIGHT;
        	}
        }

    if((movement_info.obstacle_detection[0] == FALSE) && movement_info.orientation == CONVERGING) {
    	movement_info.turn_direction = CENTER;
    		}
}

/* The next function advances forward until a change in proximity sensors is detected,
    or if the e-puck is back on the right track */

void advance_until_interest_point(int32_t *update_distance, int32_t *deviation_distance, int8_t direction_coefficient){

    go_forward();
    left_motor_set_pos(0);
    right_motor_set_pos(0);

    while ((object_detection() == FALSE) && !((*deviation_distance <= left_motor_get_pos()) && (movement_info.orientation == CONVERGING))) {
    	chThdSleepMilliseconds(10);
    }
    halt();

    *update_distance += direction_coefficient*left_motor_get_pos();
    // advance a little bit further in order to avoid the e-puck to hit the wall
    if(movement_info.obstacle_detection[1] == FALSE && movement_info.turn_direction) {
    	advance_distance(EPUCK_RADIUS);
    }

    find_turning_side();
   	turn_to(movement_info.turn_direction*STANDARD_TURN_ANGLE);
    update_orientation();

    if(movement_info.obstacle_detection[1] == FALSE && movement_info.turn_direction) {
    	advance_distance(EPUCK_RADIUS);
    }

}



void init_obstacle_tection(void){                       // Sets obstacles after a turn
    movement_info.obstacle_detection[0] = 0;
    movement_info.obstacle_detection[1] = abs(movement_info.turn_direction); //changes to 0 if advancing without sidewall
}

void set_turning_direction(void) {                      // Finds inicial turning direction, looks for a "shorter" side, or a slope


    if(get_prox(FRONT_LEFT_SENSOR) < get_prox(FRONT_RIGHT_SENSOR)) {
        movement_info.turn_direction = LEFT;
        movement_info.obstacle_avoiding_side = LEFT;
        movement_info.front_sensor = FRONT_FRONT_LEFT_SENSOR;
        movement_info.side_sensor = RIGHT_SENSOR;
        movement_info.diag_sensor = FRONT_RIGHT_SENSOR;
    }
    else {
        movement_info.turn_direction = RIGHT;
        movement_info.obstacle_avoiding_side = RIGHT;
        movement_info.front_sensor = FRONT_FRONT_RIGHT_SENSOR;
        movement_info.side_sensor = LEFT_SENSOR;
        movement_info.diag_sensor = FRONT_LEFT_SENSOR;
    }
}





/*************************END INTERNAL FUNCTIONS**********************************/

/****************************PUBLIC FUNCTIONS*************************************/
void movement_init(void){     //Initiates some values and turns to selected direction.
    movement_info.orientation = 0;

    int selector_angle = 0;

    switch(get_selector()) {
		case 0:

		case 1:

		case 2:

		case 3:

		case 4:
			selector_angle = get_selector()*FULL_PERIMETER_DEG/SELECTOR_MAX + SELECTOR_OFFSET;
			break;
		case 5:

		case 6:

		case 7:

		case 8:

		case 9:

		case 10:

		case 11:

		case 12:

		case 13:

		case 14:

		case 15:
			selector_angle = get_selector()*FULL_PERIMETER_DEG/SELECTOR_MAX - 3*SELECTOR_OFFSET;
			break;
    }

    if(selector_angle < 0){
    	movement_info.turn_direction = LEFT;
    	selector_angle -= FULL_PERIMETER_DEG/SELECTOR_MAX;
    }
    else if (selector_angle > 0) {
    	selector_angle += FULL_PERIMETER_DEG/SELECTOR_MAX;
    	movement_info.turn_direction = RIGHT;
    }
    turn_to(selector_angle);
}

uint8_t object_detection(void){                   //Returns TRUE if there is a change on proximity sensors

	chThdSleepMilliseconds(10); // avoid the instant detection of the starting front wall before the turn is completed

    if(status_on_front() == TRUE || status_on_diag()) {
        movement_info.obstacle_detection[0] = TRUE;
        return TRUE;
    }
    else if(status_on_side() == FALSE && movement_info.turn_direction) {
        movement_info.obstacle_detection[1] = FALSE;
        return TRUE;
    }
    return FALSE;
}

void avoid_obstacle(void){                              // Function used to avoid an obstacle

    int32_t deviation_distance = 0;                 // Distance from main track, in motor steps
    int32_t forward_distance = 0;                   // Distance from initial object detection, in motor steps

    //initialising all components for first turn
    movement_info.on_right_track = NO;
    set_turning_direction();
    init_obstacle_tection();
    turn_to(movement_info.turn_direction*STANDARD_TURN_ANGLE);
    movement_info.orientation = DIVERGING;

    while (movement_info.on_right_track == 0)           //Each loop is a moove forward and turn, until e-puck is back on main path
    {
        init_obstacle_tection();

        switch(movement_info.orientation) {
            case FORWARD:
                advance_until_interest_point(&forward_distance, &deviation_distance, FURTHER);
                break;

            case BACKWARD:
                advance_until_interest_point(&forward_distance, &deviation_distance, CLOSER);
                break;

            case DIVERGING:                         //Diverging from main path
                advance_until_interest_point(&deviation_distance, &deviation_distance, FURTHER);
                break;

            case CONVERGING:                        //Diverging from main path
                advance_until_interest_point(&deviation_distance, &deviation_distance, CLOSER);
                break;
            default:
            	break;
        }

        //If the e-puck is back on the main track and we advanced at least a little bit forward, the the obstacle is avoided
        if(deviation_distance <= 0 && forward_distance > 0) {
        	movement_info.on_right_track = TRUE;
        	if (movement_info.obstacle_avoiding_side == RIGHT) {
        		movement_info.turn_direction = RIGHT;
        	}
        	else {
        		movement_info.turn_direction = LEFT;
        	}
        	turn_to(STANDARD_TURN_ANGLE);
        }
    }
}

void go_forward(void) {                                  // Makes the e-puck go forward indefinetly

	movement_info.state = ADVANCING;
    left_motor_set_speed(MOTOR_SPEED_LIMIT/2);
    right_motor_set_speed(MOTOR_SPEED_LIMIT/2);
}

/**************************END PUBLIC FUNCTIONS***********************************/
