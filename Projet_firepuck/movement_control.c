#include "ch.h"
#include <stdlib.h>
#include <obstacle_avoid.h>
#include "pi_regulator.h"
#include "process_image.h"
#include "sensors/proximity.h"
#include "motors.h"
#include <control_lights.h>


/**************************** SEMAPHORES *************************************/
static BSEMAPHORE_DECL(no_obstacle_sem, TRUE);
static BSEMAPHORE_DECL(goal_not_reached_sem, FALSE); //FALSE so MovementControl can start first

/************************ INTERNAL FUNCTIONS *********************************/

uint8_t obstacle_on_front(void){

    if(get_prox(FRONT_FRONT_RIGHT_SENSOR) > DETECTION_DISTANCE
    || get_prox(FRONT_FRONT_LEFT_SENSOR) > DETECTION_DISTANCE
    || get_prox(FRONT_LEFT_SENSOR) > DETECTION_DISTANCE
    || get_prox(FRONT_LEFT_SENSOR) > DETECTION_DISTANCE){
		return TRUE;
	}
    return FALSE;
}
/********************** END INTERNAL FUNCTIONS *******************************/
/******************************* THREADS *************************************/

static THD_WORKING_AREA(waMovementControl, 128);
static THD_FUNCTION(MovementControl, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1) {

        //waits until the PiRegulator has set up the motors
        chBSemWait(&goal_not_reached_sem);

		if(obstacle_on_front()){
            avoid_obstacle();
		}
		chBSemSignal(&no_obstacle_sem);
    }
}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;
    int16_t speed_correction = 0;

    right_motor_set_speed(MOTOR_SPEED_LIMIT/2);
    left_motor_set_speed(MOTOR_SPEED_LIMIT/2);

    chThdSleepMilliseconds(500);   // wait until the rotation is completed before going into the pi control speed

    while(1){

        time = chVTGetSystemTime();
        //waits until the object detection has passed
    	chBSemWait(&no_obstacle_sem);

        //computes the speed to give to the motors
        //distance_cm is modified by the image processing thread
        speed = pi_regulator(get_distance_cm(), GOAL_DISTANCE);

        //computes a correction factor to let the robot rotate in order to be in front of the line
        speed_correction = (get_line_position() - (IMAGE_BUFFER_SIZE/2));

        //if the line is nearly in front of the camera, don't rotate
        if(abs(speed_correction) < ROTATION_THRESHOLD) {
        	speed_correction = 0;
        }

        //applies the speed from the PI regulator and the correction for the rotation
		right_motor_set_speed(speed/2 - ROTATION_COEFF * speed_correction);
		left_motor_set_speed(speed/2 + ROTATION_COEFF * speed_correction);


		if ((speed == 0) ) {  // when line is found, stop the motors and start light choreography
            right_motor_set_speed(0);
            left_motor_set_speed(0);
            start_light_choreography();
		}
		else {
            //signals that the goal has not been reached and that the object detection can start again
			chBSemSignal(&goal_not_reached_sem); 
		}
        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

/***************************** END THREADS ***********************************/
/************************* PUBLIC FUNCTIONS **********************************/

void pi_regulator_start(void){

	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}

void movement_control_start(void){

    //has a higher priority compared to the ProcessImage thread to avoid hitting an obstacle, wich is more important
	chThdCreateStatic(waMovementControl, sizeof(waMovementControl), NORMALPRIO+1, MovementControl, NULL);
}
/*********************** END PUBLIC FUNCTIONS ********************************/