#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <obstacle_avoid.h>

/////////////////////////////////////////////////////////////
static BSEMAPHORE_DECL(no_obstacle_sem, TRUE);
static BSEMAPHORE_DECL(goal_not_reached_sem, TRUE);

uint8_t obstacle_on_front(void){

    if(get_prox(FRONT_FRONT_RIGHT_SENSOR) > DETECTION_DISTANCE 
    || get_prox(FRONT_FRONT_LEFT_SENSOR) > DETECTION_DISTANCE
    || get_prox(FRONT_LEFT_SENSOR) > DETECTION_DISTANCE
    || get_prox(FRONT_LEFT_SENSOR) > DETECTION_DISTANCE){

		return TRUE;
	}
    return FALSE;
}

static THD_WORKING_AREA(waMovementControl, 256);
static THD_FUNCTION(MovementControl, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1){

        chBSemWait(&goal_not_reached_sem);
		if(obstacle_on_front()){
            avoid_obstacle();
		}
		chBSemSignal(&no_obstacle_sem);
    }
}

void movement_control_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO+1, MovementControl, NULL);
}

static THD_WORKING_AREA(waTestCamera, 256);
static THD_FUNCTION(TestCamera, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1){
		chBSemWait(&no_obstacle_sem);
		chprintf((BaseSequentialStream *)&SD3, "Camera ON\r\n");

        chBSemSignal(&goal_not_reached_sem);
    }
}

void test_camra_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, TestCamera, NULL);
}
