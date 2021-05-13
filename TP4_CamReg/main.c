#include <stdio.h>				// enlever les includes inutiles, jai copier coller tout les includes possibles de la librairie
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "chprintf.h"
#include "hal.h"
#include "shell.h"

#include "aseba_vm/aseba_node.h"
#include "aseba_vm/skel_user.h"
#include "aseba_vm/aseba_can_interface.h"
#include "aseba_vm/aseba_bridge.h"
#include "audio/audio_thread.h"
#include "audio/play_melody.h"
#include "audio/play_sound_file.h"
#include "audio/microphone.h"
#include "camera/po8030.h"
#include "epuck1x/Asercom.h"
#include "epuck1x/Asercom2.h"
#include "epuck1x/a_d/advance_ad_scan/e_acc.h"
#include "sensors/battery_level.h"
#include "sensors/imu.h"
#include "sensors/mpu9250.h"
#include "sensors/proximity.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "cmd.h"
#include "config_flash_storage.h"
#include "exti.h"
#include "i2c_bus.h"
#include "ir_remote.h"
#include "leds.h"
#include <main.h>
#include "memory_protection.h"
#include "motors.h"
#include "sdio.h"
#include "selector.h"
#include "spi_comm.h"
#include "usbcfg.h"
#include "communication.h"
#include "uc_usage.h"

#include <movement_control.h>
#include <pi_regulator.h>
#include <process_image.h>
#include <control_lights.h>

#define DELTA 250

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);



static THD_WORKING_AREA(dodge_thd_wa, 2048);


static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

static THD_FUNCTION(dodge_thd, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);
    systime_t time;


    messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
    proximity_msg_t prox_values;
    int16_t leftSpeed = 0, rightSpeed = 0;
    unsigned int left_wall = 0;
    unsigned int right_wall = 0;
    unsigned int left_wall_counter = 0;  // pas utiliser, peut peut etre servir jsp
    unsigned int right_wall_counter = 0; // pas utiliser, peut peut etre servir jsp
    unsigned int pos_counter = 0;        // pas utiliser, peut peut etre servir jsp


    while (1) {
    time = chVTGetSystemTime();
    messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));


	leftSpeed = MOTOR_SPEED_LIMIT - prox_values.delta[0]*2 - prox_values.delta[1];
	rightSpeed = MOTOR_SPEED_LIMIT - prox_values.delta[7]*2 - prox_values.delta[6];
	right_motor_set_speed(rightSpeed);
	left_motor_set_speed(leftSpeed);

	if (prox_values.delta[2] > DELTA ) {
		right_wall = 1;
	}
		else if (prox_values.delta[5] > DELTA ) {
			left_wall = 1;
		}
			else if ((prox_values.delta[2] < DELTA) && (right_wall == 1)) {

				pos_counter += abs(left_motor_get_pos());

			    left_motor_set_pos(0);
			    right_motor_set_pos(0);

			    float temp = 90;

				leftSpeed = MOTOR_SPEED_LIMIT ;
				rightSpeed = -MOTOR_SPEED_LIMIT/2;
				right_motor_set_speed(rightSpeed);
				left_motor_set_speed(leftSpeed);

			    // Turns until the desired angle is reached
			    while (abs(left_motor_get_pos()) < abs((temp/360)* 500) &&
			               abs(right_motor_get_pos()) < abs((temp/360)* 500 )) {
				};

			    right_wall = 0;
			    ++right_wall_counter;

			}
				else if ((prox_values.delta[5] < DELTA) && (left_wall == 1)) {
					left_motor_set_pos(0);
					right_motor_set_pos(0);

					float temp = 90;

					leftSpeed = -MOTOR_SPEED_LIMIT/2;
					rightSpeed = MOTOR_SPEED_LIMIT;
					right_motor_set_speed(rightSpeed);
					left_motor_set_speed(leftSpeed);

					// Turns until the desired angle is reached
					while (abs(left_motor_get_pos()) < abs((temp/360)* 500) &&
							   abs(right_motor_get_pos()) < abs((temp/360)* 500 )) {
					};


					left_wall = 0;
					++left_wall_counter;
				}


	chThdSleepUntilWindowed(time, time + MS2ST(10)); // refresh AT 100 HZ
    }
}
int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    /** Inits the Inter Process Communication bus. */
    	messagebus_init(&bus, &bus_lock, &bus_condvar);

    //starts the serial communication
    serial_start();
    //start the USB communication
    usb_start();
    //starts the camera
    dcmi_start();
	po8030_start();
	//inits the motors
	motors_init();
	//start proximity sensors
	proximity_start();
	// start light
	//lights_start();


	//stars the threads for the pi regulator and the processing of the image
	//pi_regulator_start();
	//process_image_start();

	movement_init();
	test_camra_start();
	movement_control_start();


    /* Infinite loop. */
    while (1) {

    	chprintf((BaseSequentialStream *)&SD3, "infinity   \r\n");

	//waits 1 second
	chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}

/////////////////////////////////////////////////////////////
static BSEMAPHORE_DECL(no_obstacle_sem, TRUE);


static THD_WORKING_AREA(waMovementControl, 256);
static THD_FUNCTION(MovementControl, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1){

		chprintf((BaseSequentialStream *)&SD3, "Front sensor : %d\r\n", get_prox(FRONT_FRONT_RIGHT_SENSOR));
		if(get_prox(FRONT_FRONT_RIGHT_SENSOR) > DETECTION_DISTANCE || get_prox(FRONT_FRONT_LEFT_SENSOR) > DETECTION_DISTANCE){
			chprintf((BaseSequentialStream *)&SD3, "Avoiding Obstacle\r\n");
			chThdSleepMilliseconds(2000);
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
		chThdSleepMilliseconds(200);
    }
}

void test_camra_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, TestCamera, NULL);
}

void movement_control_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO+1, MovementControl, NULL);
}