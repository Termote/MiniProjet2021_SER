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

void SendUint8ToComputer(uint8_t* data, uint16_t size) 
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

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

static THD_FUNCTION(selector_thd, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    uint8_t stop_loop = 0;
    systime_t time;

    messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
    proximity_msg_t prox_values;
    int16_t leftSpeed = 0, rightSpeed = 0;

    while(stop_loop == 0) {	// positionne le robot dans la direction indiquée
    	time = chVTGetSystemTime();

		switch(get_selector()) {
			case 0: 
				movement_init();
				stop_loop = 1;
				break;

			case 1: 
				movement_init();
				stop_loop = 1;
				break;

			case 2: 
				movement_init();
				stop_loop = 1;
				break;

			case 3: 
				movement_init();
				stop_loop = 1;
				break;

			case 4: 
				movement_init();
				stop_loop = 1;
				break;

			case 5: 
				movement_init();
				stop_loop = 1;
				break;

			case 6: 
				movement_init();
				stop_loop = 1;
				break;

			case 7:
				movement_init();
				stop_loop = 1;
				break;

			case 8: 
				movement_init();
				stop_loop = 1;
				break;
			case 9: 
				movement_init();
				stop_loop = 1;
				break;

			case 10: 
				movement_init();
				stop_loop = 1;
				break;

			case 11:
				movement_init();
				stop_loop = 1;
				break;

	            

			case 12: 
				movement_init();
				stop_loop = 1;
				break;

			case 13:
				movement_init();
				stop_loop = 1;
				break;

			case 14: 
				movement_init();
				stop_loop = 1;
				break;

			case 15:
				movement_init();
				stop_loop = 1;
				break;
		}
    }
}


int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication
    serial_start();
    //start the USB communication
    usb_start();
    //starts the camera
    dcmi_start();
	po8030_start();
	//inits the motors
	motors_init();

	//stars the threads for the pi regulator and the processing of the image
	pi_regulator_start();
	process_image_start();

    /* Infinite loop. */
    while (1) {
    	//waits 1 second
		chprintf((BaseSequentialStream*)&SD6, "This is some message with a value: %d\r\n", 42);
        chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
