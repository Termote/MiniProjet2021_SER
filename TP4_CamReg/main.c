#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "chprintf.h"
#include "hal.h"
#include "shell.h"

#include "audio/audio_thread.h"
#include "audio/play_melody.h"
#include "audio/play_sound_file.h"
#include "camera/po8030.h"
//#include "sensors/battery_level.h"
#include "sensors/proximity.h"
//#include "cmd.h"
//#include "config_flash_storage.h"
#include "exti.h"
#include "leds.h"
#include "main.h"
#include "memory_protection.h"
#include "motors.h"
//#include "sdio.h"
#include "selector.h"
#include "spi_comm.h"
#include "usbcfg.h"
#include "communication.h"

#include <obstacle_avoid.h>
#include <pi_regulator.h>
#include <process_image.h>
#include <control_lights.h>
#include <movement_control.h>

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

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

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    /* Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    //starts the serial communication
    serial_start();
    //start the USB communication
    usb_start();
    //starts the camera
    dcmi_start();
	po8030_start();
	// auto white balance disable
	po8030_set_awb(0);
	//inits the motors
	motors_init();
	//start proximity sensors
	proximity_start();
	// start lights
	clear_leds();
	spi_comm_start();
	lights_start();
	// start obstacle avoid + selector position
	movement_init();
	chThdSleepMilliseconds(500);
	// start movement control threads
	movement_control_start();
	//stars the threads for the pi regulator and the processing of the image
	pi_regulator_start();
	process_image_start();

    /* Infinite loop. */
    while (1) {

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
