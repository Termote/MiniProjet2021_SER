#include <stdio.h>
#include "sdio.h"
#include "camera/po8030.h"
#include "sensors/proximity.h"
#include "leds.h"
#include "main.h"
#include "memory_protection.h"
#include "motors.h"
#include "selector.h"
#include "spi_comm.h"
#include "usbcfg.h"

#include <obstacle_avoid.h>
#include <pi_regulator.h>
#include <process_image.h>
#include <control_lights.h>
#include <movement_control.h>

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

//starts serial communication
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
	//initialises clock generation
	po8030_start();
	// auto white balance disable
	po8030_set_awb(0);
	//inits the motors
	motors_init();
	//start proximity sensors
	proximity_start();
	// start lights
	spi_comm_start();
	clear_leds();
	// start light threads : LightsSirens and LightsCircle
	lights_start();
	// start obstacle avoid + selector position
	movement_init();

	/* Waits untill the e-puck has started the initial turn 
	and that the movement_init had tine to initialise
	*/
	chThdSleepMilliseconds(500);

	// start movement control thread 
	movement_control_start();
	//stars the threads for the pi regulator and the processing of the image
	pi_regulator_start();
	process_image_start();

    /* Infinite loop. */
    while (1) {
	//puts main to sleep for 100 milisecond
	chThdSleepMilliseconds(100);
    }
}

/* memory safety measure */
#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
