#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <leds.h>
#include <control_lights.h>

#define CHOREOGRAPHY_LENGTH     5000 //ms
#define LED_INTENSITY           255

//semaphore
static BSEMAPHORE_DECL(lights_start_sem, TRUE);

void start_light_choreography(void){
    //signals to start light choreography
    chBSemSignal(&lights_start_sem);
}

static THD_WORKING_AREA(waLights, 128);
static THD_FUNCTION(ThdLights, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    while(1){
    	//waits until the start signal
        chBSemWait(&lights_start_sem);

        time = chVTGetSystemTime();

        toggle_rgb_led(LED2, BLUE_LED ,LED_INTENSITY);
        chThdSleepMilliseconds(50);
        toggle_rgb_led(LED4, BLUE_LED ,LED_INTENSITY);

        while (chVTGetSystemTime() < time + MS2ST(CHOREOGRAPHY_LENGTH)){

            chThdSleepMilliseconds(50);
            toggle_rgb_led(LED6, BLUE_LED ,LED_INTENSITY);
            toggle_rgb_led(LED2, BLUE_LED ,LED_INTENSITY);
            chThdSleepMilliseconds(50);
            toggle_rgb_led(LED8, BLUE_LED ,LED_INTENSITY);
            toggle_rgb_led(LED4, BLUE_LED ,LED_INTENSITY);
            chThdSleepMilliseconds(50);
            toggle_rgb_led(LED2, BLUE_LED ,LED_INTENSITY);
            toggle_rgb_led(LED6, BLUE_LED ,LED_INTENSITY);
            chThdSleepMilliseconds(50);
            toggle_rgb_led(LED4, BLUE_LED ,LED_INTENSITY);
            toggle_rgb_led(LED8, BLUE_LED ,LED_INTENSITY);
        }
        chThdSleepMilliseconds(50);
        toggle_rgb_led(LED2, BLUE_LED ,LED_INTENSITY);
        void clear_leds(void);

        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void lights_start(void){
	chThdCreateStatic(waLights, sizeof(waLights), NORMALPRIO, ThdLights, NULL);
}