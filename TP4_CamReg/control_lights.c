#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <leds.h>
#include <control_lights.h>

#define CHOREOGRAPHY_LENGTH     5000 //ms
#define LED_INTENSITY           100
#define LIGHT_FLICKER_TIME      300  //ms

static BSEMAPHORE_DECL(goal_reached_sem, FALSE);
static BSEMAPHORE_DECL(sirens_on_sem, TRUE);

uint8_t goal_reached = FALSE;

void start_light_choreography(void){
    goal_reached = TRUE;
    chBSemSignal(&goal_reached_sem);
}

static THD_WORKING_AREA(waLightsSirens, 256);
static THD_FUNCTION(LightsSirens, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1){

        chBSemWait(&sirens_on_sem);
        set_led(LED1, LED_INTENSITY);
        set_led(LED3, LED_INTENSITY);
        set_led(LED5, LED_INTENSITY);
        set_led(LED7, LED_INTENSITY);
        
        toggle_rgb_led(LED2, BLUE_LED ,LED_INTENSITY);

        while (goal_reached = FALSE)){

            chThdSleepMilliseconds(LIGHT_FLICKER_TIME);
            toggle_rgb_led(LED2, BLUE_LED ,LED_INTENSITY);
            toggle_rgb_led(LED8, BLUE_LED ,LED_INTENSITY);
        }
    }
}

static THD_WORKING_AREA(waLightsCircling, 256);
static THD_FUNCTION(LightsCircling, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time = chVTGetSystemTime();

    while(1){

        chBSemWait(&goal_reached_sem)
        clear_leds();

        toggle_rgb_led(LED2, BLUE_LED ,LED_INTENSITY);
        chThdSleepMilliseconds(LIGHT_FLICKER_TIME);
        toggle_rgb_led(LED4, BLUE_LED ,LED_INTENSITY);

        while (chVTGetSystemTime() < time + MS2ST(CHOREOGRAPHY_LENGTH)){

            chThdSleepMilliseconds(LIGHT_FLICKER_TIME);
            toggle_rgb_led(LED6, BLUE_LED ,LED_INTENSITY);
            toggle_rgb_led(LED2, BLUE_LED ,LED_INTENSITY);
            chThdSleepMilliseconds(LIGHT_FLICKER_TIME);
            toggle_rgb_led(LED8, BLUE_LED ,LED_INTENSITY);
            toggle_rgb_led(LED4, BLUE_LED ,LED_INTENSITY);
            chThdSleepMilliseconds(LIGHT_FLICKER_TIME);
            toggle_rgb_led(LED2, BLUE_LED ,LED_INTENSITY);
            toggle_rgb_led(LED6, BLUE_LED ,LED_INTENSITY);
            chThdSleepMilliseconds(LIGHT_FLICKER_TIME);
            toggle_rgb_led(LED4, BLUE_LED ,LED_INTENSITY);
            toggle_rgb_led(LED8, BLUE_LED ,LED_INTENSITY);
        }
        clear_leds();
        goal_reached = FALSE;
        chBSemSignal(&sirens_on_sem);
    }
}

void lights_start(void){
    spi_comm_start();
	chThdCreateStatic(waLightsCircling, sizeof(waLightsCircling), LOWPRIO, LightsCircling, NULL);
    chThdCreateStatic(waLightsSirens, sizeof(waLightsSirens), LOWPRIO, LightsSirens, NULL);
}
