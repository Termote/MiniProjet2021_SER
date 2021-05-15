#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <leds.h>
#include "audio/audio_thread.h"
#include "audio/play_melody.h"
#include "audio/play_sound_file.h"
#include <control_lights.h>


static BSEMAPHORE_DECL(goal_reached_sem, TRUE);			// semaphore to signal when the robot is correctly arrived
static BSEMAPHORE_DECL(sirens_on_sem, FALSE);			// semaphore for light blinking when traveling to goal

uint8_t goal_reached = FALSE;

void start_light_choreography(void){
    goal_reached = TRUE;
    chBSemSignal(&goal_reached_sem);
}

/* Thread for light blinking when advancing */

static THD_WORKING_AREA(waLightsSirens, 256);
static THD_FUNCTION(LightsSirens, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1) {

        chBSemWait(&sirens_on_sem);

        set_led(LED1, LED_INTENSITY);
        set_led(LED3, LED_INTENSITY);
        set_led(LED7, LED_INTENSITY);

        toggle_rgb_led(LED2, BLUE_LED ,LED_INTENSITY);

        while (goal_reached == FALSE) {
            chThdSleepMilliseconds(LIGHT_FLICKER_TIME);
            toggle_rgb_led(LED2, BLUE_LED ,LED_INTENSITY);
            toggle_rgb_led(LED8, BLUE_LED ,LED_INTENSITY);
        }
        clear_leds();
    }
}

/* Thread for light choreography when  motors are stopped (when goal is reached) */

static THD_WORKING_AREA(waLightsCircling, 256);
static THD_FUNCTION(LightsCircling, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    while(1) {

        chBSemWait(&goal_reached_sem);			// signal that the goal is reached
        chThdSleepMilliseconds(20);
        clear_leds();
        time = chVTGetSystemTime();

        while (chVTGetSystemTime() < time + MS2ST(CHOREOGRAPHY_LENGTH)) {

            chThdSleepMilliseconds(LIGHT_FLICKER_TIME);
            toggle_rgb_led(LED4, BLUE_LED ,LED_INTENSITY);
            chThdSleepMilliseconds(LIGHT_FLICKER_TIME);
            toggle_rgb_led(LED6, BLUE_LED ,LED_INTENSITY);
            chThdSleepMilliseconds(LIGHT_FLICKER_TIME);
            toggle_rgb_led(LED8, BLUE_LED ,LED_INTENSITY);
            chThdSleepMilliseconds(LIGHT_FLICKER_TIME);
            toggle_rgb_led(LED2, BLUE_LED ,LED_INTENSITY);
        }

        clear_leds();
        goal_reached = FALSE;
        chBSemSignal(&sirens_on_sem);
    }
}

/* Fonction initializing threads for lights control */

void lights_start(void) {
    clear_leds();
    chThdCreateStatic(waLightsSirens, sizeof(waLightsSirens), HIGHPRIO, LightsSirens, NULL);
	chThdCreateStatic(waLightsCircling, sizeof(waLightsCircling), HIGHPRIO, LightsCircling, NULL);
}
