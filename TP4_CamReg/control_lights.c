#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <leds.h>
#include <control_lights.h>

#define CHOREOGRAPHY_LENGTH     5000 //ms
#define LED_INTENSITY           100

void start_light_choreography(void){
        systime_t time = chVTGetSystemTime();

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
}