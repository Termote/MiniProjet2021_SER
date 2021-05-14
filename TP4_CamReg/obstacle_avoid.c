#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "motors.h"
#include "movement_control.h"
#include <math.h>
#include "selector.h"
#include "chprintf.h"
#include "sensors/proximity.h"
#include <leds.h>



#define NSTEP_ONE_TURN              1000    // number of step for 1 turn of the motor
#define CORRECTION_FACTOR           1.05    // correct the angle of rotation to be more precise
#define WHEEL_PERIMETER             13      // cm
#define WHEEL_DISTANCE              5.35f   //cm
#define PERIMETER_EPUCK             (3.14 * WHEEL_DISTANCE)     //PI * Wheel distance
#define EPUCK_RADIUS                40      //mm
#define SELECTOR_OFFSET 	        90     //angle offset due to position of 0 on the selector wheel compared to the front
#define SELECTOR_MAX		        16      //maximum value for the selector
#define FULL_PERIMETER_DEG          360     //Degrees needed for the full perimeter
#define FRONT_FRONT_RIGHT_SENSOR    0       // Sensor number for front sensor slightly to the right
#define FRONT_FRONT_LEFT_SENSOR     7       // Sensor number for front sensor slightly to the left
#define FRONT_RIGHT_SENSOR          1       // Sensor number for right sensor at 45Â°
#define FRONT_LEFT_SENSOR           6       // Sensor number for left sensor at -45Â°
#define RIGHT_SENSOR                2       // Sensor number for right sensor at 90Â°
#define LEFT_SENSOR                 5       // Sensor number for left sensor at -90Â°

#define CONVERSION_CM_MM            10      // Decimal differrence between cm and mm
#define STANDARD_TURN_ANGLE         110      // Standard turning angle

#define CLOSER                      -1      // Multiplier used for distance calculation when getting closer to the objective
#define FURTHER                     1       // Multiplier used for distance calculation when getting closer to the objective

#define TRUE                        1       
#define FALSE                       0     

#define DETECTION_DISTANCE          300    // Desired sensor distance detection, in delta
#define ERROR_TOLERANCE             150      // Distance error range,  in delta

/***************************GLOBAL VARIABLES************************************/


struct movement{                                    // Struct with all information about the e-puck movement state and general information

    enum {
        NO = 0,
        YES = 1 
    } on_right_track;                               

    uint8_t obstacle_detection[2];                  // 1 if there is an obstacle on : [FRONT,SIDE]
    uint8_t front_sensor;                           // Determines current front sensor
    uint8_t side_sensor;                            // Determines current side sensor
    uint8_t diag_sensor;                            // Determines current diagonal sensor

    enum {
            UNDER=0,
			FORWARD=1,
			DIVERGING=2,
			BACKWARD = 3,
			CONVERGING = 4,
			OVER = 5
        } orientation;

    enum {                                          
        STOPED=0,
        ADVANCING=1,
        TURNING=2
    } state;                                        // Determines current movement state

    enum {
        LEFT=-1,
        CENTER=0,
		RIGHT=1
    } turn_direction, obstacle_avoiding_side;       // Determines current turning direction, Determines general side to get arround the obstacle

} movement_info;

/***************************INTERNAL FUNCTIONS************************************/

uint16_t delta_to_mm(uint32_t delta){
	//chprintf((BaseSequentialStream *)&SD3, "Conversion distqnce is :  %d ",51.178*exp(-delta*0.001));
    return 10;                //Return an exponential conversion of the distance in cm from the obstacle
}

void update_orientation(){                          // Updates orientation from current turning side (-1 or 1)

    	movement_info.orientation += movement_info.turn_direction*movement_info.obstacle_avoiding_side;
    		if (movement_info.orientation == UNDER) {
    			movement_info.orientation = CONVERGING;
    		}
    			else if (movement_info.orientation == OVER) {
    				movement_info.orientation = FORWARD;
    			}
}

uint8_t status_on_diag(){                          //Returns TRUE if there is an object detected on diagonal

    if (get_prox(movement_info.diag_sensor) > DETECTION_DISTANCE*10
        || get_prox(movement_info.diag_sensor-movement_info.obstacle_avoiding_side) > DETECTION_DISTANCE*10) {


    	return TRUE;
    }
    	else {
    		return FALSE;
    	}
}

uint8_t status_on_front(){                          //Returns TRUE if there is an object detected on front

    if (get_prox(movement_info.front_sensor) > DETECTION_DISTANCE) {
    	return TRUE;
    }
    	else {
    		return FALSE;
    	}
}

uint8_t status_on_side(){                           //Returns TRUE if there is an object detected on object side

    if ((get_prox(movement_info.side_sensor) > DETECTION_DISTANCE - ERROR_TOLERANCE) || (get_prox(movement_info.diag_sensor) > DETECTION_DISTANCE/3)) {

    //	set_led(LED7,100);


    	return TRUE;
    }
	
    	else {
    		return FALSE;
    	}
}

/* The next function advances forward until a change in proximity sensors is detected,
    or if the e-puck is back on the right track
*/
void advance_until_interest_point(int32_t* update_distance, int32_t* deviation_distance, int8_t direction_coefficient){ 
    
    go_forward();
    left_motor_set_pos(0);
    right_motor_set_pos(0);
    

   // chprintf((BaseSequentialStream *)&SD3, "Before while \r\n");
    while ((object_detection() == FALSE) && !((*deviation_distance <= left_motor_get_pos()) && (movement_info.orientation == CONVERGING)))
    {
       // chprintf((BaseSequentialStream *)&SD3, "Advancing :  %d         ", left_motor_get_pos());
        //chprintf((BaseSequentialStream *)&SD3, "     of    :  %d \r\n", *deviation_distance);
    }
    halt();

    chprintf((BaseSequentialStream *)&SD3, "Interest point reached \r\n");
    chprintf((BaseSequentialStream *)&SD3, "Front status : %d \r\n", movement_info.obstacle_detection[0]);

    chprintf((BaseSequentialStream *)&SD3, "POS left motor : %d \r\n",left_motor_get_pos());


    *update_distance += direction_coefficient*left_motor_get_pos();

    if(movement_info.obstacle_detection[1] == FALSE && movement_info.turn_direction) {
    	chprintf((BaseSequentialStream *)&SD3, "Nothing on front");
    	advance_distance(EPUCK_RADIUS);
    }


    find_turning_side();
    chprintf((BaseSequentialStream *)&SD3, "turn_direction is now : %d \r\n", movement_info.turn_direction);
   // if (!((*deviation_distance <= left_motor_get_pos()) && (movement_info.orientation == CONVERGING))) {
   	turn_to(movement_info.turn_direction*STANDARD_TURN_ANGLE);
  //  }
    update_orientation();
    if(movement_info.obstacle_detection[1] == FALSE && movement_info.turn_direction) {
    	advance_distance(EPUCK_RADIUS);
    }

}

void find_turning_side (){                          // Determines witch side to turn to, from proximity sensors

    chprintf((BaseSequentialStream *)&SD3, "Old side : %d \r\n", movement_info.turn_direction);

    if((movement_info.obstacle_avoiding_side == RIGHT)) {
    	if (movement_info.obstacle_detection[0] == TRUE) movement_info.turn_direction = RIGHT;
    	else movement_info.turn_direction = LEFT;
    }

    if((movement_info.obstacle_avoiding_side == LEFT)) {
        	if (movement_info.obstacle_detection[0] == TRUE) movement_info.turn_direction = LEFT;
        	else movement_info.turn_direction = RIGHT;
        }

    if((movement_info.obstacle_detection[0] == FALSE) && movement_info.orientation == CONVERGING)
    		{
    	movement_info.turn_direction = CENTER;
    		}
    
    chprintf((BaseSequentialStream *)&SD3, "New side now : %d \r\n", movement_info.turn_direction);
}

void init_obstacle_tection(){                       // Sets obstacles after a turn
    movement_info.obstacle_detection[0] = 0;
    movement_info.obstacle_detection[1] = abs(movement_info.turn_direction); //changes to 0 if advancing without sidewall
}

void set_turning_direction() {                      // Finds inicial turning direction, looks for a "shorter" side, or a slope


    if(get_prox(FRONT_LEFT_SENSOR) < get_prox(FRONT_RIGHT_SENSOR)) {

        chprintf((BaseSequentialStream *)&SD3, "Turning LEFT : %d \r\n", movement_info.turn_direction);
        movement_info.turn_direction = LEFT; 
        movement_info.obstacle_avoiding_side = LEFT;
        movement_info.front_sensor = FRONT_FRONT_LEFT_SENSOR;
        movement_info.side_sensor = RIGHT_SENSOR;
        movement_info.diag_sensor = FRONT_RIGHT_SENSOR;
    }
    else{
        chprintf((BaseSequentialStream *)&SD3, "Turning Right : %d \r\n", movement_info.turn_direction);

        movement_info.turn_direction = RIGHT;
        movement_info.obstacle_avoiding_side = RIGHT;
        movement_info.front_sensor = FRONT_FRONT_RIGHT_SENSOR;
        movement_info.side_sensor = LEFT_SENSOR;
        movement_info.diag_sensor = FRONT_LEFT_SENSOR;

    }
    chprintf((BaseSequentialStream *)&SD3, "turn_direction : %d,  ", movement_info.turn_direction);
   chprintf((BaseSequentialStream *)&SD3, "front_sensor : %d,  ", movement_info.front_sensor);
    chprintf((BaseSequentialStream *)&SD3, "side_sensor : %d,  ", movement_info.side_sensor);
}

void advance_distance(uint16_t distance){           // Makes the e-puck go forward a certain amount of mm


	//chprintf((BaseSequentialStream *)&SD3, "going forward until : %d/r/d",(distance/CONVERSION_CM_MM)* NSTEP_ONE_TURN / WHEEL_PERIMETER);
    go_forward();
    left_motor_set_pos(0);
    right_motor_set_pos(0);

    while (left_motor_get_pos() < (distance/CONVERSION_CM_MM)* NSTEP_ONE_TURN / WHEEL_PERIMETER) {
    }
    halt();
}

void halt(){                                        // Stops the motors
    movement_info.state = STOPED;
    left_motor_set_speed(0);
    right_motor_set_speed(0);
    
    chprintf((BaseSequentialStream *)&SD3, "Stopping\r\n");
}

void turn_to(int angle){                            // Turns the e-puck a derired angle


	// mettre un center ? ajuste angle erreur du au selector


	chprintf((BaseSequentialStream *)&SD3, "Turning to : %d \r\n", angle);
    movement_info.state = TURNING;


    float temp = angle;         //Ã  faire en plus clean

    left_motor_set_pos(0);
    right_motor_set_pos(0);

    // The e-puck will pivot on itself


    left_motor_set_speed(movement_info.turn_direction*MOTOR_SPEED_LIMIT/2);
    right_motor_set_speed(-movement_info.turn_direction*MOTOR_SPEED_LIMIT/2);
    // Turns until the desired angle is reached
    while ((abs(left_motor_get_pos()) < abs((temp/FULL_PERIMETER_DEG)*NSTEP_ONE_TURN*CORRECTION_FACTOR))
    	&& (abs(right_motor_get_pos()) < abs((temp/FULL_PERIMETER_DEG)*NSTEP_ONE_TURN*CORRECTION_FACTOR))) {


    	temp += 0.000003;
	}
    halt();
}

/*************************END INTERNAL FUNCTIONS**********************************/

/****************************PUBLIC FUNCTIONS*************************************/
void movement_init(){     //Initiates some values and turns to selected direction.
    movement_info.orientation = 0;

	chprintf((BaseSequentialStream *)&SD3, "Movement Init");
    int selector_angle = 0;

    switch(get_selector()) {
		case 0:

		case 1:

		case 2:

		case 3:

		case 4:
			selector_angle = get_selector()*FULL_PERIMETER_DEG/SELECTOR_MAX + SELECTOR_OFFSET;
			break;
		case 5:

		case 6:

		case 7:

		case 8:

		case 9:

		case 10:

		case 11:

		case 12:

		case 13:

		case 14:

		case 15:
			selector_angle = get_selector()*FULL_PERIMETER_DEG/SELECTOR_MAX - 3*SELECTOR_OFFSET;
			break;
    }

    if(selector_angle < 0){
    	movement_info.turn_direction = LEFT;
    	selector_angle -= FULL_PERIMETER_DEG/SELECTOR_MAX;
    }
    else if (selector_angle > 0) {
    	selector_angle += FULL_PERIMETER_DEG/SELECTOR_MAX;
    	movement_info.turn_direction = RIGHT;
    }
    turn_to(selector_angle);
}

uint8_t object_detection(){                         //Returns TRUE if there is a change on proximity sensors

    
   // chprintf((BaseSequentialStream *)&SD3, "Status on front : %d \r\n",status_on_front());
  //  chprintf((BaseSequentialStream *)&SD3, "Status on side : %d \r\n",status_on_side());

    if(status_on_front() == TRUE || status_on_diag()) {
        movement_info.obstacle_detection[0] = TRUE;
    	chprintf((BaseSequentialStream *)&SD3, "Something on front \r\n");
        return TRUE;
    }
    else if(status_on_side() == FALSE && movement_info.turn_direction) {
        movement_info.obstacle_detection[1] = FALSE;
    	chprintf((BaseSequentialStream *)&SD3, "Change on side \r\n");
        return TRUE;
    }
    return FALSE;
}

void avoid_obstacle(){                              // Function used to avoid an obstacle

    int32_t deviation_distance = 0;                 // Distance from main track, in motor steps
    int32_t forward_distance = 0;                   // Distance from initial object detection, in motor steps

    //initialising all components for first turn
    movement_info.on_right_track = NO;              
    set_turning_direction();
    init_obstacle_tection();
    turn_to(movement_info.turn_direction*STANDARD_TURN_ANGLE);
    movement_info.orientation = DIVERGING;

    while (movement_info.on_right_track == 0)           //Each loop is a moove forward and turn, until e-puck is back on main path
    { 
        init_obstacle_tection();

        switch(movement_info.orientation) {
            case FORWARD:
            	set_led(LED1 ,100);
                chprintf((BaseSequentialStream *)&SD3, "Case FORWARD \r\n");
                advance_until_interest_point(&forward_distance, &deviation_distance, FURTHER);
                chprintf((BaseSequentialStream *)&SD3, "Forward distance now  %d \r\n",forward_distance);
                chprintf((BaseSequentialStream *)&SD3, "Deviation distance now  %d \r\n",deviation_distance);
                clear_leds();
                break;

            case BACKWARD:
            	set_led(LED5 ,100);
                chprintf((BaseSequentialStream *)&SD3, "Case BACKWARD \r\n");
                advance_until_interest_point(&forward_distance, &deviation_distance, CLOSER);
                chprintf((BaseSequentialStream *)&SD3, "Forward distance now  %d \r\n",forward_distance);
                chprintf((BaseSequentialStream *)&SD3, "Deviation distance now  %d \r\n",deviation_distance);
                clear_leds();
                break;
            
            case DIVERGING:                         //Diverging from main path
            	set_led(LED7 ,100);
                chprintf((BaseSequentialStream *)&SD3, "Case DIVERGING \r\n");
                advance_until_interest_point(&deviation_distance, &deviation_distance, FURTHER);
                chprintf((BaseSequentialStream *)&SD3, "Forward distance now  %d \r\n",forward_distance);
                chprintf((BaseSequentialStream *)&SD3, "Deviation distance now  %d \r\n",deviation_distance);
                clear_leds();
                break;
            
            case CONVERGING:                        //Diverging from main path
            	set_led(LED3 ,100);
                chprintf((BaseSequentialStream *)&SD3, "Case CONVERGING \r\n");
                advance_until_interest_point(&deviation_distance, &deviation_distance, CLOSER);
                chprintf((BaseSequentialStream *)&SD3, "Forward distance now  %d \r\n",forward_distance);
                chprintf((BaseSequentialStream *)&SD3, "Deviation distance now  %d \r\n",deviation_distance);
                clear_leds();
                break;
        };

        chprintf((BaseSequentialStream *)&SD3, "JUSTE AVANT LE IF \r\n");

        //If the e-puck is back on the main track and we advanced at least a little bit forward, the the obstacle is avoided
        if(deviation_distance <= 0 && forward_distance > 0) {
        	movement_info.on_right_track = TRUE;
        	if (movement_info.obstacle_avoiding_side == RIGHT) {
        		movement_info.turn_direction = RIGHT;
        	}
        	else {
        		movement_info.turn_direction = LEFT;
        	}
        	turn_to(STANDARD_TURN_ANGLE);
        }
    }
}

void go_forward(){                                  // Makes the e-puck go forward indefinetly
    movement_info.state = ADVANCING;
    left_motor_set_speed(MOTOR_SPEED_LIMIT/2);
    right_motor_set_speed(MOTOR_SPEED_LIMIT/2);
}

/**************************END PUBLIC FUNCTIONS***********************************/
