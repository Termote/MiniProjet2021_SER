#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "motors.h"
#include "movement_control.h"
#include <math.h>
#include "selector.h"
#include "chprintf.h"


#define NSTEP_ONE_TURN              1000    // number of step for 1 turn of the motor
#define WHEEL_PERIMETER             13      // cm
#define WHEEL_DISTANCE              5.35f   //cm
#define PERIMETER_EPUCK             (3.14 * WHEEL_DISTANCE)     //PI * Wheel distance
#define EPUCK_RADIUS                40      //mm
#define SELECTOR_OFFSET 	        90      //angle offset due to position of 0 on the selector wheel compared to the front
#define SELECTOR_MAX		        16      //maximum value for the selector
#define FULL_PERIMETER_DEG          360     //Degrees needed for the full perimeter
#define FRONT_FRONT_RIGHT_SENSOR    0       // Sensor number for front sensor slightly to the right
#define FRONT_FRONT_LEFT_SENSOR     7       // Sensor number for front sensor slightly to the left
#define FRONT_RIGHT_SENSOR          1       // Sensor number for right sensor at 45°
#define FRONT_LEFT_SENSOR           6       // Sensor number for left sensor at -45°
#define RIGHT_SENSOR                2       // Sensor number for right sensor at 90°
#define LEFT_SENSOR                 5       // Sensor number for left sensor at -90°

#define CONVERSION_CM_MM            10      // Decimal differrence between cm and mm
#define STANDARD_TURN_ANGLE         90      // Standard turning angle

#define CLOSER                      -1      // Multiplier used for distance calculation when getting closer to the objective
#define FURTHER                     1       // Multiplier used for distance calculation when getting closer to the objective

#define TRUE                        1       
#define FALSE                       0     

#define DETECTION_DISTANCE          100     // Desired sensor distance detection, in delta
#define ERROR_TOLERANCE             40      // Distance error range,  in delta

/***************************GLOBAL VARIABLES************************************/

proximity_msg_t* prox_values;                       // Proximity sensor information

struct movement{                                    // Struct with all information about the e-puck movement state and general information

    enum {
        NO = 0,
        YES = 1 
    } on_right_track;                               

    uint8_t obstacle_detection[2];                  // 1 if there is an obstacle on : [FRONT,SIDE]
    uint8_t front_sensor;                           // Determines current front sensor
    uint8_t side_sensor;                            // Determines current side sensor

    enum {
        FORWARD = 0,
        DIVERGING = 1,
        BACKWARD = 2,
        CONVERGING = 3
    } orientation;                                  // Determines current orientation compared to the main track

    enum {                                          
        STOPED=0,
        ADVANCING=1,
        TURNING=2
    } state;                                        // Determines current movement state

    enum {
        RIGHT=-1,
        CENTER=0,
        LEFT=1
    } turn_direction, obstacle_avoiding_side;       // Determines current turning direction, Determines general side to get arround the obstacle

} movement_info;

/***************************INTERNAL FUNCTIONS************************************/

uint16_t delta_to_cm(uint32_t delta){

    return 51.178*exp(-delta*0.001);                //Return an exponential conversion of the distance in cm from the obstacle
}

void update_orientation(){                          // Updates orientation from current turning side (-1 or 1)

    movement_info.orientation += -movement_info.turn_direction;
    if (movement_info.orientation > CONVERGING) movement_info.orientation = FORWARD;
    else if (movement_info.orientation < FORWARD) movement_info.orientation = CONVERGING;
}

void analyse_angle(int* angle){                     // Transforms an angle from [0°, 360°] to [-180°, 180°] format, and sets turn direction

    if(*angle > FULL_PERIMETER_DEG/2) *angle = *angle - FULL_PERIMETER_DEG;
    if(*angle <= 0) movement_info.turn_direction = LEFT;
    else movement_info.turn_direction = RIGHT;
}

uint8_t status_on_front(){                          //Returns TRUE if there is an object detected on front
    chprintf((BaseSequentialStream *)&SD3, "Analysing front, delta at :  %d \r\n",get_prox(movement_info.front_sensor));

    if (get_prox(movement_info.front_sensor) > DETECTION_DISTANCE - ERROR_TOLERANCE
        || get_prox(movement_info.front_sensor-movement_info.obstacle_avoiding_side) > DETECTION_DISTANCE - ERROR_TOLERANCE) return TRUE;
    else return FALSE;
}

uint8_t status_on_side(){                           //Returns TRUE if there is an object detected on object side

    if (get_prox(movement_info.side_sensor) > DETECTION_DISTANCE - ERROR_TOLERANCE) return TRUE;
	
    else return FALSE;
}

/* The next function advances forward until a change in proximity sensors is detected,
    or if the e-puck is back on the right track
*/
void advance_until_interest_point(int32_t* update_distance, int32_t* deviation_distance, int8_t direction_coefficient){ 
    
    go_forward();
    left_motor_set_pos(0);
    
    chprintf((BaseSequentialStream *)&SD3, "Forward distance now  %d \r\n",forward_distance);
    while (object_detection() == 0 && (*deviation_distance - left_motor_get_pos() || movement_info.orientation == CONVERGING))
    {
        chprintf((BaseSequentialStream *)&SD3, "Advancing :  %d \r\n", left_motor_get_pos());
    }
    halt();
    chprintf((BaseSequentialStream *)&SD3, "Interest point reached \r\n");

    if(movement_info.obstacle_detection[1] == 0) advance_distance(EPUCK_RADIUS + delta_to_cm(DETECTION_DISTANCE));
    *update_distance += direction_coefficient*left_motor_get_pos();
    
    find_turning_side();
    turn_to(-movement_info.turn_direction*STANDARD_TURN_ANGLE);
    update_orientation();
    if(movement_info.obstacle_detection[1] == 0) advance_distance(EPUCK_RADIUS + delta_to_cm(DETECTION_DISTANCE));

}

void find_turning_side (){                          // Determines witch side to turn to, from proximity sensors
    
    if(movement_info.obstacle_detection[0] && movement_info.orientation == CONVERGING) movement_info.turn_direction = CENTER;
    else if(movement_info.obstacle_detection[0]) movement_info.turn_direction = RIGHT;
    else movement_info.turn_direction = LEFT;
    chprintf((BaseSequentialStream *)&SD3, "New side now : %d \r\n", movement_info.turn_direction);
}

void init_obstacle_tection(){                       // Sets obstacles after a turn
    movement_info.obstacle_detection[0] = 0;
    movement_info.obstacle_detection[1] = abs(movement_info.turn_direction); //changes to 0 if advancing without sidewall
}

void set_turning_direction() {                      // Finds inicial turning direction, looks for a "shorter" side, or a slope

    if(get_prox(FRONT_LEFT_SENSOR) < det_prox(FRONT_RIGHT_SENSOR)) {
        movement_info.turn_direction = LEFT; 
        movement_info.obstacle_avoiding_side = LEFT;
        movement_info.front_sensor = FRONT_FRONT_RIGHT_SENSOR;
        movement_info.side_sensor = RIGHT_SENSOR;
    }
    else{
        movement_info.turn_direction = RIGHT;
        movement_info.obstacle_avoiding_side = RIGHT;
        movement_info.front_sensor = FRONT_FRONT_LEFT_SENSOR;
        movement_info.side_sensor = RIGHT_SENSOR;
    };
}

void advance_distance(uint16_t distance){           // Makes the e-puck go forward a certain amount of mm

    go_forward();
    left_motor_set_pos(0);
    right_motor_set_pos(0);

    while (left_motor_get_pos() < (distance/CONVERSION_CM_MM)* NSTEP_ONE_TURN / WHEEL_PERIMETER)
    {
    };
    halt();
}

void halt(){                                        // Stops the motors
    movement_info.state = STOPED;
    left_motor_set_speed(0);
    right_motor_set_speed(0);
    
    chprintf((BaseSequentialStream *)&SD3, "Stopping\r\n");
}

void turn_to(int angle){                            // Turns the e-puck a derired angle

    movement_info.state = TURNING;
    // The e-puck will pivot on itself
    left_motor_set_speed(-movement_info.turn_direction*MOTOR_SPEED_LIMIT);
    right_motor_set_speed(movement_info.turn_direction*MOTOR_SPEED_LIMIT);

    left_motor_set_pos(0);
    right_motor_set_pos(0);

    float temp = angle;         //à faire en plus clean 

    // Turns until the desired angle is reached
    while (abs(left_motor_get_pos()) < abs((temp/FULL_PERIMETER_DEG)* NSTEP_ONE_TURN) &&
               abs(right_motor_get_pos()) < abs((temp/FULL_PERIMETER_DEG)* NSTEP_ONE_TURN )) {
	};
    halt();
}

/*************************END INTERNAL FUNCTIONS**********************************/

/****************************PUBLIC FUNCTIONS*************************************/
void movement_init(proximity_msg_t* proximity){     //Initiates some values and turns to selected direction.
    prox_values = proximity;
    movement_info.orientation = 0;

    int selector_angle = 0;
    selector_angle = get_selector()*FULL_PERIMETER_DEG/SELECTOR_MAX +SELECTOR_OFFSET;
    analyse_angle(&selector_angle);
    turn_to(selector_angle);
}

uint8_t object_detection(){                         //Returns TRUE if there is a change on proximity sensors

    
    chprintf((BaseSequentialStream *)&SD3, "Status on front : %d \r\n",status_on_front()); 
    chprintf((BaseSequentialStream *)&SD3, "Status on side : %d \r\n",status_on_side());

    if(status_on_front() == TRUE) {
        movement_info.obstacle_detection[0] = TRUE;
    	chprintf((BaseSequentialStream *)&SD3, "Something on front \r\n");
        return TRUE;
    }
    else if(status_on_side() == FALSE && movement_info.turn_direction) {
        movement_info.obstacle_detection[1] = FALSE;
    	chprintf((BaseSequentialStream *)&SD3, "Something on side \r\n");
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
                chprintf((BaseSequentialStream *)&SD3, "Case FORWARD \r\n");
                advance_until_interest_point(&forward_distance, &deviation_distance, FURTHER);
                chprintf((BaseSequentialStream *)&SD3, "Forward distance now  %d \r\n",forward_distance);
                chprintf((BaseSequentialStream *)&SD3, "Deviation distance now  %d \r\n",deviation_distance);
                break;

            case BACKWARD:
            
                chprintf((BaseSequentialStream *)&SD3, "Case BACKWARD \r\n");
                advance_until_interest_point(&forward_distance, &deviation_distance, CLOSER);
                chprintf((BaseSequentialStream *)&SD3, "Forward distance now  %d \r\n",forward_distance);
                chprintf((BaseSequentialStream *)&SD3, "Deviation distance now  %d \r\n",deviation_distance);
                break;
            
            case DIVERGING:                         //Diverging from main path
                
                chprintf((BaseSequentialStream *)&SD3, "Case DIVERGING \r\n");
                advance_until_interest_point(&deviation_distance, &deviation_distance, FURTHER);
                chprintf((BaseSequentialStream *)&SD3, "Forward distance now  %d \r\n",forward_distance);
                chprintf((BaseSequentialStream *)&SD3, "Deviation distance now  %d \r\n",deviation_distance);
                break;
            
            case CONVERGING:                        //Diverging from main path
            
                chprintf((BaseSequentialStream *)&SD3, "Case CONVERGING \r\n");
                advance_until_interest_point(&deviation_distance, &deviation_distance, CLOSER);
                chprintf((BaseSequentialStream *)&SD3, "Forward distance now  %d \r\n",forward_distance);
                chprintf((BaseSequentialStream *)&SD3, "Deviation distance now  %d \r\n",deviation_distance);
                break;
        };
        //If the e-puck is back on the main track and we advanced at least a liitle bit forward, the the obstacle is avoided
        if(deviation_distance == 0 && forward_distance > 0) movement_info.on_right_track = TRUE;
    };
}

void go_forward(){                                  // Makes the e-puck go forward indefinetly
    movement_info.state = ADVANCING;
    left_motor_set_speed(MOTOR_SPEED_LIMIT);
    right_motor_set_speed(MOTOR_SPEED_LIMIT);
}

/**************************END PUBLIC FUNCTIONS***********************************/
