#include "motors.h"
#include "movement_control.h"
#include <math.h>
#include "selector.h"
#include "chprintf.h"

typedef unsigned char   uint8_t;


#define NSTEP_ONE_TURN              1000 // number of step for 1 turn of the motor
#define WHEEL_PERIMETER             13 // [cm]
#define STANDARD_SPEED              5 // standard speed used for the majority of movements
#define WHEEL_DISTANCE              5.35f    //cm
#define PERIMETER_EPUCK             (3.14 * WHEEL_DISTANCE)
#define EPUCK_RADIUS                40 //mm
#define SELECTOR_OFFSET 	        90
#define SELECTOR_MAX		        16
#define FULL_PERIMETER_DEG          360
#define FRONT_FRONT_RIGHT_SENSOR    0
#define FRONT_FRONT_LEFT_SENSOR     7
#define FRONT_RIGHT_SENSOR          1
#define FRONT_LEFT_SENSOR           6
#define RIGHT_SENSOR                2
#define LEFT_SENSOR                 5

#define CONVERSION_CM_MM    10

#define CLOSER              -1
#define FURTHER              1

#define TRUE                 1
#define FALSE                0

#define DETECTION_DISTANCE   100 //delta
#define ERROR_TOLERANCE      40 

/****************************TEST MAIN ************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "movement_control.h"

int main() {
    //init motor, camera and sensors + other modules
    //Put all threads to sleep except motor
    movement_init();
    printf("Aligned\n");
    //start camera and sensor threads
    go_forward();
    while (object_detection())
    {
        avoid_obstacle();
        printf("Obstacle avoided \n");
    }

   printf("Arrived at destination");
   while (1)
   {
   };
   
   return 0;
}

/****************************TEST MAIN ************************/
proximity_msg_t prox_values;
struct movement{

    enum {
        NO = 0,
        YES = 1
    } on_right_track;

    uint8_t obstacle_detection[2]; // FRONT,SIDE

    uint8_t front_sensor;
    uint8_t side_sensor;

    enum {
        FORWARD = 0,
        DIVERGING = 1,
        BACKWARD = 2,
        CONVERGING = 3
    } orientation;

    enum {
        STOPED=0,
        ADVANCING=1,
        TURNING=2
    } state;

    enum {
        RIGHT=-1,
        CENTER=0,
        LEFT=1
    } turn_direction, obstacle_avoiding_side;

} movement_info;

void left_motor_set_pos(uint32_t x){};
uint32_t left_motor_get_pos(){return 3;};

uint16_t delta_to_cm(uint32_t delta){
    return 51.178*exp(-delta*0.001);        //Magic number
}

void movement_init(){

    movement_info.orientation = 0;
    
    int tmp_angle = 0; //get_selector
    //tmp_angle = get_selector()*FULL_PERIMETER_DEG/SELECTOR_MAX +SELECTOR_OFFSET;
    analyse_angle(&tmp_angle);
    turn_to(tmp_angle); //get_selector
};

void update_orientation(){
    movement_info.orientation += -movement_info.turn_direction;
    if (movement_info.orientation > CONVERGING) movement_info.orientation = FORWARD;
    if (movement_info.orientation < FORWARD) movement_info.orientation = CONVERGING;
    printf("New orientation : %d\n", movement_info.orientation);
};

void analyse_angle(int* angle){

    printf("Analysing given angle %d", *angle);
    if(*angle > FULL_PERIMETER_DEG/2) *angle = *angle - FULL_PERIMETER_DEG;
    printf("Angle changed to : %d", *angle);
    if(*angle <= 0) movement_info.turn_direction = LEFT;
    else movement_info.turn_direction = RIGHT;
};

uint8_t tmp; //à deleter lors de la création de la fonction
uint8_t status_on_front(){ 
    if (prox_values.delta[movement_info.front_sensor] > DETECTION_DISTANCE - ERROR_TOLERANCE
        || prox_values.delta[movement_info.front_sensor-movement_info.obstacle_avoiding_side] > DETECTION_DISTANCE - ERROR_TOLERANCE) return 1;
    //printf("Select object detected on front (1) or not (0) : ");
    //scanf("%d", &tmp);
    else return 0;
};
uint8_t status_on_side(){ 
    if (prox_values.delta[movement_info.side_sensor] > DETECTION_DISTANCE - ERROR_TOLERANCE) return 1;
    //printf("Select object detected on side (1) or not (0) : ");
    //scanf("%d", &tmp);
    else return 0;
};

uint8_t object_detection(){

    printf("Detecting objects\n");
    if(status_on_front() != movement_info.obstacle_detection[0]) {
        movement_info.obstacle_detection[0] = !movement_info.obstacle_detection[0];
        printf("Detecting change up front\n");
        return 1;
    }
    else if(status_on_side()!= movement_info.obstacle_detection[1]) {
        movement_info.obstacle_detection[1] = !movement_info.obstacle_detection[1];
        printf("Detecting change on sides\n");
        return 1;
    }
    return 0;
};

void advance_until_interest_point(int32_t* update_distance, int32_t* deviation_distance, int8_t direction_coefficient){ //until change in "obstacle_detection" 
    
    printf("Advancing until interest point \n");
    go_forward();
    left_motor_set_pos(0);
    while (!object_detection() && *deviation_distance - left_motor_get_pos())
    {
        printf(". . . \n");
    }
    halt();
    if(!movement_info.obstacle_detection[1]) advance_distance(EPUCK_RADIUS + delta_to_cm(DETECTION_DISTANCE));
    printf("Now updating number : %d", *update_distance);
    *update_distance += direction_coefficient*left_motor_get_pos();
    printf(" to : %d \n", *update_distance);
    find_turning_side();
};

void find_turning_side (){
    
    if(movement_info.obstacle_detection[0]) movement_info.turn_direction = RIGHT;
    else movement_info.turn_direction = LEFT;
    if(movement_info.obstacle_detection[0] && movement_info.orientation == CONVERGING) movement_info.turn_direction = CENTER;
    printf("Turning side now : %d \n", movement_info.turn_direction);
    printf("front obstacle : %d\n", movement_info.obstacle_detection[0]);

};

void init_obstacle_tection(){
    movement_info.obstacle_detection[0] = 0;
    movement_info.obstacle_detection[1] = abs(movement_info.turn_direction); //changes to 0 if advancing without sidewall
};

void avoid_obstacle(){

    int32_t deviation_distance = 0;//à expliquer pourquoi autant;
    int32_t forward_distance = 0;

    movement_info.on_right_track = NO;
    set_turning_direction();
    init_obstacle_tection();
    turn_to(movement_info.turn_direction*90);
    movement_info.orientation = DIVERGING;

    while (!movement_info.on_right_track)
    { 
        init_obstacle_tection();
        printf("Orientation analysis : %d \n", movement_info.orientation);
        switch(movement_info.orientation) {
            case FORWARD:
                printf("Going FORWARD \n");
                advance_until_interest_point(&forward_distance, &deviation_distance, FURTHER);
                turn_to(-movement_info.turn_direction*90);
                update_orientation();
                printf("FORWARD - Forward distance : %d \n", forward_distance);
                printf("FORWARD - Deviation travelled : %d \n", deviation_distance);
                break;

            case BACKWARD:
                printf("Going BACKWARDS \n");
                advance_until_interest_point(&forward_distance, &deviation_distance, CLOSER);
                turn_to(-movement_info.turn_direction*90);
                update_orientation();
                printf("BACKWARDS - Forward distance : %d \n", forward_distance);
                printf("BACKWARDS - Deviation travelled : %d \n", deviation_distance);
                break;
            
            case DIVERGING:
                printf("Going DIVERGING \n");
                advance_until_interest_point(&deviation_distance, &deviation_distance, FURTHER);
                turn_to(-movement_info.turn_direction*90);
                update_orientation();
                printf("DIVERGING - Forward distance : %d \n", forward_distance);
                printf("DIVERGING - Deviation travelled : %d \n", deviation_distance);
                break;
            
            case CONVERGING:
                printf("Going CONVERGING \n");
                advance_until_interest_point(&deviation_distance, &deviation_distance, CLOSER);
                turn_to(-movement_info.turn_direction*90);
                update_orientation();
                printf("CONVERGING - Forward distance : %d \n", forward_distance);
                printf("CONVERGING - Deviation travelled : %d \n", deviation_distance);
                break;
        };
        if(deviation_distance == 0 && forward_distance > 0) movement_info.on_right_track = TRUE;
    };
};

void set_turning_direction() { //à revoir avec les capteurs à distance
    printf("Setting turn direction\n");
    if(prox_values.delta[FRONT_FRONT_LEFT_SENSOR] < prox_values.delta[FRONT_FRONT_RIGHT_SENSOR]) {
        movement_info.turn_direction = LEFT;
        movement_info.front_sensor = FRONT_FRONT_RIGHT_SENSOR;
        movement_info.side_sensor = RIGHT_SENSOR;
    }
    else{
        movement_info.turn_direction = RIGHT;
        movement_info.front_sensor = FRONT_FRONT_LEFT_SENSOR;
        movement_info.side_sensor = RIGHT_SENSOR;
    };
};

void go_forward(){
    movement_info.state = ADVANCING;
    //left_motor_set_speed(STANDARD_SPEED);
    //right_motor_set_speed(STANDARD_SPEED);
    printf("FORWARD\n");
};

void advance_distance(uint16_t distance){ //mm

    go_forward()
    left_motor_set_pos(0);
    right_motor_set_pos(0);

    while (left_motor_get_pos() < (distance/(PERIMETER_EPUCK*CONVERSION_CM_MM))* NSTEP_ONE_TURN / WHEEL_PERIMETER)
    {
        printf("going ...");
    };

    left_motor_set_speed(0);
    right_motor_set_speed(0);

    movement_info.state = STOPED;
}

void halt(){
    movement_info.state = STOPED;
    //left_motor_set_speed(0);
    //right_motor_set_speed(0);
    printf("HALT\n");
};

void turn_to(int angle){

    movement_info.state = TURNING;

    left_motor_set_speed(-movement_info.turn_direction*MOTOR_SPEED_LIMIT);
    right_motor_set_speed(movement_info.turn_direction*MOTOR_SPEED_LIMIT);

    chprintf((BaseSequentialStream *)&SD3, "pos motor %d ",right_motor_get_pos());

    left_motor_set_pos(0);
    right_motor_set_pos(0);

    double temp = angle;

    while (abs(left_motor_get_pos()) < abs((temp/FULL_PERIMETER_DEG)* NSTEP_ONE_TURN) &&
               abs(right_motor_get_pos()) < abs((temp/FULL_PERIMETER_DEG)* NSTEP_ONE_TURN )) {
    	chprintf((BaseSequentialStream *)&SD3, "dans while \r\n ");
    	chprintf((BaseSequentialStream *)&SD3, "pos motor %d ",right_motor_get_pos());
	};


   left_motor_set_speed(0);
   right_motor_set_speed(0);

   movement_info.state = STOPED;
    

};
