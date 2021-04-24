//#include <motors.h>
#include "movement_control.h"
#include <math.h>

typedef unsigned char   uint8_t;
typedef short  int16_t;
typedef unsigned   uint32_t;
typedef int  int32_t;

#define NSTEP_ONE_TURN      1000 // number of step for 1 turn of the motor
#define WHEEL_PERIMETER     13 // [cm]
#define STANDARD_SPEED      5 // standard speed used for the majority of movements
#define WHEEL_DISTANCE      5.35f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)

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

struct movement{

    enum {
        SECOND_PHASE = -1,
        ON_RIGHT_TRACK = 0,
        FIRST_PHASE = 1
    } obstacle_avoid_state;

    enum {
        STOPED=0,
        ADVANCING=1,
        TURNING=2
    } state;

    enum {
        LEFT=-1,
        CENTER=0,
        RIGHT=1
    } turn_direction;

} movement_info;

void left_motor_set_pos(uint32_t x){};
uint32_t left_motor_get_pos(){return 3;};

void movement_init(){

    movement_info.obstacle_avoid_state = 0;

    int tmp_angle;
    printf("Select init angle :");
    scanf("%d", &tmp_angle);
    analyse_angle(&tmp_angle);
    turn_to(tmp_angle); //get_selector
};

void analyse_angle(int* angle){

    printf("Analysing given angle %d", *angle);
    if(*angle > 180) *angle = *angle - 360;
    printf("Angle changed to : %d", *angle);
    if(angle <= 0) movement_info.turn_direction = LEFT;
    else movement_info.turn_direction = RIGHT;
};

uint8_t tmp; //à deleter lors de la création de la fonction
uint8_t object_detection(){

    printf("Select object detected(1) or not (0) : ");
    scanf("%d", &tmp);

    return tmp;
};

void avoid_obstacle(){

    uint32_t travel_distance = 0;//à expliquer pourquoi autant;
    uint32_t distance_travelled = 0;

    set_closer_side();
    movement_info.obstacle_avoid_state = FIRST_PHASE;

    while (movement_info.obstacle_avoid_state)
    {
        turn_to(movement_info.turn_direction*movement_info.obstacle_avoid_state*90);
        left_motor_set_pos(0);

        if (movement_info.obstacle_avoid_state == FIRST_PHASE){
            printf("On first phase, advancing \n");
            advance_until_clear();
            travel_distance += left_motor_get_pos();
            printf("Travel distance : %d \n", travel_distance);
            printf("Distance travelled : %d \n", distance_travelled);
            turn_to(-movement_info.turn_direction*movement_info.obstacle_avoid_state*90);
        }
        else if (movement_info.obstacle_avoid_state == SECOND_PHASE){
            printf("On second phase, advancing \n");
            go_forward();
            while (!object_detection() && distance_travelled < travel_distance) //object_detection()
            {
                printf("Going forward\n");
                distance_travelled = distance_travelled + left_motor_get_pos(); //remove when real program
                printf("Travel distance : %d \n", travel_distance);
                printf("Distance travelled : %d \n", distance_travelled);
            }
            //distance_travelled = distance_travelled + left_motor_get_pos();
            turn_to(-movement_info.turn_direction*movement_info.obstacle_avoid_state*90);
            if (!(distance_travelled-travel_distance)) {movement_info.obstacle_avoid_state = ON_RIGHT_TRACK;printf("changing state : %d :", movement_info.obstacle_avoid_state);};
        }

        if(object_detection() && movement_info.obstacle_avoid_state) {
            printf("Object detected \n");
        }
        else if(movement_info.obstacle_avoid_state){
            printf("No object detected, advancing to second phase \n");
            advance_until_clear();
            movement_info.obstacle_avoid_state = SECOND_PHASE;
            printf("changing state : %d :", movement_info.obstacle_avoid_state);
        }
    }

};

void advance_until_clear(){
    printf("Advancing until clear \n");
};

void set_closer_side(){ //à revoir avec les capteurs à distance
    int sensor1_intensity = 0;
    int sensor2_intensity = 1;
    if(sensor1_intensity < sensor2_intensity) movement_info.turn_direction = LEFT;
    else movement_info.turn_direction = RIGHT;
};

void go_forward(){
    movement_info.state = ADVANCING;
    printf("left_motor_set_speed(STANDARD_SPEED)");
    printf("right_motor_set_speed(STANDARD_SPEED);");
};

void turn_to(int angle){

    movement_info.state = TURNING;
    /*
    left_motor_set_speed(-movement_info.turn_direction*STANDARD_SPEED);\n
    right_motor_set_speed(movement_info.turn_direction*STANDARD_SPEED);\n

    left_motor_set_pos(0);
    right_motor_set_pos(0);

    while (abs(left_motor_get_pos()) < (PERIMETER_EPUCK*360/angle)* NSTEP_ONE_TURN / WHEEL_PERIMETER &&
           abs(right_motor_get_pos()) < (PERIMETER_EPUCK*360/angle)* NSTEP_ONE_TURN / WHEEL_PERIMETER)
    {
        printf("Turning ...");
    };

    left_motor_set_speed(0);
    right_motor_set_speed(0);

    */

    movement_info.state = STOPED;
    
    printf("turned to: %d \n", angle);
};


