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
        avoid_obstacle(1);
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
        NO = 0,
        YES = 1
    } on_right_track;

    enum {
        CONVERGING = -1,
        PARRALLEL = 0,
        DEVIATING = 1
    } orientation;

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

    movement_info.orientation = 0;

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

uint8_t advance_until_clear(uint32_t* update_distance, uint8_t calculate_distance, uint32_t max_distance, uint8_t apply_maximum){
    printf("Advancing until clear \n");
    printf("Is there something up front (0) or only side (1) : ");
    scanf("%d", &tmp);
    if(calculate_distance)printf("Now updating number : %d", *update_distance);
    if(calculate_distance) *update_distance += left_motor_get_pos();
    if(calculate_distance)printf(" to : %d \n", *update_distance);
    if(apply_maximum) printf("Maximum travel is : %d \n", max_distance);
    return tmp;
};

uint8_t avoid_obstacle(uint8_t not_main_track){

    movement_info.turn_direction = RIGHT; //à retirer lors de "set_turning_direction" est fait
    uint32_t deviation_distance = 0;//à expliquer pourquoi autant;
    uint32_t distance_travelled = 0;

    movement_info.on_right_track = NO;
    set_turning_direction();
    movement_info.orientation = DEVIATING;
    printf("1 Travel distance : %d \n", deviation_distance);
    printf("1 Distance travelled : %d \n", distance_travelled);
    turn_to(-movement_info.turn_direction*movement_info.orientation*90);

    while (!movement_info.on_right_track)
    { 
        if (movement_info.orientation == DEVIATING){
            printf("On first phase (DEVIATING) \n");
            left_motor_set_pos(0);
            while(!advance_until_clear(&deviation_distance,1,0,0) && movement_info.orientation == DEVIATING) {
            printf("==============Creating new obstacle============== \n");
            movement_info.orientation += avoid_obstacle(1);
                    };

            turn_to(movement_info.turn_direction*movement_info.orientation*90);
            movement_info.orientation = PARRALLEL;
            printf("DEVIATING - Travel distance : %d \n", deviation_distance);
            printf("DEVIATING - Distance travelled : %d \n", distance_travelled);
        }
        if (movement_info.orientation == PARRALLEL){
            printf("On second phase (PARRALLEL) \n");
            left_motor_set_pos(0);
            while(!advance_until_clear(0,0,0,0) && movement_info.orientation == PARRALLEL) {
            printf("==============Creating new obstacle============== \n");
            movement_info.orientation += avoid_obstacle(1);
                    };
            movement_info.orientation = CONVERGING;
            printf("DEVIATING - Travel distance : %d \n", deviation_distance);
            printf("DEVIATING - Distance travelled : %d \n", distance_travelled);
            turn_to(-movement_info.turn_direction*movement_info.orientation*90);
        }

        else if (movement_info.orientation == CONVERGING){
            printf("On third phase (CONVERGING) \n");
            printf("CONVERGING 2.1 - Travel distance : %d \n", deviation_distance);
            printf("CONVERGING 2.1 - Distance travelled : %d \n", distance_travelled);

            left_motor_set_pos(0);
            while(!advance_until_clear(&distance_travelled,1,distance_travelled-deviation_distance,1) && movement_info.orientation == CONVERGING) {
            printf("==============Creating new obstacle============== \n");
            movement_info.orientation += avoid_obstacle(1);                         //Erreur au niveau de la sortie de boucle : type "orientation" fausse.
                    };
            printf("CONVERGING 2.2 - Travel distance : %d \n", deviation_distance);
            printf("CONVERGING 2.2 - Distance travelled : %d \n", distance_travelled);
            if (!(distance_travelled-deviation_distance)) {
                movement_info.on_right_track = YES;
                printf("changing state : %d :", movement_info.on_right_track);
                };
            
            if (!not_main_track||object_detection()) {
                turn_to(movement_info.turn_direction*movement_info.orientation*90);
                printf("==============Leaving Boucle===============");
                return 1;
                };
        }
    }
    printf("==============Leaving Boucle===============");
    return 0;

};

void set_turning_direction(){ //à revoir avec les capteurs à distance
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


