#include "sensors/proximity.h"

#ifndef OBSTACLE_AVOID
#define OBSTACLE_AVOID
typedef unsigned char   uint8_t;

void movement_init(proximity_msg_t* proximity;);
uint8_t object_detection();
void avoid_obstacle();
void go_forward();
uint8_t status_on_front();

#endif
