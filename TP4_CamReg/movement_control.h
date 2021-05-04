#ifndef MOOVEMENT_CONTROL_H
#define MOOVEMENT_CONTROL_H
typedef unsigned char   uint8_t;

void movement_init(proximity_msg_t* proximity;);
uint8_t object_detection();
void avoid_obstacle();
void go_forward();

#endif
