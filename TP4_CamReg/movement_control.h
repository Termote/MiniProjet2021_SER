#ifndef MOOVEMENT_CONTROL_H
#define MOOVEMENT_CONTROL_H
typedef unsigned char   uint8_t;

void movement_init();
uint8_t object_detection();
void avoid_obstacle();
void go_forward();
void turn_to(int angle);

#endif
