#ifndef MOOVEMENT_CONTROL_H
#define MOOVEMENT_CONTROL_H
typedef unsigned char   uint8_t;

void movement_init();
uint8_t object_detection();
uint8_t avoid_obstacle(uint8_t not_main_track);
void go_forward();

#endif