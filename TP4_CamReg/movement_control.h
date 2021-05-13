

#ifndef MOOVEMENT_CONTROL_H
#define MOOVEMENT_CONTROL_H
typedef unsigned char   uint8_t;




void movement_init(void);
uint8_t object_detection(void);
void avoid_obstacle(void);
void go_forward(void);

#endif
