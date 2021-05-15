#ifndef OBSTACLE_AVOID
#define OBSTACLE_AVOID

typedef unsigned char   uint8_t;


#define NSTEP_ONE_TURN              1000    // number of step for 1 turn of the motor
#define CORRECTION_FACTOR           1.05    // correct the angle of rotation to be more precise
#define WHEEL_PERIMETER             13      // cm
#define WHEEL_DISTANCE              5.35f   //cm
#define PERIMETER_EPUCK             (3.14 * WHEEL_DISTANCE)     //PI * Wheel distance
#define EPUCK_RADIUS                40      //mm
#define SELECTOR_OFFSET 	        90     //angle offset due to position of 0 on the selector wheel compared to the front
#define SELECTOR_MAX		        16      //maximum value for the selector
#define FULL_PERIMETER_DEG          360.0f     //Degrees needed for the full perimeter
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


void movement_init(void);
uint8_t object_detection(void);
void avoid_obstacle(void);
void go_forward(void);
uint8_t status_on_front(void);

#endif
