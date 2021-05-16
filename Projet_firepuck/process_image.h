#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

float get_distance_cm(void);
uint16_t get_line_position(void);
void process_image_start(void);

//constants for the different parts of the pi regulator and process image

#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			40
#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			2
#define PXTOCM					600.0f //experimental value
#define GOAL_DISTANCE 			5.0f
#define MAX_DISTANCE 			25.0f
#define ERROR_THRESHOLD			0.1f	//[cm] because of the noise of the camera
#define KP						600.0f  //experimental value
#define KI 						3.5f	//experimental value
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)

#endif /* PROCESS_IMAGE_H */
