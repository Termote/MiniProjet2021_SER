#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

float get_distance_cm(void);
uint16_t get_line_position(void);
void process_image_start(void);
void set_target_reached(uint8_t value);
uint8_t get_target_reached(void);
uint8_t get_avoiding_obstacle(void);
void set_avoiding_obstacle (uint8_t value);
uint8_t get_line_not_found (void);

#endif /* PROCESS_IMAGE_H */
