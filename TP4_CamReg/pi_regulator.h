#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

//start the PI regulator thread
void pi_regulator_start(void);
int16_t pi_regulator(float distance, float goal);

#endif /* PI_REGULATOR_H */
