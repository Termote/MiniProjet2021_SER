#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "movement_control.h"

int main() {
  enum {
        NORTH = 0,
        EAST = 1,
        SOUTH = 2,
        WEST = 3
    } orientation;
    while (orientation < 8)
    {
        orientation += 1;
        printf("%d \n", orientation);
    }
   return 0;
}