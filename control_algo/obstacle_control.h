#ifndef OBSTACLE_CONTROL_H
#define OBSTACLE_CONTROL_H


#include <stdbool.h>
#include "main_program/state_machine.h"


bool check_for_obstacles(void);
void handle_obstacle_detected(void);
void handle_obstacle_scanning(void);
void handle_obstacle_avoidance(void);

#endif
