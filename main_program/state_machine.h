#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

typedef enum {
    STATE_IDLE,
    STATE_LINE_FOLLOWING,
    STATE_OBSTACLE_DETECTED,
    STATE_OBSTACLE_SCANNING,
    STATE_OBSTACLE_AVOIDING,
    STATE_RETURNING_TO_LINE,
    STATE_LINE_LOST,
    STATE_STOPPED
} SystemState;

// Expose the state change function API
void change_state(SystemState new_state);
SystemState get_current_state(void);

#endif
