#include "obstacle_control.h"
#include "pico/stdlib.h"
#include <stdio.h>

// dependencies
#include "motor.h"
#include "servo.h"
#include "ultrasonic.h"
#include "obstacle_scanner.h"
#include "avoidance_maneuver.h"
#include "telemetry.h"
#include "config.h"
#include "pin_definitions.h"

#include "state_machine.h"

#define OBSTACLE_CHECK_DISTANCE_CM 20
#define CRITICAL_DISTANCE_CM 15
#define OBSTACLE_CHECK_INTERVAL_MS 500


// ============================================================================
// OBSTACLE FUNCTIONS IMPLEMENTATION
// ============================================================================
// ============================================================================
// OBSTACLE DETECTION
// ============================================================================

bool check_for_obstacles(void) {
    // Point servo forward
    servo_set_angle(ANGLE_CENTER);
    sleep_ms(100);
    
    // Read distance
    uint64_t distance;
    int status = ultrasonic_get_distance(TRIG_PIN, ECHO_PIN, &distance);
    
    if (status == SUCCESS) {
        printf("[OBSTACLE] Forward distance: %llu cm", distance);
        
        if (distance <= OBSTACLE_CHECK_DISTANCE_CM) {
            printf(" ⚠️  OBSTACLE DETECTED!\n");
            return true;
        } else {
            printf(" ✓\n");
            return false;
        }
    } else {
        printf("[OBSTACLE] Sensor error\n");
        return false;
    }
}

// ============================================================================
// OBSTACLE HANDLING
// ============================================================================

void handle_obstacle_detected(void) {
    printf("\n[OBSTACLE] Obstacle detected - stopping motors\n");
    
    // Stop motors
    motor_stop(M1A, M1B);
    motor_stop(M2A, M2B);
    
    // Move to scanning state
    change_state(STATE_OBSTACLE_SCANNING);
}

void handle_obstacle_scanning(void) {
    printf("\n[OBSTACLE] Performing scan...\n");
    
    // Perform scan
    ScanResult result = scanner_perform_scan();
    scanner_print_results(result);
    
    // Determine avoidance direction
    AvoidanceDirection direction = scanner_get_best_avoidance_direction(result);
    
    if (direction == AVOID_NONE) {
        printf("[OBSTACLE] No clear path - stopping\n");
        change_state(STATE_STOPPED);
        return;
    }
    
    // Start avoidance maneuver
    if (avoidance_start(direction)) {
        change_state(STATE_OBSTACLE_AVOIDING);
    } else {
        printf("[OBSTACLE] Failed to start avoidance\n");
        change_state(STATE_STOPPED);
    }
}

void handle_obstacle_avoidance(void) {
    // Update avoidance maneuver
    AvoidanceState avoid_state = avoidance_update();
    
    // Check if complete
    if (avoidance_is_complete()) {
        if (avoidance_was_successful()) {
            printf("\n[OBSTACLE] Avoidance complete - returning to line following\n");
            avoidance_reset();
            change_state(STATE_RETURNING_TO_LINE);
        } else {
            printf("\n[OBSTACLE] Avoidance failed\n");
            change_state(STATE_STOPPED);
        }
    }
}