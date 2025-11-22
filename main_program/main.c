/**
 * integrated_main.c
 * 
 * INTEGRATED LINE FOLLOWING + OBSTACLE AVOIDANCE
 * 
 * Features:
 * - Line following using single IR sensor
 * - Obstacle detection with ultrasonic sensor
 * - Obstacle avoidance maneuvers
 * - State machine coordination
 * - Ready for barcode integration (future)
 * 
 * States:
 * 1. LINE_FOLLOWING - Normal line tracking
 * 2. OBSTACLE_DETECTED - Obstacle spotted, stop and scan
 * 3. OBSTACLE_AVOIDING - Execute avoidance maneuver
 * 4. RETURNING_TO_LINE - Getting back to line
 * 5. LINE_LOST - Line lost, search mode
 */

#include "pico/stdlib.h"
#include <stdio.h>
#include <math.h>
//State
#include "state_machine.h"

// Hardware drivers
#include "motor.h"
#include "encoder.h"
#include "imu.h"
#include "ir_sensor.h"
#include "ultrasonic.h"
#include "servo.h"
#include "obstacle_control.h"

// Control algorithms
#include "line_following.h"
#include "obstacle_scanner.h"
#include "avoidance_maneuver.h"

// Utilities
#include "timer_manager.h"
#include "telemetry.h"
#include "calibration.h"
#include "config.h"
#include "pin_definitions.h"

// ============================================================================
// SYSTEM STATE MACHINE
// ============================================================================



static SystemState current_state = STATE_IDLE;
static SystemState previous_state = STATE_IDLE;

// ============================================================================
// CONFIGURATION
// ============================================================================

// Obstacle detection
#define OBSTACLE_CHECK_DISTANCE_CM 20  // Check for obstacles at 20cm
#define OBSTACLE_CHECK_INTERVAL_MS 500 // Check every 500ms
#define CRITICAL_DISTANCE_CM 15        // Stop if obstacle within 15cm

// ============================================================================
// GLOBAL STATE
// ============================================================================

static IMU imu;
static uint32_t last_obstacle_check = 0;
static uint32_t state_entry_time = 0;

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================

static void init_hardware(void);
static void handle_returning_to_line(void);
static void handle_line_lost(void);
static const char* state_to_string(SystemState state);

SystemState get_current_state(void) {
    return current_state;
}

// ============================================================================
// HARDWARE INITIALIZATION
// ============================================================================

static void init_hardware(void) {
    printf("\n╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║     INTEGRATED LINE FOLLOWING + OBSTACLE AVOIDANCE           ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n\n");
    
    printf("Initializing hardware...\n");
    
    // Motors
    motor_init(M1A, M1B);
    motor_init(M2A, M2B);
    printf("  ✓ Motors\n");
    
    // Encoders
    encoder_init();
    printf("  ✓ Encoders\n");
    
    // IMU (struct-based for line following)
    imu_init(&imu);
    imu_calibrate(&imu);
    printf("  ✓ IMU (struct)\n");
    
    // IMU Helper (for avoidance maneuvers)
    imu_helper_init();
    printf("  ✓ IMU Helper\n");
    
    // IR Sensors
    ir_sensor_init();
    printf("  ✓ IR Sensors\n");
    
    // Ultrasonic
    ultrasonic_init(TRIG_PIN, ECHO_PIN);
    printf("  ✓ Ultrasonic\n");
    
    // Servo
    servo_init(SERVO_PIN);
    servo_set_angle(ANGLE_CENTER);
    printf("  ✓ Servo\n");
    
    // Calibration button
    calibration_init();
    printf("  ✓ Calibration Button (GP20)\n");
    
    // Line following controller
    line_following_init();
    printf("  ✓ Line Following Controller\n");
    
    // Obstacle systems
    scanner_init();
    avoidance_init();
    timer_manager_init();
    printf("  ✓ Obstacle Detection Systems\n");
    
    printf("\n✓ All hardware initialized\n\n");
}

// ============================================================================
// STATE MANAGEMENT
// ============================================================================

void change_state(SystemState new_state) {
    if (new_state != current_state) {
        previous_state = current_state;
        current_state = new_state;
        state_entry_time = to_ms_since_boot(get_absolute_time());
        
        printf("\n>>> STATE CHANGE: %s -> %s\n", 
               state_to_string(previous_state),
               state_to_string(new_state));
        
        // Publish state change to telemetry
        if (telemetry_is_connected()) {
            char msg[64];
            snprintf(msg, sizeof(msg), "State: %s", state_to_string(new_state));
            telemetry_publish_status(msg);
        }
    }
}

static const char* state_to_string(SystemState state) {
    switch (state) {
        case STATE_IDLE: return "IDLE";
        case STATE_LINE_FOLLOWING: return "LINE_FOLLOWING";
        case STATE_OBSTACLE_DETECTED: return "OBSTACLE_DETECTED";
        case STATE_OBSTACLE_SCANNING: return "OBSTACLE_SCANNING";
        case STATE_OBSTACLE_AVOIDING: return "OBSTACLE_AVOIDING";
        case STATE_RETURNING_TO_LINE: return "RETURNING_TO_LINE";
        case STATE_LINE_LOST: return "LINE_LOST";
        case STATE_STOPPED: return "STOPPED";
        default: return "UNKNOWN";
    }
}



static void handle_returning_to_line(void) {
    // After avoidance, we should be near the line
    // Use line following logic but with more tolerance
    
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    
    // Check if we found the line
    if (ir_line_detected()) {
        printf("\n[LINE] Line detected! Resuming normal following\n");
        change_state(STATE_LINE_FOLLOWING);
        return;
    }
    
    // Continue moving forward slowly to find line
    motor_drive(M1A, M1B, -30);
    motor_drive(M2A, M2B, -30);
    
    // Timeout after 3 seconds
    if (current_time - state_entry_time > 3000) {
        printf("\n[LINE] Could not find line - entering search mode\n");
        change_state(STATE_LINE_LOST);
    }
}

// ============================================================================
// LINE LOST HANDLING
// ============================================================================

static void handle_line_lost(void) {
    printf("[LINE] Line lost - searching...\n");
    
    // Simple search: move forward slowly
    motor_drive(M1A, M1B, -25);
    motor_drive(M2A, M2B, -25);
    
    // Check if line is found
    if (ir_line_detected()) {
        printf("[LINE] ✓ Line found!\n");
        change_state(STATE_LINE_FOLLOWING);
        return;
    }
    
    // Timeout after 5 seconds
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    if (current_time - state_entry_time > 5000) {
        printf("[LINE] Search timeout - stopping\n");
        change_state(STATE_STOPPED);
    }
}

// ============================================================================
// MAIN PROGRAM
// ============================================================================

int main() {
    // Initialize stdio
    stdio_init_all();
    sleep_ms(2000);
    
    // Initialize hardware
    init_hardware();
    
    // Optional: Initialize telemetry (comment out if not using)
    // printf("\nInitializing telemetry...\n");
    // if (telemetry_init(MQTT_BROKER_ADDRESS, MQTT_CLIENT_ID) == TELEMETRY_SUCCESS) {
    //     printf("✓ Telemetry connected\n");
    //     scanner_enable_telemetry();
    // }
    
    // Run calibration
    printf("\n═══════════════════════════════════════════════════════════════\n");
    printf("  IR SENSOR CALIBRATION\n");
    printf("  Press GP20 to start calibration\n");
    printf("═══════════════════════════════════════════════════════════════\n\n");
    
    while (!calibration_button_pressed()) {
        sleep_ms(10);
    }
    
    calibration_run_sequence();
    
    // Wait for start button
    printf("\n═══════════════════════════════════════════════════════════════\n");
    printf("  Press GP20 to start robot\n");
    printf("═══════════════════════════════════════════════════════════════\n\n");
    
    while (!calibration_button_pressed()) {
        sleep_ms(10);
    }
    sleep_ms(200);  // Debounce
    
    // Start in line following mode
    change_state(STATE_LINE_FOLLOWING);
    printf("\n>>> ROBOT STARTED <<<\n\n");
    
    // Main loop
    uint32_t last_update = to_ms_since_boot(get_absolute_time());
    bool running = true;
    
    while (running) {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        float dt = (current_time - last_update) / 1000.0f;
        
        // Clamp dt to reasonable range
        if (dt > 0.1f) dt = 0.02f;
        if (dt < 0.001f) dt = 0.001f;
        last_update = current_time;
        
        // Update IMU (both interfaces)
        imu_update(&imu);          // Struct-based for line following
        imu_helper_update();       // Helper for obstacle avoidance
        
        // Process telemetry
        if (telemetry_is_connected()) {
            telemetry_process();
        }
        
        // State machine
        switch (current_state) {
            case STATE_IDLE:
                // Do nothing
                break;
                
            case STATE_LINE_FOLLOWING:
                // Use the new integrated line following update function
                if (!line_following_control_update(current_time, dt)) {
                    // Line was lost
                    change_state(STATE_LINE_LOST);
                }
                
                // Periodically check for obstacles
                if (current_time - last_obstacle_check >= OBSTACLE_CHECK_INTERVAL_MS) {
                    if (check_for_obstacles()) {
                        handle_obstacle_detected();
                    }
                    last_obstacle_check = current_time;
                }
                break;
                
            case STATE_OBSTACLE_DETECTED:
                handle_obstacle_detected();
                break;
                
            case STATE_OBSTACLE_SCANNING:
                handle_obstacle_scanning();
                break;
                
            case STATE_OBSTACLE_AVOIDING:
                handle_obstacle_avoidance();
                break;
                
            case STATE_RETURNING_TO_LINE:
                handle_returning_to_line();
                break;
                
            case STATE_LINE_LOST:
                handle_line_lost();
                break;
                
            case STATE_STOPPED:
                // Stop all motors
                motor_stop(M1A, M1B);
                motor_stop(M2A, M2B);
                printf("[SYSTEM] Stopped - press GP20 to restart\n");
                
                if (calibration_button_pressed()) {
                    change_state(STATE_LINE_FOLLOWING);
                }
                break;
        }
        
        // Emergency stop button
        if (calibration_button_pressed() && current_state != STATE_STOPPED) {
            printf("\n>>> EMERGENCY STOP <<<\n");
            change_state(STATE_STOPPED);
        }
        
        sleep_ms(10);  // 100Hz loop
    }
    
    // Cleanup
    motor_stop(M1A, M1B);
    motor_stop(M2A, M2B);
    
    if (telemetry_is_connected()) {
        telemetry_cleanup();
    }
    
    printf("\nProgram ended.\n");
    return 0;
}