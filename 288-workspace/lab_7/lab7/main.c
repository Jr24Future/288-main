#include "cyBot_Scan.h"
#include "lcd.h"
#include "uart.h"
#include "timer.h"
#include "movement.h"
#include "open_interface.h"
#include <stdio.h>

// Define constants
#define SCAN_ANGLE_STEP 2
#define IR_THRESHOLD 45
#define NUM_SAMPLES 3
#define PING_AVERAGE_COUNT 5
#define TARGET_DISTANCE_CM 5

typedef struct {
    int start_angle;
    int end_angle;
    float ping_distance;
} ObjectInfo;

ObjectInfo detected_objects[10];
int object_count = 0;

volatile char uart_data;
volatile char flag = 0;

typedef enum { MANUAL, AUTONOMOUS_SCAN, AUTONOMOUS_TURN, AUTONOMOUS_MOVE } Mode;
Mode current_mode = MANUAL;

float distances[] = {9, 12, 15, 18, 21, 24, 27, 30, 33, 36, 39, 42, 45, 48, 50};
int ir_values[] = {1499, 1277, 1147, 1087, 987, 955, 919, 877, 842, 833, 803, 795, 779, 751, 731};

// Function prototypes
float calculate_distance_linear(int raw_ir);
void setup();
void perform_180_ir_scan(cyBOT_Scan_t *scanData);
void refine_object_detection_with_ping(cyBOT_Scan_t *scanData);
void display_smallest_object();
void navigate_to_smallest_object(cyBOT_Scan_t *scanData, oi_t *sensor_data, int smallest_index);
void control_movement(oi_t *sensor_data, char command);
void handle_bump_sensor(oi_t *sensor_data);

float calculate_distance_linear(int raw_ir) {
    int i;
    if (raw_ir >= ir_values[0]) {
        return distances[0];
    }
    if (raw_ir <= ir_values[14]) {
        return distances[14];
    }
    for (i = 0; i < 14; i++) {
        if (raw_ir <= ir_values[i] && raw_ir > ir_values[i + 1]) {
            float slope = (distances[i + 1] - distances[i]) / (float)(ir_values[i + 1] - ir_values[i]);
            return distances[i] + slope * (raw_ir - ir_values[i]);
        }
    }
    return -1;
}

void setup() {
    uart_init(115200);
    lcd_init();
    timer_init();
    uart_interrupt_init();
    cyBOT_init_Scan(0b0111);

    right_calibration_value = 343000;
    left_calibration_value = 1340500;
}

void control_movement(oi_t *sensor_data, char command) {
    switch (command) {
        case 'w':
            move_forward_with_bump(sensor_data, 5); // Move forward by 10 cm, handle bump
            break;
        case 'a':
            turn_counter_clockwise(sensor_data, 15); // Turn left by 15 degrees
            break;
        case 's':
            move_backwards(sensor_data, 10); // Move backward by 10 cm
            break;
        case 'd':
            turn_clockwise(sensor_data, 15); // Turn right by 15 degrees
            break;
        default:
            uart_sendStr("Invalid movement command.\n");
            break;
    }
}

void perform_180_ir_scan(cyBOT_Scan_t *scanData) {
    int angle;
    int i;
    int raw_ir_sum;
    float ir_average;
    int in_object = 0;
    int start_angle = -1;

    uart_sendStr("\nIR Scan Results:\n");
    uart_sendStr("Angle\tIR Distance (cm)\n");
    uart_sendStr("--------------------------\n");

    for (angle = 0; angle <= 180; angle += SCAN_ANGLE_STEP) {
        raw_ir_sum = 0;

        for (i = 0; i < NUM_SAMPLES; i++) {
            cyBOT_Scan(angle, scanData);
            raw_ir_sum += scanData->IR_raw_val;
            timer_waitMillis(50);
        }

        ir_average = raw_ir_sum / (float)NUM_SAMPLES;
        float ir_distance = calculate_distance_linear(ir_average);

        char message[50];
        snprintf(message, sizeof(message), "Angle: %d\tIR Distance: %.2f cm\n", angle, ir_distance);
        uart_sendStr(message);

        if (ir_distance < IR_THRESHOLD && !in_object) {
            in_object = 1;
            start_angle = angle;
        } else if (ir_distance >= IR_THRESHOLD && in_object) {
            in_object = 0;
            detected_objects[object_count].start_angle = start_angle;
            detected_objects[object_count].end_angle = angle;
            object_count++;
        }
    }

    uart_sendStr("\nDetected Objects:\n");
    uart_sendStr("Object\tStart Angle\tEnd Angle\n");
    uart_sendStr("--------------------------------\n");
    for (i = 0; i < object_count; i++) {
        char message[50];
        snprintf(message, sizeof(message), "Object %d\t%d\t\t%d\n", i + 1, detected_objects[i].start_angle, detected_objects[i].end_angle);
        uart_sendStr(message);
    }
}

void refine_object_detection_with_ping(cyBOT_Scan_t *scanData) {
    int i, j;

    uart_sendStr("\nRefined PING Measurement Results:\n");
    uart_sendStr("Object\tMid Angle\tPING Distance (cm)\n");
    uart_sendStr("--------------------------------------\n");

    for (i = 0; i < object_count; i++) {
        int mid_angle = (detected_objects[i].start_angle + detected_objects[i].end_angle) / 2;
        float ping_sum = 0.0;

        for (j = 0; j < PING_AVERAGE_COUNT; j++) {
            cyBOT_Scan(mid_angle, scanData);
            ping_sum += scanData->sound_dist;
            timer_waitMillis(50);
        }

        detected_objects[i].ping_distance = ping_sum / PING_AVERAGE_COUNT;

        char message[100];
        snprintf(message, sizeof(message), "Object %d\tAngle: %d\tPING Distance: %.2f cm\n", i + 1, mid_angle, detected_objects[i].ping_distance);
        uart_sendStr(message);
    }

    display_smallest_object();
}

void display_smallest_object() {
    if (object_count == 0) {
        uart_sendStr("\nNo objects detected.\n");
        return;
    }

    int i;
    int smallest_index = 0;
    float smallest_width = detected_objects[0].end_angle - detected_objects[0].start_angle;

    for (i = 1; i < object_count; i++) {
        float width = detected_objects[i].end_angle - detected_objects[i].start_angle;
        if (width < smallest_width) {
            smallest_width = width;
            smallest_index = i;
        }
    }

    uart_sendStr("\nSmallest object detected:\n");
    char message[100];
    snprintf(message, sizeof(message),
             "Start Angle: %d, End Angle: %d, PING Distance: %.2f cm\n",
             detected_objects[smallest_index].start_angle,
             detected_objects[smallest_index].end_angle,
             detected_objects[smallest_index].ping_distance);
    uart_sendStr(message);

    // Set next mode to turn towards the smallest object
    current_mode = AUTONOMOUS_TURN;
}

void navigate_to_smallest_object(cyBOT_Scan_t *scanData, oi_t *sensor_data, int smallest_index) {
    if (smallest_index < 0 || smallest_index >= object_count) {
        uart_sendStr("Invalid smallest index. Cannot navigate.\n");
        return;
    }

    // Properly calculate the target angle
    int target_angle = (detected_objects[smallest_index].start_angle + detected_objects[smallest_index].end_angle) / 2;

    // Ensure the target angle is within a valid range (0 to 180 degrees)
    if (target_angle < 0 || target_angle > 180) {
        char error_message[80];
        snprintf(error_message, sizeof(error_message), "Invalid calculated angle: %d degrees. Aborting navigation.\n", target_angle);
        uart_sendStr(error_message);
        return;
    }

    float ping_distance = detected_objects[smallest_index].ping_distance - 7.0; // Adjusting for sensor placement
    char message[80];

    // Determine turning direction based on the target angle
    if (current_mode == AUTONOMOUS_TURN) {
        snprintf(message, sizeof(message), "Turning to Mid Angle: %d degrees\n", target_angle);
        uart_sendStr(message);
        lcd_printf("Turning to %d deg", target_angle);

        if (target_angle < 90) {
            turn_clockwise(sensor_data, 90 - target_angle);  // Turn clockwise if the target is on the right
        } else {
            turn_counter_clockwise(sensor_data, target_angle - 90);  // Turn counter-clockwise if the target is on the left
        }

        // Set next mode to move towards the target
        current_mode = AUTONOMOUS_MOVE;
    } else if (current_mode == AUTONOMOUS_MOVE) {
        snprintf(message, sizeof(message), "Moving towards object at %d degrees\n", target_angle);
        uart_sendStr(message);
        lcd_printf("Moving to %d cm", (int)ping_distance);

        if (ping_distance > TARGET_DISTANCE_CM) {
            int move_distance = (ping_distance > 12.0) ? 15 : 5;
            move_forward_with_bump(sensor_data, move_distance);

            // After each movement, update current mode to scan again
            current_mode = AUTONOMOUS_SCAN;
        } else {
            uart_sendStr("Arrived within target distance of the smallest object.\n");
            lcd_printf("Arrived at %.2f cm", ping_distance);
            current_mode = MANUAL;  // Stop after reaching the object
        }
    }
}

void handle_bump_sensor(oi_t *sensor_data) {
    oi_update(sensor_data);
    if (sensor_data->bumpLeft) {
        handle_bump(sensor_data, 'L');
    } else if (sensor_data->bumpRight) {
        handle_bump(sensor_data, 'R');
    }
}


int main(void) {
    setup();

    cyBOT_Scan_t scanData;
    oi_t *sensor_data = oi_alloc();
    oi_init(sensor_data);
    int i;

    uart_sendStr("Press 't' to toggle modes (Manual/Autonomous), 'h' to proceed in autonomous mode, or 'q' to quit.\n");

    int smallest_index = -1;  // Initialize smallest index to -1 for autonomous mode

    while (1) {
        // Read bump sensor status and handle bumps if any
        oi_update(sensor_data);
        if (sensor_data->bumpLeft || sensor_data->bumpRight) {
            // Handle bump sensor activation and stop autonomous actions
            if (sensor_data->bumpLeft) {
                handle_bump(sensor_data, 'L');
            } else if (sensor_data->bumpRight) {
                handle_bump(sensor_data, 'R');
            }
            continue; // Skip the rest of the loop to handle the bump properly and prompt the user
        }

        if (flag) {
            flag = 0;

            if (uart_data == 't') {
                if (current_mode == MANUAL) {
                    current_mode = AUTONOMOUS_SCAN;
                    uart_sendStr("Switched to Autonomous Mode. Press 'h' to start scanning and moving.\n");
                } else {
                    current_mode = MANUAL;
                    uart_sendStr("Switched to Manual Mode. Use 'w', 'a', 's', 'd' to control the CyBot.\n");
                }
            } else if (current_mode == MANUAL) {
                control_movement(sensor_data, uart_data);
            } else if (current_mode == AUTONOMOUS_SCAN && uart_data == 'h') {
                lcd_clear();
                lcd_printf("Performing Scan...");
                object_count = 0;

                perform_180_ir_scan(&scanData);
                refine_object_detection_with_ping(&scanData);

                if (object_count > 0) {
                    // Find the smallest object
                    smallest_index = 0;
                    float smallest_width = detected_objects[0].end_angle - detected_objects[0].start_angle;

                    for (i = 1; i < object_count; i++) {
                        float width = detected_objects[i].end_angle - detected_objects[i].start_angle;
                        if (width < smallest_width) {
                            smallest_width = width;
                            smallest_index = i;
                        }
                    }

                    display_smallest_object();

                    uart_sendStr("\nPress 'h' again to move towards the smallest object.\n");
                    current_mode = AUTONOMOUS_TURN;
                } else {
                    uart_sendStr("No objects detected. Press 'h' to scan again.\n");
                }
            } else if ((current_mode == AUTONOMOUS_TURN || current_mode == AUTONOMOUS_MOVE) && uart_data == 'h') {
                navigate_to_smallest_object(&scanData, sensor_data, smallest_index);
            } else if (uart_data == 'q') {
                uart_sendStr("Quitting the application.\n");
                oi_free(sensor_data);
                break;
            } else {
                uart_sendStr("Invalid command. Press 't' to toggle modes, 'h' to proceed, or 'q' to quit.\n");
            }
        }
    }

    return 0;
}
