#include "cyBot_Scan.h"
#include "lcd.h"
#include "uart.h"
#include "timer.h"
#include "movement.h"
#include <stdio.h>

// Define constants
#define SCAN_ANGLE_STEP 2
#define IR_AVERAGE_COUNT 3
#define PING_AVERAGE_COUNT 5  // Number of PING measurements for averaging
#define DETECTION_THRESHOLD 10
#define TARGET_DISTANCE_CM 5
#define SENSOR_OFFSET_CM 7.0  // Distance between sensor and front of the CyBot
#define MOVE_DISTANCE_SHORT 5
#define MOVE_DISTANCE_LONG 15

typedef struct {
    int start_angle;
    int end_angle;
    float ping_distance;
} ObjectBoundary;

volatile char uart_data;
volatile char flag;

// Calibration data (global variables)
float distances[] = {9, 12, 15, 18, 21, 24, 27, 30, 33, 36, 39, 42, 45, 48, 50};
int ir_values[] = {1499, 1277, 1147, 1087, 987, 955, 919, 877, 842, 833, 803, 795, 779, 751, 731};

ObjectBoundary detected_objects[10];
int object_count = 0;

// Function to convert raw IR values to distances using linear interpolation
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
    return -1;  // If the value does not match any valid range
}

// Setup function to initialize the CyBot
void setup() {
    uart_init(115200);
    lcd_init();
    timer_init();
    cyBOT_init_Scan(0b0111);  // Enable servo, IR, and PING sensors

    // Set calibration values for your specific CyBot
    right_calibration_value = 343000; // Calibrated value for 0 degrees (right)
    left_calibration_value = 1340500; // Calibrated value for 180 degrees (left)
}

// Function to perform the IR scan and detect objects
void perform_180_ir_scan(cyBOT_Scan_t *scanData) {
    int angle;
    int raw_ir_sum;
    float ir_average;
    int in_object = 0;
    int start_angle = -1;

    uart_sendStr("\nIR Scan Results:\n");
    uart_sendStr("Angle\tIR Distance (cm)\n");
    uart_sendStr("--------------------------\n");

    for (angle = 0; angle <= 180; angle += SCAN_ANGLE_STEP) {
        raw_ir_sum = 0;

        // Take multiple IR measurements and sum them for averaging
        for (int i = 0; i < IR_AVERAGE_COUNT; i++) {
            cyBOT_Scan(angle, scanData);  // Scan at the specified angle
            raw_ir_sum += scanData->IR_raw_val;
            timer_waitMillis(50);  // Small delay between readings
        }

        // Calculate average IR distance
        ir_average = raw_ir_sum / (float)IR_AVERAGE_COUNT;
        float ir_distance = calculate_distance_linear(ir_average);  // Convert raw IR to distance

        // Display IR data to PuTTY
        char message[50];
        snprintf(message, sizeof(message), "Angle: %d\tIR Distance: %.2f cm\n", angle, ir_distance);
        uart_sendStr(message);

        // Detect objects based on IR distance
        if (ir_distance < 50.0 && !in_object) { // Detected an object start
            in_object = 1;
            start_angle = angle;
        } else if (ir_distance >= 50.0 && in_object) { // Detected an object end
            in_object = 0;
            detected_objects[object_count].start_angle = start_angle;
            detected_objects[object_count].end_angle = angle;
            object_count++;
        }
    }
}

// Refine detected objects using the PING sensor for more precise distance measurement
void refine_object_detection_with_ping(cyBOT_Scan_t *scanData) {
    for (int i = 0; i < object_count; i++) {
        int midpoint_angle = (detected_objects[i].start_angle + detected_objects[i].end_angle) / 2;

        // Point PING sensor to the midpoint and get distance
        cyBOT_Scan(midpoint_angle, scanData);
        float ping_distance = scanData->sound_dist - SENSOR_OFFSET_CM;  // Adjusting for sensor offset

        // Store refined PING distance
        detected_objects[i].ping_distance = ping_distance;

        // Display refined data to PuTTY
        char message[100];
        snprintf(message, sizeof(message), "Object %d: Start: %d°, End: %d°, PING Distance: %.2f cm\n",
                 i + 1,
                 detected_objects[i].start_angle,
                 detected_objects[i].end_angle,
                 ping_distance);
        uart_sendStr(message);
    }
}

// Function to navigate to the smallest object
void navigate_to_smallest_object(cyBOT_Scan_t *scanData, oi_t *sensor_data) {
    // Find the smallest object by width
    int smallest_index = 0;
    float smallest_width = detected_objects[0].end_angle - detected_objects[0].start_angle;

    for (int i = 1; i < object_count; i++) {
        float width = detected_objects[i].end_angle - detected_objects[i].start_angle;
        if (width < smallest_width) {
            smallest_width = width;
            smallest_index = i;
        }
    }

    // Get the target angle and distance of the smallest object
    int target_angle = (detected_objects[smallest_index].start_angle + detected_objects[smallest_index].end_angle) / 2;
    float ping_distance = detected_objects[smallest_index].ping_distance;

    char message[80];
    snprintf(message, sizeof(message), "Turning to Mid Angle: %d degrees\n", target_angle);
    uart_sendStr(message);
    lcd_printf("Turning to %d deg", target_angle);

    // Determine turning direction and turn
    if (target_angle < 90) {
        turn_clockwise(sensor_data, 90 - target_angle);
    } else {
        turn_counter_clockwise(sensor_data, target_angle - 90);
    }

    // Start moving towards the object until we are within the target distance
    while (ping_distance > TARGET_DISTANCE_CM) {
        int move_distance = (ping_distance > 10.0) ? MOVE_DISTANCE_LONG : MOVE_DISTANCE_SHORT;

        snprintf(message, sizeof(message), "Moving %d cm towards object\n", move_distance);
        uart_sendStr(message);
        move_forward(sensor_data, move_distance);

        // After moving, perform a scan from 70 to 110 degrees to find the closest point again
        int start_scan_angle = 70;
        int end_scan_angle = 110;
        float closest_distance = 1000;  // Initialize with a large value
        int closest_angle = target_angle;

        for (int angle = start_scan_angle; angle <= end_scan_angle; angle += SCAN_ANGLE_STEP) {
            float ping_sum = 0.0;

            for (int i = 0; i < PING_AVERAGE_COUNT; i++) {
                cyBOT_Scan(angle, scanData);
                ping_sum += scanData->sound_dist;

                // Delay for sensor stabilization
                timer_waitMillis(50);
            }

            float average_ping_distance = (ping_sum / PING_AVERAGE_COUNT) - SENSOR_OFFSET_CM;

            // Check if this is the closest object detected so far
            if (average_ping_distance < closest_distance) {
                closest_distance = average_ping_distance;
                closest_angle = angle;
            }

            snprintf(message, sizeof(message), "Scan Angle: %d, PING Distance: %.2f cm\n", angle, average_ping_distance);
            uart_sendStr(message);
        }

        // Update target angle and distance
        target_angle = closest_angle;
        ping_distance = closest_distance;

        // Adjust direction if necessary
        snprintf(message, sizeof(message), "Adjusting direction to Angle: %d degrees\n", target_angle);
        uart_sendStr(message);
        lcd_printf("Adjusting to %d deg", target_angle);

        if (target_angle < 90) {
            turn_clockwise(sensor_data, 90 - target_angle);
        } else if (target_angle > 90) {
            turn_counter_clockwise(sensor_data, target_angle - 90);
        }
    }

    // Stop once the CyBot is within the target distance
    snprintf(message, sizeof(message), "Arrived within %.2f cm of the object.\n", ping_distance);
    uart_sendStr(message);
    lcd_printf("Arrived at %.2f cm", ping_distance);
}

int main(void) {
    setup();

    cyBOT_Scan_t scanData;
    oi_t *sensor_data = oi_alloc();
    oi_init(sensor_data);

    uart_sendStr("Press 'm' to perform a 180-degree scan and navigate to the smallest object\n");

    while (1) {
        if (flag) {
            flag = 0;  // Reset the flag when data is received

            if (uart_data == 'm') {
                lcd_clear();
                lcd_printf("Performing Scan...");
                perform_180_ir_scan(&scanData);  // Perform IR scan
                refine_object_detection_with_ping(&scanData);  // Refine with PING
                navigate_to_smallest_object(&scanData, sensor_data);  // Navigate to the smallest object
                object_count = 0;  // Reset object count after each scan
            } else if (uart_data == 'q') {
                break;  // Quit command
            } else {
                uart_sendStr("Invalid command. Press 'm' to scan.\n");
            }
        }
    }

    oi_free(sensor_data);
    return 0;
}