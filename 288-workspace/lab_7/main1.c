#include "cyBot_Scan.h"
#include "lcd.h"
#include "uart.h"
#include "timer.h"
#include "movement.h"
#include <stdio.h>

// Define constants
#define SCAN_ANGLE_STEP 2
#define IR_THRESHOLD 45   // Distance threshold for object detection in cm (used for detecting objects closer than the background)
#define NUM_SAMPLES 3      // Number of samples for averaging
#define PING_AVERAGE_COUNT 5 // Number of PING measurements for averaging during refinement

typedef struct {
    int start_angle;
    int end_angle;
    float ping_distance;
} ObjectInfo;

ObjectInfo detected_objects[10];
int object_count = 0;

volatile char uart_data;  // UART received data
volatile char flag = 0;   // Flag to indicate UART data is available

// Calibration data (global variables)
float distances[] = {9, 12, 15, 18, 21, 24, 27, 30, 33, 36, 39, 42, 45, 48, 50};
int ir_values[] = {1499, 1277, 1147, 1087, 987, 955, 919, 877, 842, 833, 803, 795, 779, 751, 731};

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
    uart_interrupt_init();
    cyBOT_init_Scan(0b0111);  // Enable servo, IR, and PING sensors

    // Set calibration values for your specific CyBot
    right_calibration_value = 343000; // Calibrated for 0 degrees (right)
    left_calibration_value = 1340500; // Calibrated for 180 degrees (left)
}

// Perform a 180-degree scan using the IR sensor to detect objects
void perform_180_ir_scan(cyBOT_Scan_t *scanData) {
    int angle;      // Variable declared outside of for loop
    int i;          // Variable declared outside of for loop
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
        for (i = 0; i < NUM_SAMPLES; i++) {
            cyBOT_Scan(angle, scanData);  // Scan at the specified angle
            raw_ir_sum += scanData->IR_raw_val;
            timer_waitMillis(50);  // Small delay between readings
        }

        // Calculate average IR distance
        ir_average = raw_ir_sum / (float)NUM_SAMPLES;
        float ir_distance = calculate_distance_linear(ir_average); // Convert raw IR to distance

        // Display IR data to PuTTY
        char message[50];
        snprintf(message, sizeof(message), "Angle: %d\tIR Distance: %.2f cm\n", angle, ir_distance);
        uart_sendStr(message);

        // Detect objects based on IR distance
        if (ir_distance < IR_THRESHOLD && !in_object) {
            // Detected the start of an object
            in_object = 1;
            start_angle = angle;
        } else if (ir_distance >= IR_THRESHOLD && in_object) {
            // Detected the end of an object
            in_object = 0;
            detected_objects[object_count].start_angle = start_angle;
            detected_objects[object_count].end_angle = angle;
            object_count++;
        }
    }

    // Display start and end angles of each detected object
    uart_sendStr("\nDetected Objects:\n");
    uart_sendStr("Object\tStart Angle\tEnd Angle\n");
    uart_sendStr("--------------------------------\n");
    for (i = 0; i < object_count; i++) {
        char message[50];
        snprintf(message, sizeof(message), "Object %d\t%d\t\t%d\n", i + 1, detected_objects[i].start_angle, detected_objects[i].end_angle);
        uart_sendStr(message);
    }
}

// Refine detected objects using the PING sensor for more precise distance measurement
void refine_object_detection_with_ping(cyBOT_Scan_t *scanData) {
    int i, j;  // Variables declared outside of for loop

    uart_sendStr("\nRefined PING Measurement Results:\n");
    uart_sendStr("Object\tMid Angle\tPING Distance (cm)\n");
    uart_sendStr("--------------------------------------\n");

    for (i = 0; i < object_count; i++) {
        // Average the PING readings for a better estimate
        float ping_sum = 0.0;
        int angle_sum = 0;

        // Instead of scanning just the midpoint, scan multiple angles within the object and average
        for (j = detected_objects[i].start_angle; j <= detected_objects[i].end_angle; j += 2) {
            cyBOT_Scan(j, scanData);
            ping_sum += scanData->sound_dist;
            angle_sum += j;
            timer_waitMillis(100);  // Allow time for servo movement and sensor stabilization
        }

        int avg_angle = angle_sum / ((detected_objects[i].end_angle - detected_objects[i].start_angle) / 2 + 1);
        float avg_ping = ping_sum / ((detected_objects[i].end_angle - detected_objects[i].start_angle) / 2 + 1);

        detected_objects[i].ping_distance = avg_ping;

        // Send the averaged PING measurement to PuTTY
        char message[100];
        snprintf(message, sizeof(message), "Object %d\tAngle: %d\tPING Distance: %.2f cm\n", i + 1, avg_angle, avg_ping);
        uart_sendStr(message);
    }

    // Now point the servo to the smallest object found after PING refinement
    display_smallest_object(scanData);
}

// Display the smallest detected object and point the servo at it
void display_smallest_object(cyBOT_Scan_t *scanData) {
    if (object_count == 0) {
        uart_sendStr("\nNo objects detected.\n");
        return;
    }

    int i;          // Variable declared outside of for loop
    int smallest_index = 0;
    float smallest_width = detected_objects[0].end_angle - detected_objects[0].start_angle;

    // Find the smallest object
    for (i = 1; i < object_count; i++) {
        float width = detected_objects[i].end_angle - detected_objects[i].start_angle;
        if (width < smallest_width) {
            smallest_width = width;
            smallest_index = i;
        }
    }

    // Display the smallest object
    uart_sendStr("\nSmallest object detected:\n");
    char message[100];
    snprintf(message, sizeof(message),
             "Start Angle: %d, End Angle: %d, PING Distance: %.2f cm\n",
             detected_objects[smallest_index].start_angle,
             detected_objects[smallest_index].end_angle,
             detected_objects[smallest_index].ping_distance);
    uart_sendStr(message);

    // Point the servo at the midpoint of the smallest object
    int mid_angle = (detected_objects[smallest_index].start_angle + detected_objects[smallest_index].end_angle) / 2;
    cyBOT_Scan(mid_angle, scanData);  // Move to the midpoint of the smallest object
    char debug_msg[50];
    snprintf(debug_msg, sizeof(debug_msg), "\nMoving to Mid Angle: %d degrees\n", mid_angle);
    uart_sendStr(debug_msg);
}

// Main function to perform scanning and send results to PuTTY
int main(void) {
    // Initialize all components
    setup();

    cyBOT_Scan_t scanData;

    uart_sendStr("Press 'm' to perform a 180-degree scan and refine object detection\n");

    while (1) {
        if (flag) {
            flag = 0;  // Reset the flag when data is received

            if (uart_data == 'm') {
                lcd_clear();
                lcd_printf("Performing Scan...");

                object_count = 0;  // Reset object count before each scan

                perform_180_ir_scan(&scanData);  // Perform IR scan
                refine_object_detection_with_ping(&scanData);  // Refine object detection with PING

                uart_sendStr("\nPress 'm' to scan again or 'q' to quit.\n");
            } else {
                uart_sendStr("Invalid command. Press 'm' to scan.\n");
            }
        }
    }

    return 0;
}
