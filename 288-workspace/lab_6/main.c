/**
 * lab6_template_extra_help.c
 *
 * Description: This is file is meant for those that would like a little
 *              extra help with formatting their code.
 *
 */

#define _RESET 0
#define _PART1 0
#define _PART2 0
#define _PART3 0
#define _PART4 1
#define BUFFER_SIZE 20
#define NUM_SAMPLES 3
#define SCAN_DELAY_MS 1

#include "timer.h"
#include "lcd.h"
#include "uart.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "cyBot_Scan.h"
#include "open_interface.h"
#include "movement.h"

volatile  char uart_data;
volatile  char flag;

float distances[] = {9, 12, 15, 18, 21, 24, 27, 30, 33, 36, 39, 42, 45, 48, 50};
int ir_values[] = {1499, 1277, 1147, 1087, 987, 955, 919, 877, 842, 833, 803, 795, 779, 751, 731};


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
            float slope = (distances[i + 1] - distances[i]) / (ir_values[i + 1] - ir_values[i]);
            return distances[i] + slope * (raw_ir - ir_values[i]);
        }
    }
    return -1;
}

void perform_scan(cyBOT_Scan_t *scanData) {
    int angle;
    int i;
    int raw_ir_sum, ping_sum;
    float avg_ir, avg_ping;

    uart_sendStr("\nAngle  \tPING Distance (cm)  \tIR Distance (cm)\n");
    uart_sendStr("---------------------------------------------------\n");

    // Scan from 0 to 180 degrees
    for (angle = 0; angle <= 180; angle += 2) {  // Scan every 2 degrees
        raw_ir_sum = 0;
        ping_sum = 0;

        // Collect NUM_SAMPLES values for averaging
        for (i = 0; i < NUM_SAMPLES; i++) {
            cyBOT_Scan(angle, scanData);
            raw_ir_sum += scanData->IR_raw_val;
            ping_sum += scanData->sound_dist;

            // Delay between readings to slow down the scan
            timer_waitMillis(SCAN_DELAY_MS);
        }

        // Calculate the average for IR and ping values
        avg_ir = raw_ir_sum / (float)NUM_SAMPLES;
        avg_ping = ping_sum / (float)NUM_SAMPLES;

        // Display the averaged values on the LCD and send to PuTTY
        lcd_printf("Angle: %d\nIR: %.2f\nPING: %.2f cm", angle, avg_ir, avg_ping);

        char message[50];
        sprintf(message, "Angle: %d IR: %.2f PING: %.2f cm\r\n", angle, avg_ir, avg_ping);
        for (i = 0; i < strlen(message); i++) {
            uart_sendChar(message[i]);
        }
    }
}

void control_movement(oi_t *sensor_data, char command) {
    switch (command) {
    case 'a':
        lcd_printf("Turning Left");
        turn_counter_clockwise(sensor_data, 5);
        timer_waitMillis(500);  // Small delay after turning
        break;
    case 'd':
        lcd_printf("Turning Right");
        turn_clockwise(sensor_data, 5);
        timer_waitMillis(500);  // Small delay after turning
        break;
    default:
        break;
    }
}

void move_robot(oi_t *sensor_data, char *move_command) {
    char direction = move_command[strlen(move_command) - 1];  // Get the direction ('w' or 's')
    move_command[strlen(move_command) - 1] = '\0';            // Remove the direction from the string
    int distance = atoi(move_command);                        // Convert remaining part to integer distance

    if (direction == 'w') {
        lcd_printf("Moving Forward %d cm", distance);
        move_forward(sensor_data, distance);  // Move forward
    } else if (direction == 's') {
        lcd_printf("Moving Backward %d cm", distance);
        move_backwards(sensor_data, distance);  // Move backward
    }
}

int main(void)
{
    uart_init(115200);
    lcd_init();
    timer_init();
    uart_interrupt_init();
    cyBOT_init_Scan(0b0111);
    right_calibration_value = 343000;
    left_calibration_value = 1340500;


#if _PART1
    lcd_printf("UART Part 1 Test");

    char buffer[20];
    int count = 0;

    while (1)
    {
        char received = uart_receive();

        if (received == '\r' || count == 20)
        {
            buffer[count] = '\0';
            lcd_clear();
            lcd_printf("%s", buffer);
            count = 0;
        }
        else
        {
            buffer[count++] = received;
            lcd_printf("Char: %c\nCount: %d", received, count);
        }
    }

#elif _PART2
    lcd_printf("UART Echo Test");

    while (1)
    {
        char received = uart_receive();
        uart_sendChar(received);

        if (received == '\r')
        {
            uart_sendChar('\n');
        }

        lcd_printf("Received: %c", received);
    }

#elif _PART3
    lcd_printf("UART Interrupt Test");
    char last_displayed = '\0';

    while (1)
    {
        if (flag == 1)
        {
            flag = 0;

            if (uart_data != last_displayed)
            {
                lcd_clear();
                lcd_printf("Received: %c", uart_data);
                last_displayed = uart_data;
            }

            uart_sendChar(uart_data);

            if (uart_data == '\r')
            {
                uart_sendChar('\n');
            }
        }
    }


#elif _PART4
    oi_t *sensor_data = oi_alloc();
    oi_init(sensor_data);
    cyBOT_Scan_t scanData;

    lcd_printf("UART Manual Control and 180-degree scan");

    char command_buffer[10];  // Buffer to collect distance commands like '20w'
    int buffer_index = 0;

    while (1) {
        if (flag == 1) {  // Check for UART input
            flag = 0;

            // Append the received character to the buffer
            if (buffer_index < sizeof(command_buffer) - 1 && uart_data != '\r') {
                command_buffer[buffer_index++] = uart_data;
                command_buffer[buffer_index] = '\0';  // Null-terminate the string
            }

            // If a single character command is received ('a', 'd', 'm', 'q')
            if (uart_data == 'a' || uart_data == 'd') {
                control_movement(sensor_data, uart_data);
                buffer_index = 0;  // Reset the buffer after executing a command
            } else if (uart_data == 'm') {
                perform_scan(&scanData);
                buffer_index = 0;  // Reset the buffer after executing a command
            } else if (uart_data == 'q') {
                oi_free(sensor_data);
                uart_sendStr("Quitting...\r\n");
                break;
            } else if (uart_data == '\r') {  // Enter key pressed, indicating end of a command
                // Check if the command is a distance command like '20w' or '15s'
                if (buffer_index > 1) {
                    char direction = command_buffer[buffer_index - 1];
                    if ((direction == 'w' || direction == 's') && buffer_index > 1) {
                        move_robot(sensor_data, command_buffer);
                    } else {
                        uart_sendStr("Invalid command. Please use a valid format like '20w' or '15s'.\r\n");
                    }
                } else {
                    uart_sendStr("Invalid command. Please try again.\r\n");
                }
                buffer_index = 0;  // Reset the buffer after executing a command
                memset(command_buffer, 0, sizeof(command_buffer));  // Clear the buffer
            }

            // Echo the received character (for debugging purposes)
            if (uart_data != '\r') {
                uart_sendChar(uart_data);
            } else {
                uart_sendChar('\n');  // Send newline for Enter key
            }
        }
    }

#endif
    return 0;
}
