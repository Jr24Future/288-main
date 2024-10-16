#include <stdio.h>
#include <stdlib.h>
#include "button.h"
#include "timer.h"
#include "lcd.h"
#include "cyBot_uart.h"
#include "cyBot_Scan.h"
#include "open_interface.h"
#include "movement.h"

#warning "Possible unimplemented functions"
#define REPLACEME 0

extern volatile int button_event;
extern volatile int button_num;

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

void control_movement(oi_t *sensor_data, char command) {
    switch (command) {
        case 'w':   // Move forward
            move_forward(sensor_data, 50);  // Move forward 50 cm
            break;
        case 's':   // Move backward
            move_backwards(sensor_data, 25);  // Move backward 25 cm
            break;
        case 'a':   // Turn left
            turn_counter_clockwise(sensor_data, 15);  // Turn left 15 degrees
            break;
        case 'd':   // Turn right
            turn_clockwise(sensor_data, 15);  // Turn right 15 degrees
            break;
        default:
            break;
    }
}

int main(void) {
    button_init();
    init_button_interrupts();
    lcd_init();
    cyBot_uart_init();        //uncomment it for part1
    //cyBot_uart_init_clean();    //comment it out for part1
    cyBOT_init_Scan(0b0111);    //part3

    //right_calibration_value = 337750;
    //left_calibration_value = 1351000;

    cyBOT_Scan_t scanData;
    int raw_ir;
    int i = 0;
    int loop = 0;
    oi_t *sensor_data = oi_alloc();
    oi_init(sensor_data);
    char command;


    // Complete this code for configuring the  (GPIO) part of UART initialization       comment this part out for part1
      SYSCTL_RCGCGPIO_R |= 0x02;
      timer_waitMillis(1);
      GPIO_PORTB_AFSEL_R |= 0x03;
      GPIO_PORTB_PCTL_R &= ~0x000000FF;
      GPIO_PORTB_PCTL_R |= 0x00000011;
      GPIO_PORTB_DEN_R |= 0x03;
      GPIO_PORTB_DIR_R &= ~0x01;
      GPIO_PORTB_DIR_R |= 0x02;

     cyBot_uart_init_last_half();


     char startup_message[] = "Waiting for button presses...\r\n";
                                                                  //part1
         for (loop; loop < strlen(startup_message); loop++) {
             cyBot_sendByte(startup_message[loop]);  // Send startup message to PuTTY
         }


     while (1) {

        /* //part4w
         command = cyBot_getByte_blocking();
         lcd_printf("Command: %c", command);

         if (command == 'w' || command == 'a' || command == 's' || command == 'd') {
             control_movement(sensor_data, command);
         }

         if (command == 'q') {
             oi_free(sensor_data);
             break;
         }*/


            /* // Part 3: IR sensor and distance calibration
             cyBOT_Scan(90, &scanData);
             raw_ir = scanData.IR_raw_val;

             float distance = calculate_distance_linear(raw_ir);
             lcd_printf("Raw IR: %d\nDist: %.2f cm", raw_ir, distance);

             char message[50];
             sprintf(message, "Raw IR Value: %d\r\n", raw_ir);
             for (i; i < strlen(message); i++) {
                 cyBot_sendByte(message[i]);
             }*/

             // Handle button interrupts (part 1)
             if (button_event) {
                 lcd_printf("Button %d pressed", button_num);
                 char button_message[50];
                 sprintf(button_message, "Button %d pressed\r\n", button_num);
                 for (i; i < strlen(button_message); i++) {
                     cyBot_sendByte(button_message[i]);
                 }
                 button_event = 0;
             }

             timer_waitMillis(2000);
         }

         return 0;
     }



/*#include "cyBot_Scan.h"
#include "cyBot_uart.h"
#include "Timer.h"
#include "lcd.h"
#include <stdio.h>
#include <string.h>
void main() {

    timer_init();
    lcd_init();
    cyBot_uart_init();
    cyBOT_init_Scan(0b0011);

    cyBOT_Scan_t scanData;
    cyBOT_SERVO_cal();

}*/
