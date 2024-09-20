#include "cyBot_Scan.h"
#include "cyBot_uart.h"
#include "lcd.h"
#include <stdio.h>
#include <string.h>

void main() {

    cyBot_uart_init();
    cyBOT_init_Scan(0b0011);
    lcd_init();
    cyBOT_Scan_t scanData;

    int loop;
    int angle;
    char data[200];


    sprintf(data, "Degrees\t\tDistance (cm)\n\r");
       for (loop = 0; loop < strlen(data); loop++) {
           cyBot_sendByte(data[loop]);
       }

    for(angle = 0; angle <= 180; angle += 2) {
       cyBOT_Scan(angle, &scanData);
       sprintf(data, "%d\t\t%.2f\n\r", angle, scanData.sound_dist);
       for (loop = 0; loop < strlen(data); loop++) {
           cyBot_sendByte(data[loop]);
       }
    }
}