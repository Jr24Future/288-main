#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include "Timer.h"

void ping_init(void);
void ping_trigger(void);
void ping_init_timer(void);
void ping_measure(void);

int main(void) {
    ping_init();
    ping_init_timer();

    while (1) {
        ping_measure();
        // lcd_printf("Pulse Width: %d", pulse_width);

        timer_waitMillis(100);
    }

    return 0;
}
