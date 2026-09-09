#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/sleep.h>


void setupWatchdog(void) {
    cli();  // disable interrupts globally (protects code from being interrupted)
    wdt_reset();  // reset the watchdog timer

    // MCUSR (MCU Status Register) delete the WDRF (Watchdog Reset Flag) bit
    MCUSR &= ~(1 << WDRF);

    // start the timed sequence by setting WDCE and WDE (opens configuration window for 4 clock cycles)
    WDTCR |= (1 << WDCE) | (1 << WDE);

    // write the new configuration immidiately into WDTCR (Watchdog Timer Control Register)
    // WDIE (Watchdog Interrupt Enable) = 1 -> enable interrupt mode (prevents system reset)
    // WDP3 & WDP0 = 1 -> set preescaler timeout period to exactly 8 seconds
    WDTCR = (1 << WDIE) | (1 << WDP3) | (1 << WDP0);
    sei();  // enable interrupts globally
}


extern volatile uint8_t watchdog_counter;  // declare the global variable defined in main.cpp

void deepSleep(uint8_t cycles) {
    watchdog_counter = 0;  // reset the watchdog counter

    // set the deepest possible sleep mode (Power-down mode)
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);

    // loop until the requested number of 8-second watchdog cycles has elapsed
    while (watchdog_counter < cycles) {
        sleep_enable();         // Enable the sleep circuit (allows the MCU to enter sleep mode)

        cli();                  // Disable interrupts globally
        sleep_bod_disable();    // Disable the Brown-Out Detector (BOD) to save power during sleep
        sei();                  // Enable interrupts globally

        sleep_cpu();            // Put the MCU to sleep (enters sleep mode and pauses execution right here)

        /*
        **********     AtTiny sleeps ... Wakes up after 8s -> runs ISR -> returns here     **********
        */

        sleep_disable();        // Disable sleep mode immidiately after waking up (prevents accidental re-entry into sleep mode)
    }
}