#ifndef __TIMERLEDSTESTING_H
#define __TIMERLEDSTESTING_H

void timer_clock_init (void);
void timer_start(void);
void timer_stop(void);
void timer_pwm_init (void);

// these are the LEDs on the STM32F4Discovery board
void board_leds_init (void);

/* the flashing green LED acts as a visual timing reference */
void flash_green_led_forever (void);

#endif	// __TIMERLEDSTESTING_H
