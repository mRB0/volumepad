#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <stdint.h>

#include "usb_keyboard.h"

// Multimedia keys aren't listed in usb_keyboard.h.
// 
// These are from usb_hid_usages.txt, from
// http://www.freebsddiary.org/APC/usb_hid_usages
// 
// These keys don't work on Windows, but they work on Mac and
// apparently Linux.  It looks like Windows expects media keys in the
// 12 (0x0C) usage page (not 0x07), but usb_keyboard doesn't have any
// obvious way of specifying a page with a request (although it does
// mention 0x07 as the key codes usage page on line 116).
//
// Translate.pdf is also included, from:
//
// http://download.microsoft.com/download/1/6/1/161ba512-40e2-4cc9-843a-923143f3456c/translate.pdf
// mirror: http://www.hiemalis.org/~keiji/PC/scancode-translate.pdf
//
// Hopefully these will work on the Nexus 7.

#define KEY_VOLUME_UP 0x80
#define KEY_VOLUME_DOWN 0x81
#define KEY_VOLUME_MUTE 0x7f

struct debounce_state {
    uint8_t state; // pin state
    uint8_t count; // how many ticks it has been in this state
};

typedef enum {
    DirectionCCW,
    DirectionCW
} Direction;

#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))
#define LED_CONFIG	(DDRD |= (1<<6))
#define LED_OFF		(PORTD &= ~(1<<6))
#define LED_ON		(PORTD |= (1<<6))

// Specify 0 to not send a key when that button is pressed.
static uint8_t const SwitchKeyMap[7] = {
    KEY_2, // PORTB0 = S2
    0,     // PORTB1 = A (dial)
    KEY_1, // PORTB2 = S1
    KEY_5, // PORTB3 = S5
    KEY_4, // PORTB4 = S4
    0,     // PORTB5 = B (dial)
    KEY_3, // PORTB6 = S3
};

static uint8_t const DialA = 1; // PORTB2
static uint8_t const DialB = 5; // PORTB5


// Timer 0 clock select (prescaling; controls TCCR0B[2:0] aka CS0[2:0]).
//
// This controls how fast the system ticks.  We need to debounce
// before registering a keypress, so this should be reasonably fast in
// order to feel responsive.  See also DebounceTickLimit, which should
// be decreased if you increase the tick frequency.
//
//   0x05; // clkIO/1024 -> 61 Hz
//   0x04; // clkIO/256 -> 244.14 Hz
//   0x03; // clkIO/64 -> 976.6 Hz
static uint8_t const Timer0Overflow = 0x04;

// Number of consecutive ticks that a switch has to maintain the same
// value in order to register a keypress.
static uint8_t const DebounceTickLimit = 3;

// Interrupt communication.
static volatile uint8_t _timer0_fired;
    // Default state: all high = nothing pressed
static volatile uint8_t _raw_switches_state;

static struct debounce_state switch_debounce_states[7];
static uint8_t debounced_switches = 0x7f;

static void setup(void) {
	// 16 MHz clock speed
	CPU_PRESCALE(0);

    // Configure PORTB[0:6] as inputs.
    DDRB = 0x00;
    // Turn on internal pull-ups on PORTB[0:6].  This means we will read
    // them as logic high when the switches are open.  Logic low means
    // the switch is pressed (ie. active low).
    PORTB = 0x7f;

    LED_CONFIG;
    LED_OFF;
    
    for (uint8_t i = 0; i < 7; i++) {
        switch_debounce_states[i].state = 1;
        switch_debounce_states[i].count = 0;
    }

	// Initialize USB, and then wait for the host to set
	// configuration.  If the Teensy is powered without a PC connected
	// to the USB port, this will wait forever.
	usb_init();
	while (!usb_configured()) /* wait */ ;
    
	// Wait an extra second for the PC's operating system to load drivers
	// and do whatever it does to actually be ready for input
	_delay_ms(1000);

    cli();
    
    // Configure timer 0 to give us ticks
	TCCR0A = 0x00;
	TCCR0B = Timer0Overflow & 0x07;
	TIMSK0 = (1<<TOIE0); // use the overflow interrupt only
    _timer0_fired = 0;
}

static uint8_t update_debounced_state(uint8_t raw_switches_state) {
    for(int i = 0; i < 7; i++) {
        uint8_t key_val = (raw_switches_state >> i) & 0x01;
        if (key_val != switch_debounce_states[i].state) {
            // Any time the read value doesn't match our debounce
            // state, we reset the count.
            switch_debounce_states[i].count = 0;
            switch_debounce_states[i].state = key_val;
        } else if (switch_debounce_states[i].count < DebounceTickLimit) {
            // If it DOES match and we haven't reached the debounce
            // tick limit, we increment it.
            switch_debounce_states[i].count++;
            if (switch_debounce_states[i].count == DebounceTickLimit) {
                // Once we've hit the tick limit, we register that as
                // a keypress state change.
                
                // Clear the bit for this switch, and then set it to
                // the new, debounced value.
                debounced_switches &= ~(0x01 << i);
                debounced_switches |= (key_val << i);
            }
            
        }
    }
    
    return debounced_switches;
}

static void run(void) {
    uint8_t last_pressed_keys = 0x7f;

    uint8_t dial_moving = 0;
    uint8_t dial_position = (PINB >> DialA) & 0x01;
    Direction dial_direction = DirectionCCW;
    
	for(;;) {        
        uint8_t timer0_fired = 0;
        uint8_t raw_switches_state = 0x7f;
        
        // Watch for interrupts, and sleep if nothing has fired.
        cli();
        while(!_timer0_fired) {
            set_sleep_mode(SLEEP_MODE_IDLE);
            sleep_enable();
            // It's safe to enable interrupts (sei) immediately before
            // sleeping.  Interrupts can't fire until after the
            // following instruction has executed, so there's no race
            // condition between re-enabling interrupts and sleeping.
            sei();
            sleep_cpu();
            sleep_disable();
            cli();
        }

        timer0_fired = _timer0_fired;
        raw_switches_state = _raw_switches_state;
        _timer0_fired = 0;

        sei();

        if (timer0_fired) {            
            uint8_t pressed_keys = update_debounced_state(raw_switches_state);
            uint8_t changed_keys = last_pressed_keys ^ pressed_keys;

            // Process normal keys
            for(int i = 0; i < 7; i++) {
                // A switch is pressed if it's logic low.
                if ((((pressed_keys >> i) & 0x01) == 0) &&
                    ((changed_keys >> i) & 0x01)) {

                    uint8_t key = SwitchKeyMap[i];
                    if (key != 0) {
                        usb_keyboard_press(key, 0, 0);
                    }
                }
            }

            // Process dial
            if (((pressed_keys >> DialA) & 0x01) != ((pressed_keys >> DialB) & 0x01)) {
                // The dial inputs are different from one another, so
                // it's moving now.
                dial_moving = 1;
                dial_direction = (((pressed_keys >> DialA) & 0x01) != dial_position) ? DirectionCW : DirectionCCW;
            } else if (dial_moving) {
                // Dial was moving but now has stopped, as indicated
                // by the fact that the two inputs now have the same
                // value.
                dial_moving = 0;
                if (((pressed_keys >> DialA) & 0x01) != dial_position) {
                    // Dial moved to new position.
                    dial_position = ((pressed_keys >> DialA) & 0x01);
                    if (dial_direction == DirectionCW) {
                        usb_keyboard_press(KEY_V, 0, 0);
                    } else {
                        usb_keyboard_press(KEY_6, 0, KEY_SHIFT);
                    }
                } else {
                    // Dial returned to old position.  (Nothing to
                    // do.)
                }
            } else if (((pressed_keys >> DialA) & 0x01) != dial_position) {
                // Dial isn't moving, and A and B positions match, but
                // they don't match what we expect so we missed a full
                // click and need to update our internal state to
                // match.
                dial_position = ((pressed_keys >> DialA) & 0x01);
            }

                
            last_pressed_keys = pressed_keys;
        }

    }
}

int main(void) {
    setup();
    run();
    return 0;
}


// Timer 0 overflow interrupt handler.
ISR(TIMER0_OVF_vect) {
    _timer0_fired = 1;
    _raw_switches_state = PINB & 0x7f;
}
