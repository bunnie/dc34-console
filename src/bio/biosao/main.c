#include <stdint.h>

#include "bio.h" // this must always be first

#define SAO_GPIO1 21
#define SAO_GPIO2 22
#define SAO_GPIO3 30
#define SAO_GPIO4 31

#define QUANTUM_PER_MS 1000 // assumes `--quantum 1MHz` passed as part of init
// gpio config assumes `--sao 1`

void main(void) {
    uint32_t aclk_start;
    uint32_t aclk_end;
    uint32_t elapsed;
    // setup input and output pins
    uint32_t input_mask = 1 << SAO_GPIO3;
    set_gpio_mask(input_mask);
    set_input_pins(input_mask);

    set_output_pins(input_mask);
    clear_gpio_pins_n(0); // drives the pin low
    while (1) {
        for (uint32_t low_wait = 0; low_wait < 256; low_wait++) {
            // wait
        }
        aclk_start = aclk_counter();
        set_input_pins(input_mask);
        while (read_gpio_pins() == 0) {
            // wait for rising edge
        }
        aclk_end = aclk_counter();

        if (aclk_end > aclk_start) {
            elapsed = aclk_end - aclk_start;
        } else {
            elapsed = (0x3fffffff - aclk_end) + aclk_start;
        }
        push_fifo3(elapsed);
        pop_fifo3(); // read it back so that the loop doesn't stall

        set_output_pins(input_mask);
        clear_gpio_pins_n(0); // drives the pin low
    }
}

/*
// this scans through all of memory and tries to extract any data it can.
// it should only return data from the BIO segment, which is at 0x0.

void main(void) {
    uint32_t counter = 0;
    uint32_t *raw_ptr = (uint32_t *) 0x10000000;
    uint32_t sample = 0;
    // search for data in all of memory!
    raw_ptr[0] = 0x0;
    while (1) {
        sample = raw_ptr[counter];
        counter++;
        if (sample != 0) {
            // this automatically blocks
            push_fifo3(counter * 4 + (uint32_t) raw_ptr);
            push_fifo3(raw_ptr[counter]);
        }
    }
}
*/