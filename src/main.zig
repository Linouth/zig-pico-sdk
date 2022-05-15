const std = @import("std");
const c = @cImport({
    @cInclude("pico/stdlib.h");
});

const LED_PIN = c.PICO_DEFAULT_LED_PIN;

pub export fn main() void {
    c.gpio_init(LED_PIN);
    c.gpio_set_dir(LED_PIN, c.GPIO_OUT == 1);
    while (true) {
        c.gpio_put(LED_PIN, true);
        c.sleep_ms(250);
        c.gpio_put(LED_PIN, false);
        c.sleep_ms(1000);
    }
}
