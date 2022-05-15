# zig-pico-sdk

After first attempting to rewrite the SDK from scratch in Zig
([zig-pico](https://github.com/Linouth/zig-pico)), I realized that
this would be way too much work for me right now. Instead, this repo is a
wrapper around the official [pico-sdk](https://github.com/raspberrypi/pico-sdk).
It allows you to compile Zig programs for the Pico microcontroller (RP2040),
while being able to use the official SDK.

The `build.zig` file is setup to use the zig compiler to compile your program to
an object file. Then the pico-sdk will run as normal, and link the libraries
together with this object file into a binary.

Make sure that you have `arm-none-eabi-gcc`, `cmake`, `make`, and
the arm libs installed (`arm-none-eabi-newlib` on arch).

**This wrapper is extremely experimental still!** I expect plenty of bugs and
issues. Feel free to open issues, or to contribute.
