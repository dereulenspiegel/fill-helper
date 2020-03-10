#!/bin/sh
openocd -f openocd.cfg -c "program target/thumbv7m-none-eabi/release/fill-helper verify reset exit"
