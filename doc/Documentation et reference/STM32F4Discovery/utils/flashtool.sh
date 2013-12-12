#!/bin/bash
arm-none-eabi-gdb -batch -ex "target remote | openocd -c 'gdb_port pipe' -f $1/config.cfg" -ex "monitor reset halt" -ex "load" -ex "monitor reset" $2
