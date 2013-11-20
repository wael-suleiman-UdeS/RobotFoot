target remote | openocd -c 'gdb_port pipe' -f config.cfg


define progflash
    monitor reset halt
    load
    end

define reset
    monitor reset halt
    set $pc = Reset_Handler
    end
