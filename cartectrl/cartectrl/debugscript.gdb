target remote | openocd -c 'gdb_port pipe' -f config.cfg


define progflash
    monitor reset halt
    load
    end

define reset
    monitor reset halt
    set $pc = Reset_Handler
    end

define dumpmem
    dump binary memory dump.bin 0x20000000 0x20020000
    shell xxd dump.bin | view -
    end

