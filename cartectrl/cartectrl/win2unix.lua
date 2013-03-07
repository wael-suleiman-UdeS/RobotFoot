#!/usr/bin/env lua5.1

local main, help;

local fprintf, printf;

local arg, os, io = arg, os, io;

local PROGNAME = 'win2unix';

local configs = 
{
    unix = [[					<options conn_type="0" ]] ..
           [[serial_baud="115200" ]] ..
           [[additional_cmds_before='target remote | openocd -c ]] ..
           [[&quot;gdb_port pipe&quot; ]] ..
           [[-f config.cfg&#x0A;monitor reset halt&#x0A;load' />]];
    win =  [[					<options conn_type="0" ]] ..
           [[serial_baud="115200" ]] ..
           [[additional_cmds_before="target remote]] ..
           [[ localhost:3333&#x0A;monitor reset halt&#x0A;load" />]];
};

------------------------------------------------------------------------------
function fprintf(file, fmt, ...)
    return file:write(fmt:format(...));
end
------------------------------------------------------------------------------
function printf(fmt, ...)
    return fprintf(io.stdout, fmt, ...);
end
------------------------------------------------------------------------------
------------------------------------------------------------------------------
function help()
    printf([=[
Usage: %s [unix|win] <file>

    file: an mcu Codeblocks project.
]=], PROGNAME
    );
end
------------------------------------------------------------------------------
function main()
    if #arg < 2 and arg[1] ~= 'unix' and arg[1] ~= 'win' then
        help();
        return #arg == 0 and 0 or 1;
    end
    
    local lines = {};
    
    local f, err = io.open(arg[2], 'r');
    
    if not f then 
        printf('Could not open "%s" for reading: %s.\n', arg[2], err); 
        return 1;
    end
    
    for l in f:lines() do 
        table.insert(lines, l);
    end
    -- Simple check.
    if not lines[2] or lines[2] ~= '<CodeBlocks_project_file>' then
        printf('"%s" is not a Codeblocks project.\n', arg[2]);
        return 1;
    end
     
    f:close();
    
    local found = false;
    for i in ipairs(lines) do
        if lines[i]:find('target remote') then
            found = i;
            break;
        end
    end
    
    if not found then 
        printf('"%s" doesn\'t seem to be an mcu project...\n', arg[2]);
        return 1;
    end

    lines[found] = configs[arg[1]];
    
    f, err = io.open(arg[2], 'w');
    if not f then 
        printf('Could not open "%s" for writing: %s.\n', arg[2], err); 
        return 1;
    end
    
    for _,l in ipairs(lines) do
        fprintf(f, '%s\n', l);
    end
    
    f:close();
end
------------------------------------------------------------------------------
os.exit(main() or 0);

