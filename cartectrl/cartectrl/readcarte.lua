#!/usr/bin/env lua5.2
-------------------------------------------------------------------------------
local bit32 = bit32;

local exec  = os.execute;
local print = print;
local band  = bit32.band;

-------------------------------------------------------------------------------
function getlen(s)
    local lsb, msb = s:byte(3,4);
    return lsb + msb * 256 + 2;
end
-------------------------------------------------------------------------------
local Tags = 
{
    MO = function(str, len)
        local l = getlen(str);
        if l < 4   then return 4; end
        if l > len then return len; end
        ---[[
        io.stdout:write(
            string.format('Motor %3d: pos = %6d, stat = %3d, %3d',
                str:byte(5),
                str:byte(6) + str:byte(7) * 256,
                str:byte(9,10))
              );
        if l > 10 then
            io.stdout:write(
                string.format(' PWM: %6d, V: %3d, T: %3d',
                    str:byte(11) + str:byte(12) * 256,
                    str:byte(13),
                    str:byte(14))
            );
        end
        io.stdout:write '\n';
              --]]
        return l;
    end;
    BU = function (str, len)
        local l = getlen(str);
        if l < 4   then return 4; end
        if l > len then return len; end
        io.stdout:write(
            string.format('Button %3d : val = %3d\n',
                str:byte(5),
                str:byte(6))
              );
        return l;
    end;
    MR = function (str, len)
        local l = getlen(str);
        if l < 4   then return 4; end
        if l > len then return len; end
        io.stdout:write(
            string.format('Motor %3d: cmd: 0x%02X, data = [',
                str:byte(5),
                str:byte(6))
              ); 
        local data = { str:byte(7,l) };
        io.stdout:write(table.concat(data,', '));
        io.stdout:write(']\n');
        return l;
    end
};
-------------------------------------------------------------------------------
setmetatable(Tags, { __index = function (self, idx)
    local v = rawget(self, idx);
    if v then return v; end
    
    print('Unknown tag ' .. idx );
    return function (s) 
                return getlen(s); 
            end 
    end
});
-------------------------------------------------------------------------------
function calcsum(s, len)
    if len < 2 then return false; end
    
    local sum = 0;
    for i=3,len+2 do
        sum = sum + s:byte(i);
        --print(i, sum, string.format('%02X',sum), s:byte(i), string.format('%02X',s:byte(i)));
    end
    --print('end>', s:byte(len+2), string.format('%02X',s:byte(len+2)));
    return band(sum, 0xFF) == s:byte(len+3);
end
-------------------------------------------------------------------------------
assert(exec 'stty -F /dev/ttyACM0 -ixon raw');
local readPort = assert(io.open('/dev/ttyACM0','r'));
--local readPort = assert(io.open('tmp.bin','r'));
readPort:setvbuf 'no';

local s = readPort:read(4);
while true do
    local packetlen = s:match('^\xFF\xFF(..)');
    if packetlen then
        local len = 0;
        do local lsb, msb = packetlen:byte(1,-1); len = lsb + 256 * msb; end
        
        s = s..readPort:read(len-1);
        
        local check = calcsum(s, len);
        s = s:sub(5,-1);
        if not check then
            print('Checksum error');
        else
            len = len - 2;
            --print(len);
            repeat
                local l = Tags[s:sub(1,2)](s, len);
                
                s = s:sub(l+1,-1);
                len = len - l;
                --print('nowlen', len);
            until len == 0;
            
            s = readPort:read(4);
        end
    else
        print('Error packet!');
        s = s:sub(2,-1)..readPort:read(1);
    end
    
    --if #s < numlen then  
    --   s = s .. (readPort:read(numlen) or os.exit(0,true));
    --end
end
-------------------------------------------------------------------------------


