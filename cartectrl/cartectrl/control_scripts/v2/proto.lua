#!/usr/bin/env lua5.2

-------------------------------------------------------------------------------
local bit32 = require 'bit32';

local _G = _G;
local select, print = select, print;

local lshift, band = bit32.lshift, bit32.band;

local portAddress = '/dev/ttyACM0';
local writePort = assert(io.open(portAddress, 'wb'));
local readPort  = assert(io.open(portAddress, 'rb'));

writePort:setvbuf "no";
readPort:setvbuf "no";

-------------------------------------------------------------------------------
do local tinsert = table.insert;
    function table.insert(t, arg1, arg2)
        local val = arg2 or arg1;
        local pos = arg2 and arg1 or #t + 1;

        if type(val) == 'table' then
            for _, v in ipairs(val) do
                tinsert(t, pos, v);
                pos = pos + 1;
            end
        else
            tinsert(t, pos, val);
        end
    end
end
-------------------------------------------------------------------------------
local meta_table__  = { __index = table; };
local function newtable(init)
    return setmetatable(init, meta_table__);
end
-------------------------------------------------------------------------------
local function tonumbers(item, ...)
    if not item then return; end
    return tonumber(item), tonumbers(...);
end
-------------------------------------------------------------------------------
local function bytes(num)
    return band(num,0xFF), lshift(band(num,0xFF00), -8);
end
-------------------------------------------------------------------------------
local function tohex(s)
    t = { s:byte(1,-1) };
    r = {};
    local function tochar(int)
        if int >=32 and int <= 126 then return string.char(int); end
        return '.';
    end
    
    for _,v in ipairs(t) do
        local str = string.format("0x%02X (%3d '%s')",v,v,tochar(v));
        table.insert(r,str);
    end
    return "[ " .. table.concat(r,",\n  ") .. " ]";
end
-------------------------------------------------------------------------------
local function check(name, val, min, max, stackpos)
    stackpos = stackpos or 2;
    if not val then error("missing value '"..name.."'", stackpos); end
    if val < min or val > max then 
        error("invalid value '"..name.."'", stackpos); 
    end
end
-------------------------------------------------------------------------------
local function checkid(id)
    return check("id", id, 0, 254, 3);
end
-------------------------------------------------------------------------------
local function chksum(t)
    r = 0;
    for _,v in ipairs(t) do
        r = r + v;
    end
    return band(0xFF, r);
end
-------------------------------------------------------------------------------
local function preparePacket(pkt)
    local t = newtable {};
    t:insert{ bytes(#pkt + 2) };
    t:insert(pkt);
    t:insert(chksum(t));
    t:insert(1, {0xFF, 0xFF});
    return t;
end
-------------------------------------------------------------------------------
local function stringify(t)
    return string.char(table.unpack(t));
end
-------------------------------------------------------------------------------
local function writeCart(str)
    writePort:write(str);
end
-------------------------------------------------------------------------------
local function readCart(what)
    return readPort:read(what)
end
-------------------------------------------------------------------------------
local function sendPacket(data)
    local data = newtable(data);
    local packets = newtable {};

    for _, v in ipairs(data) do
        local innerpkt  = newtable {};
        innerpkt:insert{ v.tag:byte(1, -1) };
        innerpkt:insert{ bytes(#v + 2) };
        innerpkt:insert(v);
        packets:insert(innerpkt);
    end
    return writeCart(stringify(preparePacket(packets)));
end
-------------------------------------------------------------------------------
local env = {};

do local _ENV = env;

    function MO(id, position, time)
        checkid(id);
        check("position",position, 0,1023);
        check("time",    time,     0,255);
        
        local lsb, msb = bytes(position);
        sendPacket {{ tag = 'MO', id, lsb, msb, time }};
    end
    
    function MR(id, cmd, ...) 
        checkid(id);
        check('cmd', cmd, 1, 9);
        -- ... is a list of numbers.

        sendPacket {{ tag = 'MR', id, cmd, ... }};
    end

    function MRR(id, cmd, addr, len)
        MR(id, cmd, addr, len);
        local numread = 15 + len;
        
        local s = readPort:read(numread);
        print(tohex(s));
    end
    
    function TO(id, val)
        checkid(id);
        check("val",val, 0,2);
        
        sendPacket {{ tag = 'TO', id, val }};
    end
    
    function TT(color)
        sendPacket {{tag = 'MR', 5, 3, 53, 1, color},{tag = 'MR', 253, 3, 53, 1, color}};
    end

    function T1()
        sendPacket {{tag = 'MO', 253, 0, 1, 100}, {tag = 'MO', 5, 0, 3, 100}};
    end
    
    function T2()
        sendPacket {{tag = 'MO', 253, 0, 3, 100}, {tag = 'MO', 5, 0, 1, 100}};
    end
    
    function KP(id, val)
        local lsb, msb = bytes(val);
        MR(id, 3, 24, 2, lsb, msb);
    end
    
    function KD(id, val)
        local lsb, msb = bytes(val);
        MR(id, 3, 26, 2, lsb, msb);
    end
    
    function KI(id, val)
        local lsb, msb = bytes(val);
        MR(id, 3, 28, 2, lsb, msb);
    end
    
    function FD(id, val)
        local lsb, msb = bytes(val);
        MR(id, 3, 30, 2, lsb, msb);
    end
    
    function FDD(id, val)
        local lsb, msb = bytes(val);
        MR(id, 3, 32, 2, lsb, msb);
    end
end
-------------------------------------------------------------------------------
-------------------------------------------------------------------------------
-------------------------------------------------------------------------------
local invoked = arg[0]:match("/(.-)$",-4) or arg[0];

env[invoked](tonumbers(...));

writePort:close();
readPort:close();
-------------------------------------------------------------------------------
-- end
