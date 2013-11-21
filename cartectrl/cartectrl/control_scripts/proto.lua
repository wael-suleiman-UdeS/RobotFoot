#!/usr/bin/env lua5.2

-------------------------------------------------------------------------------
local bit32 = require 'bit32';

local _G = _G;
local lshift, band = bit32.lshift, bit32.band;


local writePort = assert(io.open('/dev/ttyACM0', 'wb'));
local readPort  = assert(io.open('/dev/ttyACM0', 'rb'));

writePort:setvbuf "no";
readPort:setvbuf "no";

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
    for _,v in ipairs(t) do
        table.insert(r,string.format("0x%02X",v));
    end
    return "[ " .. table.concat(r,", ") .. " ]";
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
local function preparePacket(t)
    table.insert(t,1,#t);
    table.insert(t,1,chksum(t));
    table.insert(t,1,0xFF);
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
    return writeCart(stringify(preparePacket(data)));
end
-------------------------------------------------------------------------------
env = {};

do local _ENV = env;

    function SP(id, position, time)
        checkid(id);
        check("position",position, 0,1023);
        check("time",    time,     0,255);
        
        local lsb, msb = bytes(position);
        sendPacket { 1, id, msb, lsb, time };
    end
    
    function RP(id)
        checkid(id);
        
        sendPacket { 2, id };
        local s = readCart(6);
        _G.print(tohex(s));
    end
    
    function ST(id, val)
        checkid(id);
        check("val",val, 0,2);
        
        sendPacket { 3, id, val };
    end
    
    function RS(id)
        checkid(id);
        
        sendPacket { 4, id };
        local s = readCart(5);
        _G.print(tohex(s));
    end
    
    function CS(id)
        checkid(id);
        
        sendPacket { 5, id };
    end
      
    function RB(id)
        checkid(id);
        
        sendPacket { 6, id };
    end
end
-------------------------------------------------------------------------------
-------------------------------------------------------------------------------
-------------------------------------------------------------------------------
local invoked = arg[0]:match("/(.*)$",-8) or arg[0];

env[invoked](tonumbers(...));

writePort:close();
readPort:close();
-------------------------------------------------------------------------------
-- end
