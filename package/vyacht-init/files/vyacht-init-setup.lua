
local uci      = require "uci"
local _go      = require "vyacht.get-opt-alt"


vtest = 1
local vy      = require "vyacht"

_uci_real  = cursor or _uci_real or uci.cursor()                   

-- option lua_prefix       /lua              
-- option lua_handler      /www/vyacht.lua
        
_uci_real:set("uhttpd", "main", "lua_prefix", "/lua")
_uci_real:set("uhttpd", "main", "lua_handler", "/www/vyacht.lua")
                                      
_uci_real:commit("uhttpd")

os.execute("/etc/init.d/uhttpd restart");

-- vi /etc/inittab
local file = io.open("/etc/inittab", "w")
if file ~= nil then
   file:write("::sysinit:/etc/init.d/rcS S boot\n")
   file:write("::shutdown:/etc/init.d/rcS K shutdown\n")
   file:flush()
   io.close(file) 
end

-- vi /etc/config/vyacht
local opts = _go.getopt(arg, options)
local eths = -1
local seatalk = 0
local n2k = 0

if opts["eth"] == nil then
  print("no number of ethernet devices given")
  return
else 
  eths = tonumber(opts["eth"])
end

if eths < 0 or eths > 2 then
  print("wrong number of ethernet devices given")
  return
end

if opts["seatalk"] then
  seatalk = 1
end
if opts["n2k"] then
  n2k = 1
end

local hw = readHardwareOptions()

if not hw then
  print("no hardware description found")
  return
end

if hw.software.version.x == 0 then
  print("too old software version found")
  return
end
  

if eths == 1 then
  hw.network.devices = {"radio0", "eth0.2"}
elseif eths == 2 then
  hw.network.devices = {"radio0", "eth0.1", "eth0.2"}
else 
  hw.network.devices = {"radio0"}
end

hw.module.type = "nmea0183"
if seatalk == 1 then
  hw.module.type = "seatalk"
  _uci_real:set("gpsd", "core", "device", {"/dev/ttyS0", "st:///dev/ttyS1"})
  _uci_real:commit("gpsd")
elseif n2k == 1 then
  hw.module.type = "nmea2000"
  _uci_real:set("gpsd", "core", "device", {"vyspi:///dev/vyspi0.0"})
  _uci_real:commit("gpsd")
  
  -- write module start file
  local file = io.open("/etc/modules.d/63-vyspi", "w")
   file:write("vyspi")
   file:flush()
   io.close(file) 
   
else
  _uci_real:set("gpsd", "core", "device", {"/dev/ttyS0", "/dev/ttyS1"})
  _uci_real:commit("gpsd")
end

writeHardwareOptions(hw)

resetSystem()

