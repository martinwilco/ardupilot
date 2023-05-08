--[[
    This script reads a SN-GCJA5 panasonic particle sensor on i2c
    reading will be saved to data flash logs, CSV file and streamed as named value floats
    Development of this script was sponsored by Cubepilot
    the code is heavily based on the SparkFun arduino library
    https://github.com/sparkfun/SparkFun_Particle_Sensor_SN-GCJA5_Arduino_Library
]]--

-- search for a index without a file, this stops us overwriting from a previous run
local index = 0
local file_name
while true do
  file_name = string.format('PMPmdot %i.csv',index)
  local file = io.open(file_name)
  local first_line = file:read(1) -- try and read the first character
  io.close(file)
  if first_line == nil then
    break
  end
  index = index + 1
end

-- open file and make header
file = assert(io.open(file_name, 'w'), 'Could not make file :' .. file_name)
file:write('Lattitude (°), Longitude (°), Absolute Altitude (m), mdot\n')
file:close()

-- register names
local EEPROM_ADDRESS = 0x40
local FLOW_MEASUREMENT = 0x00
local TEMPERATURE_MEASUREMENT = 0x1001
local FLOW_SCALE_FACTOR = 0x30DE
local FLOW_OFFSET = 0x30DF

-- load the i2c driver, bus 1
local sensor = i2c.get_device(1,EEPROM_ADDRESS) -- 0x40 is the slave address
sensor:set_retries(10)

-- Reads two consecutive bytes from a given location
local function readRegister16(addr)
  local lsb = sensor:read_registers(addr+00)
  local msb = sensor:read_registers(addr+10)
  if lsb and msb then
    return msb << 8 | lsb
  end
end

local function getmdot(mdotRegister)
  local count = readRegister16(mdotRegister)
  --gcs:send_named_float('count',count)
  if count then
    return (count - 32768.0)/ 120.0
  end
end

function update() -- this is the loop which periodically runs

  -- read flowrate
  local mdot = getmdot(FLOW_MEASUREMENT)

  if (not mdot)  then
    gcs:send_text(0, "Failed to read mass flowrate")
    return update, 10000
  end

  -- -- write to csv
  -- file = io.open(file_name, 'a')
  -- file:write(string.format('%0.8f\n', mdot))
  -- file:close()

  -- -- save to data flash
  -- logger.write('PART','mdot','f',mdot)

  -- send to GCS
  gcs:send_named_float('mdot',mdot)


  return update, 1000 -- reschedules the loop, 1hz
end

return update() -- run immediately before starting to reschedule


-- -- Reads two consecutive bytes from a given location
-- local function readRegister16(addr)
--   local thingy sensor:read_registers(addr)
--   return thingy
-- end