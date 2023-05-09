--[[
    This script reads a CDC connected to a P14 humidity sensor on I²C,
    reading will be saved to data flash logs and streamed as named value floats
]]--

-- register names
local CDC_STATUS = 0x00;
local CDC_CAP_SETUP = 0x07;
local CDC_VT_SETUP = 0x08;
local CDC_EXC_SETUP = 0x09;

-- load the i2c driver, bus 0
local sensor = i2c:get_device(0,72)
sensor:set_retries(10)

-- reads two consecutive bytes from a given location
local function readRegister16(addr)
  local lsb = sensor:read_registers(addr+0)
  local msb = sensor:read_registers(addr+1)
  if lsb and msb then
    return msb << 8 | lsb
  end
end

-- reads three consecutive bytes from a given location
local function readRegister24(addr)
  local ll = sensor:read_registers(addr+0)
  local lh = sensor:read_registers(addr+1)
  local hl = sensor:read_registers(addr+2)
  if ll and lh and hl then
    return (hl << 16) | (lh << 8) | (ll << 0)
  end
end

function update()
  if sensor:read_registers(0) then
    gcs:send_text(5, "Found CDC at 0x48")
  end

  -- read status
  local status = sensor:read_registers(CDC_STATUS)
    gcs:send_text(5,"CDC status: "..tostring(status))
  if not status then
    gcs:send_text(5, "Failed to read CDC status")
    return update, 10000
  end

  -- status register bit map
  -- local not_used = (status >> 4) & 3
  local EXCERR =  (status >> 3) & 3
  local RDY =  (status >> 2) & 3
  local RDYVT = (status >> 1) & 3
  local RDYCAP = (status >> 0) & 3

  -- report excitation output errors
  if EXCERR ~= 0 then
    gcs:send_text(0, "CDC: excitation output cannot be driven properly")
    return update, 10000
  end

  -- report capacitive and voltage/temperature conversion status
  if RDY ~=1 then
    gcs:send_text(5, "CDC: conversion cap & vt finished")
  end

  -- report voltage/temperature conversion status
  if RDYVT ~=1 then
    gcs:send_text(5, "CDC: conversion vt finished")
  end

  -- report capacitive conversion status
  if RDYCAP ~=1 then
    gcs:send_text(5, "CDC: conversion cap finished")
  end
  return update, 10000
end

return update()