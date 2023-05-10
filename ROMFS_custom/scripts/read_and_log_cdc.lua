--[[
    This script reads a CDC connected to a P14 humidity sensor on I²C,
    reading will be saved to data flash logs and streamed as named value floats
]]
--

-- register names
local CDC_STATUS = 0x00;
local CDC_CAP_SETUP = 0x07;
local CDC_VT_SETUP = 0x08;
local CDC_EXC_SETUP = 0x09;
local CDC_CONFIG = 0x0A;
local CDC_CAP_DAC_A = 0x0B;
local CDC_CAP_DAC_B = 0x0C;
local CDC_CAP_DATA = 0x01;
local CDC_VT_DATA = 0x04;

-- load the i2c driver, bus 0
local sensor = i2c:get_device(0, 72)
sensor:set_retries(10)

-- reads three consecutive bytes from a given location
local function data(addr)
  local data_h = sensor:read_registers(addr + 1)
  local data_m = sensor:read_registers(addr + 2)
  local data_l = sensor:read_registers(addr + 3)
  if data_h and data_m and data_l then
    return data_l << 16 | data_m << 8 | data_h
  end
end

local function getcap(addr)
  local count = data(addr)
  if count then
    return count / 1000.0
  end
end

local function getvt(addr)
  local measure = data(addr)
  if measure then
    return (measure / 2048) - 4096
  end
end

-- reads seven consecutive bytes from a given location
local function measure(addr)
  local status = sensor:read_registers(addr + 0)
  local cap_h = sensor:read_registers(addr + 1)
  local cap_m = sensor:read_registers(addr + 2)
  local cap_l = sensor:read_registers(addr + 3)
  local vt_h = sensor:read_registers(addr + 4)
  local vt_m = sensor:read_registers(addr + 5)
  local vt_l = sensor:read_registers(addr + 6)
  if status and cap_h and cap_m and cap_l and vt_h and vt_m and vt_l then
    return (vt_l << 48) | (vt_m << 40) | (vt_h << 32) | (cap_l << 24) | (cap_m << 16) | (cap_h << 8) | (status << 0)
  end
end

-- initialize cdc
sensor:write_register(CDC_CAP_SETUP, 0x80)
sensor:write_register(CDC_VT_SETUP, 0x80)
sensor:write_register(CDC_EXC_SETUP, 0x1B)
sensor:write_register(CDC_CONFIG, 0x01)
sensor:write_register(CDC_CAP_DAC_A, 200)
sensor:write_register(CDC_CAP_DAC_B, 0x00)

function update()
  -- check, whether CDC is connected and if so, read status
  if sensor:read_registers(0) then
    -- gcs:send_text(6, "CDC: found at 0x48")
    status = sensor:read_registers(CDC_STATUS)
    -- gcs:send_text(6,"CDC: status "..tostring(status))
    if not status then
      gcs:send_text(3, "CDC: failed to read status")
      return update, 10000
    end
  else
    gcs:send_text(3, "CDC: not connected")
    return update, 10000
  end

  -- status register bit map
  local EXCERR = (status >> 3) & 1
  local RDY = (status >> 2) & 1
  local RDYVT = (status >> 1) & 1
  local RDYCAP = (status >> 0) & 1

  -- report excitation output errors
  if EXCERR == 1 then
    gcs:send_text(3, "CDC: excitation output cannot be driven properly")
    return update, 10000
  end

  -- report capacitive and voltage/temperature conversion status
  if RDY == 1 then
    gcs:send_text(3, "CDC: cap and vt conversion not finished")
    return update, 10000
  end

  -- report voltage/temperature conversion status
  if RDYVT == 1 then
    gcs:send_text(3, "CDC: vt conversion not finished")
  else
    local vt_data = data(CDC_VT_DATA)
    if vt_data then
      -- gcs:send_text(6, "CDC: vt data " .. tostring(vt_data))
    else
      gcs:send_text(1, "CDC: failed to read vt data")
      return update, 10000
    end
    temp = getvt(CDC_VT_DATA)
    gcs:send_text(6, string.format("CDC: temperature %.1f °C", temp))
    gcs:send_named_float('Temp. (°C)', temp)
    if not temp then
      gcs:send_text(1, "CDC: failed to read vt data")
    end
  end

  -- report capacitive conversion status
  if RDYCAP == 1 then
    gcs:send_text(3, "CDC: cap conversion not finished")
  else
    local cap_data = data(CDC_CAP_DATA)
    if cap_data then
      -- gcs:send_text(6, "CDC: cap data " .. tostring(cap_data))
    else
      gcs:send_text(1, "CDC: failed to read cap data")
      return update, 10000
    end

    local cap = getcap(CDC_CAP_DATA)
    if not cap then
      gcs:send_text(1, "CDC: failed to read cap data")
    end
  end

  logger:write('CDC', 'Temperature', 'f', 'O', '-', temp)

  --[[   local data = measure(CDC_STATUS)
  if data then
    gcs:send_text(6, "CDC: data " .. tostring(data))
  else
    gcs:send_text(1, "CDC: failed to read data")
    return update, 10000
  end
 ]]
  return update, 500
end

return update()