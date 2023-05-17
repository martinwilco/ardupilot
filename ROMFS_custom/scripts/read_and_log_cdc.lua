--[[
    This script reads a P14 Rapid humidity sensor on I²C,
    reading will be saved to data flash logs and streamed to gcs.
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
  local data_h = sensor:read_registers(addr + 0)
  local data_m = sensor:read_registers(addr + 1)
  local data_l = sensor:read_registers(addr + 2)
  if data_h and data_m and data_l then
    return data_h << 16 | data_m << 8 | data_l
  end
end

local function getcap(addr)
  local cap = data(addr)
  -- gcs:send_text(6, "CDC: capacitance "..tostring(cap))
  if cap then
    return cap / 42279.1 - 151.5 - 106
  end
end

local function getvt(addr)
  local vt = data(addr)
  if vt then
    return (vt / 2048) - 4096
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
    gcs:send_text(3, "CDC: not connected or no power")
    return update, 10000
  end

  local tsys01_temp = battery:get_temperature(0)
  if tsys01_temp then
    gcs:send_named_float('TSYS (°C)', tsys01_temp)
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

  if RDY == 0 then -- report capacitance and voltage/temperature conversion status
    local hum = getcap(CDC_CAP_DATA)
    local temp = getvt(CDC_VT_DATA)
    gcs:send_named_float('P14 (°C)', temp)
    gcs:send_named_float('P14 (% RH)', hum)
    if temp and hum then
      logger:write('P14', 'Humidity,Temperature', 'ff', '-O', '--', hum, temp)
    else
      gcs:send_text(1, "CDC: failed to read data")
    end
  else
    return update()
  end

  return update, 10
end

return update()