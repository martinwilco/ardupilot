--[[
    This script reads a AMS5812 pressure sensor on I²C,
    reading will be saved to data flash logs and streamed to gcs.
]]
--

-- AMS register names
local AMS_I2C_ADDR = 120; --I²C address 0x78

-- AMS 5812-0150-B specs
local min_pres_psi = 11.0;
local max_pres_psi = 17.5;

-- load the I²C driver, bus 0
local sensor = i2c:get_device(0, AMS_I2C_ADDR)
sensor:set_retries(10)

-- read data as 32bit integer (probably not possible with ArduPilot Lua scripting)
local function data_read(addr)
  local data = sensor:read_registers(addr,4)
  if data then
    return (data << 16) & 0x7FFF, data & 0x7FFF
  end
end

-- consecutive read
local function consecutive_read(addr)
  local msb_pres = sensor:read_registers(addr + 0)
  local lsb_pres = sensor:read_registers(addr + 1)
  local msb_temp = sensor:read_registers(addr + 2)
  local lsb_temp = sensor:read_registers(addr + 3)
  if msb_pres and lsb_pres and msb_temp and lsb_temp then
    return (msb_pres & 0x7F) << 24 | lsb_pres << 16, (msb_temp & 0x7F) << 8 | lsb_temp
  end
end

-- multibyte read
local function multibyte_read(addr)
  local data = sensor:read_registers(addr,4)
  if data then
    return (data[0] & 0x7F) << 8 | data[1], (data[2] & 0x7F) << 8 | data[3]
  end
end

-- calculate sensor values from consecutive read
local function get_data_consecutive(addr)
    local pres_cnts, temp_cnts = consecutive_read(addr)
    if pres_cnts and temp_cnts then
        return (pres_cnts - 3277) / ((29491 - 3277) / (max_pres_psi - min_pres_psi)) + min_pres_psi, (temp_cnts - 3277) / 26214 * 110 - 25
    end
end

-- calculate sensor values from multibyte read
local function get_data_multibyte(addr)
  local pres_cnts, temp_cnts = multibyte_read(addr)
  if pres_cnts and temp_cnts then
      return (pres_cnts - 3277) / ((29491 - 3277) / (max_pres_psi - min_pres_psi)) + min_pres_psi, (temp_cnts - 3277) / 26214 * 110 - 25
  end
end

function update()
  -- check whether AMS is connected
  -- if not sensor:read_registers(AMS_I2C_ADDR) then
  --   gcs:send_text(3, "AMS: not connected or no power")
  --   return update, 10000
  -- end

  -- read and log data
  local pres_psi, temp = get_data_consecutive(AMS_I2C_ADDR)
  if pres_psi and temp then
    local pres_pa = pres_psi * 0.45359237 * 9.80665 / 0.0254 / 0.0254
    gcs:send_named_float('AMS (psi)', pres_psi)
    gcs:send_named_float('AMS (Pa)', pres_pa)
    gcs:send_named_float('AMS (°C)', temp)
    if pres_pa and temp then
      logger:write('MMT', 'AMS_Pres,AMS_Temp', 'ff', 'PO', '--', pres_pa, temp)
    else
      gcs:send_text(1, "AMS: failed to read data")
      return update()
    end
  end

  -- local pres_psi, temp = get_data_multibyte(AMS_I2C_ADDR)
  -- if pres_psi and temp then
  --   local pres_pa = pres_psi * 0.45359237 * 9.80665 / 0.0254 / 0.0254
  --   gcs:send_named_float('AMS (psi)', pres_psi)
  --   gcs:send_named_float('AMS (Pa)', pres_pa)
  --   gcs:send_named_float('AMS (°C)', temp)
  --   if pres_pa and temp then
  --     logger:write('MMT', 'AMS_Pres,AMS_Temp', 'ff', 'PO', '--', pres_pa, temp)
  --   else
  --     gcs:send_text(1, "AMS: failed to read data")
  --     return update()
  --   end
  -- end
  -- reaction time typical = 1 ms, maximum = 2 ms, theoretical output data rate 500 Hz
  return update, 2
end

return update () -- run immediately before starting to reschedule