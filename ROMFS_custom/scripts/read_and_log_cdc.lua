--[[
    This script reads a P14 Rapid humidity sensor via CDC AD7745 on I²C,
    reading will be saved to data flash logs and streamed to gcs.
]]
--

-- CDC register names
local CDC_I2C_ADDR = 72; --I²C address 0x48
local CDC_STATUS = 0x00;
local CDC_CAP_DATA = 0x01;
local CDC_VT_DATA = 0x04;
local CDC_CAP_SETUP = 0x07;
local CDC_VT_SETUP = 0x08;
local CDC_EXC_SETUP = 0x09;
local CDC_CONFIG = 0x0A;
local CDC_CAP_DAC_A = 0x0B;
local CDC_CAP_DAC_B = 0x0C;

-- load the I²C driver, bus 0
local sensor = i2c:get_device(0, CDC_I2C_ADDR)
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

-- parameter for p14 humidity offset, default -106
local PARAM_TABLE_KEY = 72
assert(param:add_table(PARAM_TABLE_KEY, "P14_", 30), 'could not add param table')
assert(param:add_param(PARAM_TABLE_KEY, 1, 'OFFSET', -106), 'could not add param1')

-- get capacitance and convert it to humidity
local function getcap(addr)
  local cap = data(addr)
  if cap then
    local P14_OFFSET = Parameter()
    P14_OFFSET:init('P14_OFFSET')
    local p14_offset = P14_OFFSET:get()
    return cap / 42279.1 - 151.5 + p14_offset
  end
end

-- get voltage/temperature, return temperature
local function getvt(addr)
  local vt = data(addr)
  if vt then
    return (vt / 2048) - 4096
  end
end

-- set TSYS01 Source ID parameter (battery number)
param:set('TEMP1_SRC', 3)
param:set('TEMP1_SRC_ID', 1)

-- initialize CDC
sensor:write_register(CDC_CAP_SETUP, 0x80) --bit 7 CAPEN = 1
sensor:write_register(CDC_VT_SETUP, 0x80)  --bit 7 VTEN = 1
sensor:write_register(CDC_EXC_SETUP, 0x1B) --bit 0 EXCLVL0 = 1, bit 1 EXCLVL0 = 1, bit 3 EXCA = 1, 4 EXCB = 1
sensor:write_register(CDC_CONFIG, 0x01)    --bit 0 MD0 = 1
sensor:write_register(CDC_CAP_DAC_A, 200)
sensor:write_register(CDC_CAP_DAC_B, 0x00)

function update()
  -- check whether CDC is connected and if so, read status
  if sensor:read_registers(0) then
    status = sensor:read_registers(CDC_STATUS)
    if not status then
      gcs:send_text(3, "CDC: failed to read status")
      return update, 10000
    end
  else
    gcs:send_text(3, "CDC: not connected or no power")
    return update, 10000
  end

  -- show TSYS01 temperature on Mission Planner Quicktab
  local tsys01_temp = battery:get_temperature(0)
  if tsys01_temp then
    gcs:send_named_float('TSYS (°C)', tsys01_temp)
  end

  -- status register bit map
  local EXCERR = (status >> 3) & 1
  local RDY = (status >> 2) & 1
  -- following status bits are not needed, temperature and humidity is only read when new data for both is available
  -- local RDYVT = (status >> 1) & 1
  -- local RDYCAP = (status >> 0) & 1

  -- report CDC excitation output errors
  if EXCERR == 1 then
    gcs:send_text(3, "CDC: excitation output cannot be driven properly")
    return update, 10000
  end

  -- check if capacitance and voltage/temperature conversion is finished, read and log data
  if RDY == 0 then
    local hum = getcap(CDC_CAP_DATA)
    local temp = getvt(CDC_VT_DATA)
    gcs:send_named_float('CDC (°C)', temp)
    gcs:send_named_float('CDC (% RH)', hum)
    if temp and hum then
      logger:write('CDC', 'Humidity,Temperature', 'ff', '-O', '--', hum, temp)
    else
      gcs:send_text(1, "CDC: failed to read data")
    end
  else
    return update()
  end

  return update, 10
end

return update()