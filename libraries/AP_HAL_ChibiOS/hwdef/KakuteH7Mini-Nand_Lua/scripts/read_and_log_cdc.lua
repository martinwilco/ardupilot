--[[
    This script reads a P14 Rapid humidity sensor via CDC AD7745 on I²C,
    reading will be saved to data flash logs and streamed to gcs.
    The logging rate is adjustable via parameter P14_LOG_RATE.
    The humidity offset is adjustable via parameter P14_OFFSET2NOM.
--]]

-- CDC register names
local CDC_I2C_ADDR = 0x48; --I²C address 72
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

-- read three consecutive bytes from a given location
local function data(addr)
  local data_h = sensor:read_registers(addr + 0)
  local data_m = sensor:read_registers(addr + 1)
  local data_l = sensor:read_registers(addr + 2)
  if data_h and data_m and data_l then
    return data_h << 16 | data_m << 8 | data_l -- cdc_rapid_val
  end
end

-- create parameter table
local PARAM_TABLE_KEY = 73 -- parameter table key must be used by only one script on a particular flight controller, unique index value between 0 and 200
local PARAM_TABLE_PREFIX = 'P14_'
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 2),
  string.format('Could not add param table %s', PARAM_TABLE_PREFIX))

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
  assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('Could not add param %s', name))
  return Parameter(PARAM_TABLE_PREFIX .. name)
end

--[[
  // @Param: P14_OFFSET
  // @DisplayName: P14 humidity offset
  // @Description: Adjust offset to RH_P14_raw_nom
  // @Units: %
  // @User: Standard
--]]
P14_OFFSET2NOM = bind_add_param('OFFSET2NOM', 1, -106) -- offset to RH_P14_raw_nom [% RH]

--[[
  // @Param: P14_LOG_RATE
  // @DisplayName: P14 logging rate
  // @Description: Adjust P14 logging rate in ms
  // @Units: ms
  // @User: Standard
--]]
P14_LOG_RATE = bind_add_param('LOG_RATE', 2, 10) -- logging rate [ms], CDC conversion time set to 11 ms, output data rate 90.9 Hz -> loop every 10 ms

local OFFSET = Parameter()
OFFSET:init('P14_OFFSET2NOM')
local LOG_RATE = Parameter()
LOG_RATE:init('P14_LOG_RATE')

-- get capacitance and convert it to humidity
-- RH_P14_raw_nom = cdc_rapid_val / 42279.1 - 198.4 + 576.9 - 560 + 30 = cdc_rapid_val / 42279.1 - 151.5
-- RH_P14_raw_min = cdc_rapid_val / 42279.1 - 198.4 + 461.5 - 720 + 30 = cdc_rapid_val / 42279.1 - 426.9
-- RH_P14_raw_max = cdc_rapid_val / 42279.1 - 198.4 + 692.3 - 400 + 30 = cdc_rapid_val / 42279.1 + 123.9
-- scale factor at 1% R1, R2 resistance and 100pm/deg, i.e. 1.5% deviation
-- RH_P14_raw_nomscale = cdc_rapid_val / 42279.1 - Offset
-- RH_P14_raw_minscale = cdc_rapid_val / 34695.5 - Offset
-- RH_P14_raw_maxscale = cdc_rapid_val / 49836.4 - Offset
local function getcap(addr)
  local cdc_rapid_val = data(addr)
  if cdc_rapid_val then
    -- parameter P14 offset to nominal
    local p14_offset2nom = OFFSET:get()
    gcs:send_named_float('P14_OFFSET', p14_offset2nom)
    return cdc_rapid_val / 42279.1 - 151.5 + p14_offset2nom, cdc_rapid_val
  end
end

-- get voltage/temperature, return temperature
local function getvt(addr)
  local vt = data(addr)
  if vt then
    return (vt / 2048) - 4096
  end
end

-- initialize CDC
sensor:write_register(CDC_CAP_SETUP, 0x80) -- bit 7 CAPEN = 1
sensor:write_register(CDC_VT_SETUP, 0x80)  -- bit 7 VTEN = 1
sensor:write_register(CDC_EXC_SETUP, 0x1B) -- bit 0 EXCLVL0 = 1, bit 1 EXCLVL0 = 1, bit 3 EXCA = 1, 4 EXCB = 1
sensor:write_register(CDC_CONFIG, 0x01)    -- bit 0 MD0 = 1
sensor:write_register(CDC_CAP_DAC_A, 200)  -- amp-factor
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

  -- show TSYS01 temperature on Mission Planner Quicktab, works only if parameter TEMP1_SRC is set to 3 and TEMP1_SRC_ID to 1
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
    local p14_raw, cdc_raw = getcap(CDC_CAP_DATA)
    local temp = getvt(CDC_VT_DATA)
    gcs:send_named_float('P14 (% RH)', p14_raw)
    gcs:send_named_float('CDC Raw', cdc_raw)
    gcs:send_named_float('CDC (°C)', temp)
    if p14_raw and cdc_raw and temp then
      logger:write('MMT', 'P14_Raw,CDC_Raw,CDC_Temp', 'fff', '--O', '---', p14_raw, cdc_raw, temp)
    else
      gcs:send_text(1, "CDC: failed to read data")
    end
  else
    return update()
  end
  return update, LOG_RATE:get()
end

return update()