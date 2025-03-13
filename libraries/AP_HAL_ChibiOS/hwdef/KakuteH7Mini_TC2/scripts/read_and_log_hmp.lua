--[[
    This script reads a HMP 110 humidity sensor via UART,
    reading will be saved to data flash logs and streamed to gcs.
    The logging rate is adjustable via parameter HMP_LOG_RATE.
--]]

-- create parameter table
local PARAM_TABLE_KEY = 74 -- parameter table key must be used by only one script on a particular flight controller, unique index value between 0 and 200
local PARAM_TABLE_PREFIX = 'HMP_'
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 1), string.format('Could not add param table %s', PARAM_TABLE_PREFIX))

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
  assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('Could not add param %s', name))
  return Parameter(PARAM_TABLE_PREFIX .. name)
end

--[[
  // @Param: HMP_LOG_RATE
  // @DisplayName: HMP logging rate
  // @Description: Adjust HMP logging rate in Hz
  // @Units: Hz
  // @User: Standard
--]]
local HMP_LOG_RATE = bind_add_param('LOG_RATE', 1, 10) -- logging rate [Hz]

local LOG_RATE = Parameter()
LOG_RATE:init('HMP_LOG_RATE')


local port = serial:find_serial(0)

port:begin(19200)
port:set_flow_control(0)

if not port then
  gcs:send_text(0, "No Scripting Serial Port")
  return
end

buffer = {}

function decode_and_write_to_flash(fullmessage)
  local serialStr = string.char(table.unpack(fullmessage))
  count = 0
  log_data = {}
  for word in serialStr:gmatch("[0-9.-]+") do
    count = count + 1
    log_data[count] = tonumber(word)
  end
  if count == 3 then -- only log if everything is there
    logger.write('HMP', 'Temperature,Humidity,Dewpoint', 'fff', 'O-O', '---', table.unpack(log_data))
    gcs:send_named_float('HMP (°C)', log_data[1])
    gcs:send_named_float('HMP (% RH)', log_data[2])
  else
    gcs:send_text(4, "HMP data error: " .. serialStr)
  end
end

function spit()
  local n_bytes = port:available()
  if n_bytes > 0 then
    while n_bytes > 0 do
      table.insert(buffer, port:read())
      if buffer[#buffer] == 10 then
        if #buffer > 20 then
          decode_and_write_to_flash(buffer)
        end
        buffer = {}
      end
      n_bytes = n_bytes - 1
    end
  end
  return spit, (1000/LOG_RATE:get())
end

return spit()