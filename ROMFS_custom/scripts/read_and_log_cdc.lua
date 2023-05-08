--[[
    This script reads a P14 humidity sensor on I²C,
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


function update()
  if sensor:read_registers(0) then
    gcs:send_text(5, "Found cdc at 0x48")
  end

  -- read status
  local status = sensor:read_registers(CDC_STATUS)
    gcs:send_text(5,"Status: "..tostring(status))
  if not status then
    gcs:send_text(0, "Failed to read cdc status")
    return update, 10000
  end
  return update, 10000
end

return update()