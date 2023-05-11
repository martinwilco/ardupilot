local port = serial:find_serial(0)

port:begin(19200)
port:set_flow_control(0)

if not port then
  gcs:send_text(0, "No Scripting Serial Port")
  return
end

nth_msg2display = 50
msg_count = 0

buffer = {}

function decode_and_write_to_flash(fullmessage)
  local serialStr = string.char(table.unpack(fullmessage))
  count = 0
  log_data = {}
  for word in serialStr:gmatch("[0-9.-]+") do
    count = count + 1
    log_data[count] = tonumber(word)
  end
  if count == 3 then
    -- nur wegschreiben, wenn alles da ist. und auch nicht zuviel.
    logger.write('HMP', 'Temperature,Humidity,Dewpoint', 'fff', 'O-O', '---', table.unpack(log_data))
    msg_count = msg_count + 1
    if msg_count == nth_msg2display then
      gcs:send_text(5, string.format("HMP %.1f °C @ %.0f pcRH", log_data[1], log_data[2]))
      gcs:send_named_float('HMP (°C)', log_data[1])
      gcs:send_named_float('HMP (% RH)', log_data[2])
      msg_count = 0
    end
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
  return spit, 300
end

return spit, 300 -- do we need to wait here?