--[[
    This script starts RunCam video recording on arm and stops video recording on disarm.
    To access the camera controls the port the RunCam is connected to must be set as a scripting port.
--]]

local runcam = serial:find_serial(0)

if not runcam then
    gcs:send_text(0, "RunCam: No scripting serial port")
    return
end

runcam:begin(115200)
runcam:set_flow_control(0)

local RUNCAM_HEADER = 0xCC
local RCDEVICE_PROTOCOL_COMMAND_CAMERA_CONTROL = 0x01
local RCDEVICE_PROTOCOL_SIMULATE_POWER_BTN = 0x01
local recording = false

local function send_packet(command, action)
    local buffer = {}
    buffer[1] = RUNCAM_HEADER
    buffer[2] = command
    buffer[3] = action
    buffer[#buffer + 1] = 0xE7 -- hard coded CRC-8/DVB-S2 checksum (0xCC 0x01 0x01)
    for i = 1, #buffer do
        runcam:write(buffer[i])
    end
end

function start_stop_recording()
    if not recording and arming:is_armed() then
        send_packet(RCDEVICE_PROTOCOL_COMMAND_CAMERA_CONTROL, RCDEVICE_PROTOCOL_SIMULATE_POWER_BTN)
        gcs:send_text(6, 'RunCam recording started')
        recording = true
    elseif recording and not arming:is_armed() then
        send_packet(RCDEVICE_PROTOCOL_COMMAND_CAMERA_CONTROL, RCDEVICE_PROTOCOL_SIMULATE_POWER_BTN)
        gcs:send_text(6, 'RunCam recording stopped')
        recording = false
    end
    return start_stop_recording, 1000
end

return start_stop_recording()