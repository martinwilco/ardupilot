--[[-----------------------------------------------------------------------------------------------------------
Copter fast descent maneouvre Lua script

Copter descends as fast as possible to a preset altitude while being turned
nosedown and levels off in a parabolic interception arc

CAUTION: This script is only tested for Copter 4.4 (and higher)
         USE AT YOUR OWN RISK

The script waits for the copter to be switched to guided flight mode and then:
1. slows copter down vertically
2. rotates copter in a nosedown dive and lets it descent at maximum possible speed
3. flares copter in a parabolic arc to slow it down
4. switches to loiter flight mode to return control back to pilot

How to use:
1.  set SCR_ENABLE = 1 to enable scripting (and reboot the autopilot)
2.  set SCR_HEAP_SIZE to 80000 or higher to allocate enough memory for this script
3.  set FDES_ENABLE = to enable fast descent maneouvre, guided mode will not be usable otherwise
4.  check FDES_MIN_ALT with the surrounding terrain, set minimum altitude from which fast descent is available
5.  set FDES_TARGET_ALT to the desired altitude where the fast descent manoeuvre should end
6.  arm copter, takeoff and fly in any other mode than guided
7.  to start fast descent maneouvre switch to guided flight mode, do not manipulate controls
8.  monitor the descent carefully and be ready to intervene in the event of irregularities (-> Troubleshooting)
9.  wait until fast descent manoeuvre is finished (gcs message and loiter mode engaged)
10. take back control and proceed as usual

Troubleshooting:
a) if descent is unstable, you can always switch to stabilize mode to regain control
b) if target altitude is missed by an unacceptable margin, modify FDES_TARGET_OFFS parameter
c) if copter cannot maintain desired pitch angle during descent, modify FDES_PITCH_ANGLE parameter
d) adjust the radius of the flare arc by modifying FDES_FLARE_TIME parameter

Martin Wilichowski, Nov 2023
Institute of Flight Guidance
TU Braunschweig
-------------------------------------------------------------------------------------------------------------]]

-- create parameter table
local PARAM_TABLE_KEY = 72 -- parameter table key must be used by only one script on a particular flight controller, unique index value between 0 and 200
local PARAM_TABLE_PREFIX = 'FDES_'
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 8), string.format('Could not add param table %s', PARAM_TABLE_PREFIX))

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('Could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

--[[
  // @Param: FDES_ENABLE
  // @DisplayName: Enable fast descent manoeuvre
  // @Description: Enable fast descent manoeuvre, guided mode not usable
  // @Bitmask: 0:Disabled,2:Enabled
  // @User: Standard
--]]
FDES_ENABLE = bind_add_param('ENABLE', 1, 1) -- 1: enabled, 2: disabled

--[[
  // @Param: FDES_TARGET_ALT
  // @DisplayName: Target altitude
  // @Description: Target altidude for fast descent manoeuvre
  // @Units: m
  // @User: Standard
--]]
FDES_TARGET_ALT = bind_add_param('TARGET_ALT', 2, 40) -- target altitude [m]

--[[
  // @Param: FDES_TARGET_OFFS
  // @DisplayName: Target altitude offset
  // @Description: Target altidude offset, modify if target altitude is missed
  // @Units: m
  // @User: Advanced
--]]
FDES_TARGET_OFFS = bind_add_param('TARGET_OFFS', 3, 28) -- target altitude offset [m]

--[[
  // @Param: FDES_MIN_ALT
  // @DisplayName: Minimum altitude
  // @Description: Minimum altidude from which fast descent manoeuvre is available
  // @Units: m
  // @User: Advanced
--]]
FDES_MIN_ALT = bind_add_param('MIN_ALT', 4, 80) -- minimum start altitude [m]

--[[
  // @Param: FDES_PITCH_ANGLE
  // @DisplayName: Desired pitch angle
  // @Description: Desired pitch angle for descent (90 deg equals nosedown)
  // @Range: 0 90
  // @Units: deg
  // @User: Advanced
--]]
FDES_PITCH_ANGLE = bind_add_param('PITCH_ANGLE', 5, 86) -- desired pitch angle [deg]

--[[
  // @Param: FDES_FLARE_TIME
  // @DisplayName: Flare duration
  // @Description: Time in seconds for flare arc to slow down descent
  // @Range: 1 5
  // @Units: s
  // @User: Advanced
--]]
FDES_FLARE_TIME = bind_add_param('FLARE_TIME', 6, 2) -- flare duration [s]

--[[
  // @Param: FDES_HOVER_OFFS
  // @DisplayName: Vertical hover velocity offset
  // @Description: Offset for vertical velocity to determine when copter is hovering
  // @Range: 0 5
  // @Units: m/s
  // @User: Advanced
--]]
FDES_HOVER_OFFS = bind_add_param('HOVER_OFFS', 7, 0.2) -- vertical hover velocity offset [m/s]

--[[
  // @Param: FDES_MOTOR_PWM
  // @DisplayName: Max motor pwm output during descent
  // @Description: Decrease motor pwm output for descent to minimize horizontal drifting
  // @Range: 1000 2000
  // @Units: pwm
  // @User: Advanced
--]]
FDES_MOTOR_PWM = bind_add_param('MOTOR_PWM', 8, 1250) -- motor descent pwm output [pwm]

-- bind parameters to variables
local ENABLE = Parameter()
ENABLE:init('FDES_ENABLE')
local TARGET_ALT = Parameter()
TARGET_ALT:init('FDES_TARGET_ALT')
local TARGET_OFFS = Parameter()
TARGET_OFFS:init('FDES_TARGET_OFFS')
local MIN_ALT = Parameter()
MIN_ALT:init('FDES_MIN_ALT')
local PITCH_ANGLE = Parameter()
PITCH_ANGLE:init('FDES_PITCH_ANGLE')
local FLARE_TIME = Parameter()
FLARE_TIME:init('FDES_FLARE_TIME')
local HOVER_OFFS = Parameter()
HOVER_OFFS:init('FDES_HOVER_OFFS')
local MOTOR_PWM = Parameter()
MOTOR_PWM:init('FDES_MOTOR_PWM')

-- constants
local copter_guided_mode_num = 4 -- guided mode is 4 on copter
local copter_loiter_mode_num = 5 -- loiter mode is 5 on copter
local motor1_fn = 33             -- motor 1 function number
local motor2_fn = 34             -- motor 2 function number
local motor3_fn = 35             -- motor 3 function number
local motor4_fn = 36             -- motor 4 function number

-- timing and state machine variables
local stage = 0               -- stage of descent
local interval_ms = 100       -- update interval in ms, 10Hz
local mot_pwm_max_set = false -- check whether max motor output is set
local heading_set = false     -- check whether heading is set
local reset = true            -- check whether parameters are reset

-- control related variables
local motor_des_pwm = MOTOR_PWM:get()                                                              -- decrease motor output for descent to minimize horizontal drifting
local new_motor_des_pwm = motor_des_pwm                                                            -- initialise new max motor output for gradual increase to preset motor output once at start
local desired_pitch_angle = -PITCH_ANGLE:get()                                                     -- get desired pitch angle for descent
local pitch_angle_offset = 1                                                                       -- pitch angle offset to determine, if desired pitch angle is reached before lowering motor power
local flare = desired_pitch_angle                                                                  -- initialise flare angle to desired pitch angle once at start
local flare_frequency = 10                                                                         -- frequency for flare angle and max motor output increase
local flare_duration = FLARE_TIME:get()                                                            -- get duration of flare in seconds
local heading                                                                                      -- heading variable
local hover_offset = HOVER_OFFS:get()                                                              -- get vertical hover velocity offset

-- get vertical velocity in z
local function get_vertical_vel()
    local vel = ahrs:get_velocity_NED()
    if vel then
        return -vel:z()
    end
end

-- get horizontal velocity in x and y
local function get_horizontal_vel()
    local groundspeed = ahrs:groundspeed_vector()
    return groundspeed:x(), groundspeed:y()
end

-- get servo outputs and average the values
local motor_functions = { motor1_fn, motor2_fn, motor3_fn, motor4_fn }
local function get_avg_servo_output()
    local servo_output = {}
    for i, fn in ipairs(motor_functions) do
        servo_output[i] = SRV_Channels:get_output_pwm(fn)
    end
    if servo_output then
        local avg_output = (servo_output[1] + servo_output[2] + servo_output[3] + servo_output[4]) / 4
        return avg_output
    end
end

-- get current altitude
local function get_altitude()
    if ahrs:healthy() and ahrs:home_is_set() then
        local home_loc = ahrs:get_home()              -- get home location
        local curr_loc = ahrs:get_location()          -- get current location
        if curr_loc and home_loc then
            local home_alt_asl = home_loc:alt() / 100 -- home altitude above sea level [m]]
            local curr_alt_asl = curr_loc:alt() / 100 -- current altitude above sea level [m]
            return curr_alt_asl - home_alt_asl        -- current altitude above home [m]
        end
    end
end

-- read maximum pwm motor value parameter before starting fast descent manoeuvre
local MOT_PWM_MAX = Parameter()
MOT_PWM_MAX:init('MOT_PWM_MAX')
local mot_pwm_max_before = MOT_PWM_MAX:get()

-- read options for guided mode before starting fast descent manoeuvre
local GUID_OPTIONS = Parameter()
GUID_OPTIONS:init('GUID_OPTIONS')
local guid_options = GUID_OPTIONS:get()

-- calculate intervals to increase motor pwm and pitch angle during flare
local function calc_interval(param)
    if param == 'pwm' then
        local interval_pwm = (mot_pwm_max_before - motor_des_pwm) / flare_duration * flare_frequency / 1000 -- calculate interval motor pwm
        return interval_pwm
    elseif param == 'deg' then
        local interval_deg = math.abs(desired_pitch_angle) / flare_duration * flare_frequency / 1000 -- calculate interval flare angle
        return interval_deg
    end
end

-- disable pilot yaw control in guided mode
local function deactivate_guided_pilot_yaw(bool)
    if bool == true then
        GUID_OPTIONS:set(4)            -- set to 4 to disable pilot yaw control
    elseif guid_options then
        GUID_OPTIONS:set(guid_options) -- reset guided options to setting previous to fast descent
    elseif not guid_options then
        GUID_OPTIONS:set(0)            -- set guided options to default
    end
end

function fast_descent()                                  -- fast descent manoeuvre loop with state machine
    if vehicle:get_mode() ~= copter_guided_mode_num then -- make sure guided mode is engaged
        gcs:send_text(6, 'Fast descent aborted')
        return standby, interval_ms
    end
    if stage == 0 then -- first stage: slow down copter vertically before starting descent
        local velocity_z = get_vertical_vel()
        if velocity_z >= -hover_offset and velocity_z <= hover_offset then
            local hover_servo_output = get_avg_servo_output() -- get average servo output values while hovering
            if hover_servo_output then
                MOT_PWM_MAX:set(hover_servo_output)           -- set hover servo output values as new maximum available motor pwm
            end
            stage = stage + 1
        end
        return fast_descent, 2.5               -- 400Hz
    elseif stage == 1 then                     -- second stage: rotate copter nosedown to start descent
        if not heading_set then
            heading = math.deg(ahrs:get_yaw()) -- get current heading once before descent
            heading_set = true
        else
            if get_altitude() <= (TARGET_ALT:get() + TARGET_OFFS:get()) then -- check current altitude during descent
                stage = stage + 1
            else
                vehicle:set_target_angle_and_climbrate(0, desired_pitch_angle, heading, 0, false, 0) -- set desired pitch angle and heading for descent
                if not mot_pwm_max_set and math.deg(ahrs:get_pitch()) >= (desired_pitch_angle - pitch_angle_offset) and math.deg(ahrs:get_pitch()) <= (desired_pitch_angle + pitch_angle_offset) then
                    MOT_PWM_MAX:set(motor_des_pwm)                                                   -- when desired pitch angle is reached, reduce maximum motor power to prevent drifting
                    mot_pwm_max_set = true
                end
            end
        end
        return fast_descent, 2.5 -- 400Hz
    elseif stage == 2 then       -- third stage: flare to slow down descent and switch to loiter mode
        if math.deg(ahrs:get_pitch()) < 0 then
            flare = flare + calc_interval('deg')                                     -- increase desired pitch angle gradually to 0 to flare in a smooth arc
            if new_motor_des_pwm < mot_pwm_max_before then
                new_motor_des_pwm = new_motor_des_pwm + calc_interval('pwm')         -- increase maximum available motor power gradually
            elseif mot_pwm_max_before then
                new_motor_des_pwm = mot_pwm_max_before
            end
            vehicle:set_target_angle_and_climbrate(0, flare, heading, 0, false, 0) -- set new pitch angle
            MOT_PWM_MAX:set(new_motor_des_pwm)                                     -- set new maximum motor power
        else
            flare = 0                                                              -- when pitch angle >= 0 is reached let copter fly further with current heading
            if mot_pwm_max_before then
                MOT_PWM_MAX:set(mot_pwm_max_before)
            else
                MOT_PWM_MAX:set(2000)
            end
            vehicle:set_target_angle_and_climbrate(0, flare, heading, 0, false, 0)
            stage = stage + 1
        end
        return fast_descent, flare_frequency -- 100Hz
    elseif stage == 3 then                   -- fourth stage: slow copter down horizontally and switch to loiter mode to end fast descent manoeuvre
        local groundspeed_x, groundspeed_y = get_horizontal_vel()
        if math.abs(groundspeed_x) <= 1 and math.abs(groundspeed_y) <= 1 then
            vehicle:set_mode(copter_loiter_mode_num)
            deactivate_guided_pilot_yaw(false) -- activate pilot yaw control for guided mode
            gcs:send_text(6, 'Fast descent finished, take control')
            return standby, interval_ms
        end
        return fast_descent, interval_ms
    end
    return fast_descent, interval_ms
end

function standby()                                                                     -- wait for guided mode, check whether fast descent manoeuvre is available
    if ENABLE:get() ~= 1 or not arming:is_armed() then return standby, interval_ms end -- do nothing if not enabled or not armed
    if vehicle:get_mode() == copter_guided_mode_num then                               -- guided mode check
        if get_altitude() < MIN_ALT:get() then                                         -- minimum altitude check for fast descent
            gcs:send_text(6, 'Fast descent below minimum altitude')
            return standby, 1000                                                       -- 1Hz
        end
        gcs:send_text(6, 'Fast descent engaged')
        deactivate_guided_pilot_yaw(true)
        reset = false
        return fast_descent, interval_ms
    elseif not reset and mot_pwm_max_before then -- reset fast descent parameters
        MOT_PWM_MAX:set(mot_pwm_max_before)
        mot_pwm_max_set = false
        heading_set = false
        stage = 0
        heading = 0
        desired_pitch_angle = -PITCH_ANGLE:get()
        flare = desired_pitch_angle
        motor_des_pwm = MOTOR_PWM:get()
        new_motor_des_pwm = motor_des_pwm
        hover_offset = HOVER_OFFS:get()
        flare_duration = FLARE_TIME:get()
        deactivate_guided_pilot_yaw(false)
        reset = true
    end

    return standby, interval_ms
end

return standby()