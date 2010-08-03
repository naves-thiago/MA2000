----------------------------------------------------------
--          MA2000 Robotic Arm Control Code             --
--                                                      --
--                      Version 0.1                     --
--                                                      --
--         By Thiago Naves, Led Lab, PUC-Rio            --
----------------------------------------------------------

local trashHold = 250
local params = {}

-- Initializes the ADC and set parameters
function init()
  -- Global parameters
  params.blocking = 0
  params.clock = 4
  params.buffer = 4
  params.pwm_clock = 50000

  -- Motor 0
  params.max0 = 0
  params.min0 = 0
  params.pos0 = 0
  params.objective0 = 0
  params.pwm0 = 5
  params.dir0 = pio.pin.PB7

  -- ADC 0
  ADCConfig( 0 )
end

-- Configure ADC
function ADCConfig( id )
  adc.setblocking( id, params.blocking )
  adc.setsmoothing( id, params.smoothing )
end

-- Returns a position for a given adc value
function adcToPos( max, min, in )
  return ( in - min ) * 1000 / ( max - min )
end

-- Returns expected adc value for a given position
function posToAdc( max, min, out )
  return out * ( max - min ) / 1000 + min
end

-- Scales internal values to the 0~99 range for the PWM
function scalePWM( value )
 -- 
end

-- Returns the distance between the current position and destination
-- Sign represents direction
function distance()
  return objective - pos
end

-- Calculates the desired speed for a given distance
function calcSpeed( dist )
  if math.abs( dist ) >= trashHold then
    if dist < 0 then
      return 0
    else
      return 1000
    end
  else
    if dist > 0 then
      return 500 + math.abs( dist ) * 2
    else
      return 500 - math.abs( dist ) * 2
    end
  end
end

-- Sets pwm and direction pin values
function out( motor, value )
  local pwmp, dirp -- PWM and Direction pins

  pwmp = params[ "pwm" .. motor ]
  
  if value == 0 then
    pwm.stop( pwmp )
  else
    dirp = params[ "dir" .. motor ]

    -- Sets direction pin
    if value < 0 then
      pio.pin.sethigh( dirp )
    else
      pio.pin.setlow( dirp )
    end

    -- Sets PWM
    pwm.setup( pwmp, params.pwm_clock, abs( value ) )
    pwm.start( pwmp )
  end
end

