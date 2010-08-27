----------------------------------------------------------
--          MA2000 Robotic Arm Control Code             --
--                                                      --
--                      Version 0.1                     --
--                                                      --
--         By Thiago Naves, Led Lab, PUC-Rio            --
----------------------------------------------------------

local trashHold = 25
local params = {}

-- Initializes the ADC and set parameters
function init()
  -- Global parameters
  params.smoothing = 4
  params.blocking = 0
  params.clock = 0
  params.buffer = 4
  params.timer = 1
  params.pwm_clock = 50000

  -- Motor 0
  params.max0 = 550
  params.min0 = 3
  params.pos0 = 100
  params.objective0 = 50
  params.pwm0 = 5
--  params.dir0 = pio.PF_0
  params.dir0 = pio.PC_7

  -- ADC 0
  ADCConfig( 0 )

  -- PIOs
  pio.pin.setdir( pio.OUTPUT, params.dir0 )

  -- Run
  run()
end

-- Configure ADC
function ADCConfig( id )
  adc.setblocking( id, params.blocking )
  adc.setsmoothing( id, params.smoothing )
  adc.setclock( id, params.clock, params.timer )
end

-- Returns a position for a given adc value
function adcToPos( max, min, val )
  return ( val - min ) * 100 / ( max - min )
end

-- Returns expected adc value for a given position
function posToAdc( max, min, val )
  return val * ( max - min ) / 100 + min
end

-- Returns the distance between the current position and destination
-- Sign represents direction
function distance( motor )
  local p
  p = adcToPos( params[ "max"..motor ], params[ "min"..motor], adc.getsample( motor ) )
  if p == nil then
    p = params[ "pos" .. motor ]
  else
    params[ "pos" .. motor ] = p
  end

--  return params[ "objective" .. motor ] - params[ "pos" .. motor ]
  return params[ "objective" .. motor ] - p
end

-- Calculates the desired speed for a given distance
function calcSpeed( dist )
  if math.abs( dist ) >= trashHold then
    if dist < 0 then
      return -100
    else
      return 100
    end
  else
    if dist > 0 then
      return math.abs( dist ) * 4
    else
      return - math.abs( dist ) * 4
    end
  end
end

-- Sets pwm and direction pin values
function out( motor, value )
  local pwmp, dirp -- PWM and Direction pins
  local pwm_val = math.abs( value )

  if pwm_val == 100 then
    pwm_val = 99
  end

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
    pwm.setup( pwmp, params.pwm_clock, pwm_val )
    pwm.start( pwmp )
  end
end

function run()
  while true do
    -- Get sample
    if adc.isdone( 0 ) == 1 then
      adc.sample( 0, params.buffer )
    end

    out( 0, calcSpeed( distance( 0 ) ) )
  end
end

