----------------------------------------------------------
--          MA2000 Robotic Arm Control Code             --
--                                                      --
--                      Version 0.1                     --
--                                                      --
--         By Thiago Naves, Led Lab, PUC-Rio            --
----------------------------------------------------------

local trashHold = 30
local sqrtTH = math.sqrt( trashHold )
local params = {}
local btn = 0
local kit = require( pd.board() )
local count = 0
local on

-- Initializes the ADC and set parameters
function init()
  -- Global parameter
  params.blocking = 0
  params.smoothing = 64
  params.clock = 256
  params.buffer = 64
  params.timer = 1
  params.timerAdc = 2
  params.pwm_clock = 4000
  params.integralInc = 0.10
  params.integralMax = 9

  -- Motor 3
  params.max3 = 348
  params.min3 = 163
  params.pos3 = 78
  params.objective3 = 78
  params.pwm3 = 5
  params.dir3 = pio.PC_7
  params.ndir3 = pio.PC_5

  -- CTRL 3
  params.lastError3 = 0
  params.lastSpeed3 = 0
  params.integral3 = 0
  params.derivative3 = 0
  params.ks = 1

  -- ADC 0
  ADCConfig( 3 )

  -- PIOs
  pio.pin.setdir( pio.OUTPUT, params.dir3, params.ndir3 )
  pio.pin.setlow( params.dir3 )
  pio.pin.sethigh( params.ndir3 )

  -- PWM
  pwm.setup( params.pwm3, params.pwm_clock, 20 )
  pwm.stop( params.pwm3 )

  on = 0
  params.log = io.open( "/mmc/log.txt", "w" ) -- objective, pos, error, out
end

-- Configure ADC
function ADCConfig( id )
  adc.setblocking( id, params.blocking )
  adc.setsmoothing( id, params.smoothing )
  adc.setclock( id, params.clock, params.timerAdc )
end

-- Returns a position for a given adc value
function adcToPos( motor )
  local max, min, val

  max = params[ "max"..motor ]
  min = params[ "min"..motor ]
  val = adc.getsample( motor )
  
  if val == nil then
    val = params[ "pos"..motor ]
  end

  return ( val - min ) * 200 / ( max - min )
end

-- Returns expected adc value for a given position
function posToAdc( max, min, val )
  return val * ( max - min ) / 200 + min
end

-- Returns the distance between the current position and destination
-- Sign represents direction
function distance( motor )
  return params[ "objective" .. motor ] - adcToPos( motor )
end

function expSpeed( motor )
  local tmp
  tmp = math.pow( ( params[ "lastError"..motor ] / 30 ) * sqrtTH, 2 )
  tmp = params[ "ke"..motor ] * tmp * ( params[ "lastError"..motor ] / absDist )
  tmp = tmp + params[ "ki"..motor ] * params[ "integral"..motor ]

  return tmp
end

function run()
  local pos, speed, dir, es

  while true do
    -- Get sample
    if adc.isdone( 3 ) == 1 then
      adc.sample( 3, params.buffer )
    end

    es = expSpeed( 3 )
    out( 3, es )

    pos = adcToPos( 3 )
    speed = pos - params[ "pos"..motor ]
    dir = speed / math.abs( speed )
    speed = math.abs( speed )

    if speed < es then
      params[ "integral"..motor ] = params[ "integral"..motor ] + params.integralInc
    else
      if speed > es then
        params[ "integral"..motor ] = params[ "integral"..motor ] - params.integralInc
    end
  end
end

-- Sets pwm and direction pin values
function out( motor, value )
  local pwmp, dirp, ndirp -- PWM and Direction pins
  local pwm_val = math.abs( value )

--  print( value )

  if pwm_val >= 100 then
    pwm_val = 99
  end

  pwmp = params[ "pwm" .. motor ]
  
  if value == 0 then
    pwm.stop( pwmp )
  else
    dirp = params[ "dir" .. motor ]
    ndirp = params[ "ndir" .. motor ]

    -- Sets direction pins
    -- Avoid shotcut
    if ( value > 0 ) ~= ( pio.pin.getval( dirp ) == 1 ) then -- Changed Direction
      pwm.stop( pwmp )
      tmr.delay( params.timer, 1000 )

      -- Set pins
      if value > 0 then
        pio.pin.sethigh( dirp )
        pio.pin.setlow( ndirp )
      else
        pio.pin.setlow( dirp )
        pio.pin.sethigh( ndirp )
      end
    end

    -- Sets PWM
    pwm.setup( pwmp, params.pwm_clock, pwm_val )
    pwm.start( pwmp )
  end
end

