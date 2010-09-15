----------------------------------------------------------
--          MA2000 Robotic Arm Control Code             --
--                                                      --
--                      Version 0.1                     --
--                                                      --
--         By Thiago Naves, Led Lab, PUC-Rio            --
----------------------------------------------------------

local trashHold = 35
local params = {}
local btn = 0
local kit = require( pd.board() )

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

  -- Motor 3
--  params.min3 = 108
--  params.max3 = 392
  params.max3 = 264
  params.min3 = 37
  params.pos3 = 78
  params.objective3 = 78
  params.pwm3 = 5
  params.dir3 = pio.PC_7
  params.ndir3 = pio.PC_5

  -- ADC 0
  ADCConfig( 3 )

  -- PIOs
  pio.pin.setdir( pio.OUTPUT, params.dir3, params.ndir3 )
  pio.pin.setlow( params.dir3 )
  pio.pin.sethigh( params.ndir3 )

  -- PWM
  pwm.setup( params.pwm3, params.pwm_clock, 20 )
  pwm.stop( params.pwm3 )

  -- Run
--  run()
end

-- Configure ADC
function ADCConfig( id )
  adc.setblocking( id, params.blocking )
  adc.setsmoothing( id, params.smoothing )
  adc.setclock( id, params.clock, params.timerAdc )
end

-- Returns a position for a given adc value
function adcToPos( max, min, val )
  if val == nil then
    return nil
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
  local p
  p = adcToPos( params[ "max"..motor ], params[ "min"..motor], adc.getsample( motor ) )
  if p == nil then
    p = params[ "pos" .. motor ]
  else
    params[ "pos" .. motor ] = p
  end
  
--  print( p )

  return params[ "objective" .. motor ] - p
end

-- Calculates the desired speed for a given distance
function calcSpeed( dist )
  local absDist = math.abs( dist )

  if absDist >= trashHold then
    if dist < 0 then
      return -100
    else
      return 100
    end
  else
    if dist < 0 then
      return - absDist * 2
    else
      return absDist * 2
    end
  end
end

-- Sets pwm and direction pin values
function out( motor, value )
  local pwmp, dirp, ndirp -- PWM and Direction pins
  local pwm_val = math.abs( value )

  --print( value )

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
    if ( value < 0 ) ~= ( pio.pin.getval( dirp ) == 1 ) then -- Changed Direction
      pwm.stop( pwmp )
      tmr.delay( params.timer, 1000 )

      -- Set pins
      if value < 0 then
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

function setObjective( motor, val )
  params[ "objective" .. motor ] = val
end

function run()
--  print( "Objective: %04d" , params.objective )
  while true do
    -- Get sample
    if adc.isdone( 3 ) == 1 then
      adc.sample( 3, params.buffer )
    end

    out( 3, calcSpeed( distance( 3 ) ) )
    
    -- Read buttons
    if kit.btn_pressed( kit.BTN_LEFT ) then
      if btn ~= 1 then
        btn = 1
        params.objective3 = math.min( 200, params.objective3 + 10 )
      end
    else
      if kit.btn_pressed( kit.BTN_RIGHT ) then
        if btn ~= 2 then
          btn = 2
          params.objective3 = math.max( 0, params.objective3 - 10 )
        end
      else
        btn = 0
      end
    end  
  end
end

init()
run()
--]]
