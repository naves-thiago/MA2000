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
  params.integralMax = 9 -- 40

  -- Motor 3
  params.max3 = 348
  params.min3 = 163
  params.pos3 = 78
  params.objective3 = 78
  params.pwm3 = 5
  params.dir3 = pio.PC_7
  params.ndir3 = pio.PC_5

  -- PID 3
  params.lastError3 = 0
--  params.lastSpeed3 = 0
  params.integral3 = 0
  params.derivative3 = 0
  params.kd3 = 55 --- MAX: 60
--  params.ke3 = 3
  params.ke3 = 4
--  params.ki3 = 0.35
  params.ki3 = 1.3

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
function calcSpeed( motor )
  local absDist = math.abs( params[ "lastError"..motor ] )
  local tmp

  if absDist >= trashHold then
    if params[ "lastError"..motor ] < 0 then
--      print( -100 )
      return -100
    else
--      print( 100 )
      return 100
    end
  else
--    tmp = params[ "ke"..motor ] * params[ "lastError"..motor ]
--    
    if absDist > 7 then
      params[ "integral"..motor ] = 0
    end

    tmp = math.pow( ( params[ "lastError"..motor ] / 30 ) * sqrtTH, 2 )
    tmp = params[ "ke"..motor ] * tmp * ( params[ "lastError"..motor ] / absDist )
    tmp = tmp + params[ "ki"..motor ] * params[ "integral"..motor ]
    tmp = tmp + params[ "kd"..motor ] * params[ "derivative"..motor ]
  end

  if count == 0 then
    params.log:write( string.format( "%d\t%d\t%d\t%d\n", params.objective3, params.pos3, params.lastError3, math.min( 100, math.max( -100, tmp ))))
  end

  if count < 9 then
    count = count + 1
  else
    count = 0
  end



--    tmp = - tmp * 0.06
    print( string.format( "%02d   %02d   %02d   %02d", params.ke3 * params[ "lastError"..motor ], params.ki3 * params[ "integral"..motor ], params.kd3 * params[ "derivative"..motor], tmp ) )  
--    params[ "lastSpeed"..motor ] = tmp;
    return tmp
--  end
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

function run()
--  print( "Objective: %04d" , params.objective )
  
  local dist
  while true do
    -- Hit ESC to stop
    key = term.getchar( term.NOWAIT )
    if key == term.KC_ESC then
      params.log:close()
      print( "OK" )
      break
    end
    if key == term.KC_ENTER then
      if on == 0 then
        params.objective3 = params.pos3
        on = 1
      else
        params.objective3 = params.objective3 + 20
      end

      params.integral3 = 0
      params.lastError3 = 0
      params.derivative3 = 0

      print( "OK" )
    end

    -- Get sample
    if adc.isdone( 3 ) == 1 then
      adc.sample( 3, params.buffer )
    end

    dist = distance( 3 )
--    print( dist )
    params.derivative3 = dist - params.lastError3

    params.integral3 = params.integral3 + dist * params.integralInc

    if params.integral3 < -params.integralMax then
      params.integral3 = -params.integralMax  
    else 
      if params.integral3 > params.integralMax then
        params.integral3 = params.integralMax
      end
    end

    params.lastError3 = dist
  --  print( dist )
  
    if kit.btn_pressed( kit.BTN_RIGHT ) then
      out( 3, 100 )
    else
      if kit.btn_pressed( kit.BTN_LEFT ) then
        out( 3, -100 )
      else
        if on == 0 then
          out( 3, 0 )
        else
          out( 3, calcSpeed( 3 ) )
        end
      end
    end
  end
end

init()
run()
