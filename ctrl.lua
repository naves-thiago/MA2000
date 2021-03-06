----------------------------------------------------------
--          MA2000 Robotic Arm Control Code             --
--                                                      --
--                      Version 0.1                     --
--                                                      --
--         By Thiago Naves, Led Lab, PUC-Rio            --
----------------------------------------------------------

local trashHold = 300 -- 30
--local sqrtTH = math.sqrt( trashHold )
local sqrtTH = 10 
local sqrtTH = math.sqrt( 40 )
local params = {}
local btn = 0
local kit = require( pd.board() )
local count = 0
local countSpeed = 0
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
--  params.integralInc = 0.30
--  params.integralInc = 0.10
  params.integralInc = 1
  params.integralMax = 5

  -- Motor 3
  params.max3 = 636
  params.min3 = 220
  params.pos3 = 78
  params.objective3 = 78
  params.pwm3 = 5
  params.dir3 = pio.PC_7
  params.ndir3 = pio.PC_5

  -- CTRL 3
  params.posSpeed3 = 78
  params.lastError3 = 0
  params.lastSpeed3 = 0
  params.integral3 = 0
  params.derivative3 = 0
  params.ke3 = 10 -- 15 -- 3.5
  params.ki3 = 0 -- -7
  params.kd3 = 0 --40 --50

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
  out( 3, 0 )
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
    return params[ "pos"..motor ]
  else
    params[ "pos"..motor ]  = val
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
--  return params[ "objective" .. motor ] - params[ "pos" .. motor ]
end

function expDir( motor )
  return params[ "lastError"..motor ] / math.abs( params[ "lastError"..motor ] )
end

function expSpeed( motor )
  local tmp

  if math.abs( params[ "lastError"..motor ] ) >= trashHold then
    tmp = 100 * expDir( motor )
  else
    tmp = math.pow( ( params[ "lastError"..motor ] / 100 ) * sqrtTH, 2 )
    tmp = params[ "ke"..motor ] * tmp * expDir( motor )
  end

  return tmp;
end

function calcOut( motor )
    return expSpeed( motor ) + params[ "ki"..motor ] * params[ "integral"..motor ] + params[ "kd"..motor ] * params[ "derivative"..motor ]
end

function run()
  local pos, speed, dir, es, tmp

  while true do
    -- Hit ESC to stop
    key = term.getchar( term.NOWAIT )
    if key == term.KC_ESC then
      out( 3, 0 )
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
      params.lastSpeed3 = 0

      print( "OK" )
    end

    -- Get sample
    if adc.isdone( 3 ) == 1 then
      adc.sample( 3, params.buffer )
    end

    tmp = calcOut( 3 )

    params[ "derivative3" ] = distance( 3 ) - params[ "lastError3" ]
    params[ "lastError3" ] = distance( 3 )
    es = expSpeed( 3 ) / 7

    if countSpeed == 0 then
      pos = adcToPos( 3 )
      speed = pos - params[ "posSpeed3" ]
      params[ "posSpeed3" ] = pos
      dir = speed / math.abs( speed )
      speed = math.abs( speed )
    end

    if countSpeed < 200 then
      countSpeed = countSpeed + 1
    else
      countSpeed = 0
      if speed < es then
        params[ "integral3" ] = math.min( params.integralMax, params[ "integral3" ] + params.integralInc )
      else
        if speed > es then
          params[ "integral3" ] = math.max( -params.integralMax, params[ "integral3" ] - params.integralInc )
        end
      end
    end

    if count == 0 then
      params.log:write( string.format( "%d\t%d\t%d\t%d\n", params.objective3, adcToPos(3), params.lastError3, math.min( 100, math.max( -100, tmp ))))
    end

    if count < 9 then
      count = count + 1
    else
      count = 0
    end

    print( string.format( "%02d\t%02d\t%02d\t%02d", speed, es, params[ "integral3" ] * params[ "ki3" ], tmp ) )

    -- Manual control
    if kit.btn_pressed( kit.BTN_RIGHT ) then
      out( 3, 100 )
    else
      if kit.btn_pressed( kit.BTN_LEFT ) then
        out( 3, -100 )
      else
        if on == 0 then
          out( 3, 0 )
        else
          out( 3, calcOut( 3 ) )
        end
      end
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

init()
run()
