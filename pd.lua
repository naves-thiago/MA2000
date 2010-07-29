local trashHold = 250
local params = {}

-- Initializes the ADC and set parameters
function init()
  -- Global parameters
  params.blocking = 0
  params.clock = 4
  params.buffer = 4

  -- Motor 0
  params.max0 = 0
  params.min0 = 0
  params.pos0 = 0
  params.objective0 = 0

  -- ADC 0
  ADCConfig( 0 )
end

-- Configure ADC
function ADCConfig( id )
  adc.setblocking( id, params.blocking )
  adc.setsmoothing( id, params.smoothing )
end

-- Returns a position for a given adc value
function scaleIn( max, min, in )
  return ( in - min ) * 1000 / ( max - min )
end

-- Returns expected adc value for a given position
function scaleOut( max, min, out )
  return out * ( max - min ) / 1000 + min
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


