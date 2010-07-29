local trashHold = 250
local port = 0

local params = {}


--[[
local max1
local min1
local pos1
local objective1
local max2
local min2
local pos2
local objective2
local max3
local min3
local pos3
local objective3
]]

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


