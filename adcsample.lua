local port = 4

function sample( smoothing, clock, buffer )
  local tmp, key

  -- No blocking
  adc.setblocking( port, 0 )

  -- Smoothing
  adc.setsmoothing( port, smoothing )

  -- Clock
  adc.setclock( port, clock, 1 )

  -- Start sampling
  adc.sample( port, buffer )

  while true do
    -- If samples are not being collected, start
    if adc.isdone( port ) == 1 then
      adc.sample( port, buffer )
    end

    -- Get a sample
    tmp = adc.getsample( port )

    -- Got something ?
    if tmp ~= nil then
      term.print( 1, 1, string.format( "%04d", tmp ) )
    end

    -- Hit ESC to stop
    key = term.getchar( term.NOWAIT )
    if key == term.KC_ESC then
      break
    end
  end

  term.clrscr()
  term.moveto(1, 1)
end
