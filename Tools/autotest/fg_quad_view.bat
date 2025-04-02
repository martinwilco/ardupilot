set AUTOTESTDIR="%~dp0\aircraft"
c:
FOR /F "delims=" %%D in ('dir /b "\Program Files"\FlightGear*') DO set FGDIR=%%D
echo "Using FlightGear %FGDIR%"
cd "\Program Files\%FGDIR%\bin"

fgfs ^
    --native-fdm=socket,in,30,,5503,udp ^
    --fdm=external ^
    --aircraft=arducopter ^
    --fg-aircraft=%AUTOTESTDIR% ^
    --lat=52.402791 ^
	--lon=10.230939 ^
    --geometry=650x480 ^
    --bpp=32 ^
    --disable-hud-3d ^
    --disable-horizon-effect ^
    --timeofday=noon ^
    --disable-sound ^
    --disable-fullscreen ^
    --disable-random-objects ^
    --disable-ai-models ^
    --fog-disable ^
    --disable-specular-highlight ^
    --disable-anti-alias-hud ^
    --wind=0@0 ^
    --enable-terrasync
pause
