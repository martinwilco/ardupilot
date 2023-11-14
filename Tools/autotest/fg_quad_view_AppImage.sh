#!/bin/sh

AUTOTESTDIR=$(dirname $0)

nice /$HOME/Programme/FlightGear.AppImage \
    --native-fdm=socket,in,10,,5503,udp \
    --fdm=external \
    --aircraft=IRIS \
    --fg-aircraft="$AUTOTESTDIR/aircraft" \
    --lat=52.402791 \
    --lon=10.230939 \
    --altitude=78 \
    --heading=74 \
    --geometry=650x550 \
    --bpp=32 \
    --max-fps=30 \
    --timeofday=noon \
    --disable-hud-3d \
    --disable-horizon-effect \
    --disable-sound \
    --disable-fullscreen \
    --disable-random-objects \
    --disable-ai-models \
    --disable-clouds \
    --fog-disable \
    --disable-specular-highlight \
    --disable-anti-alias-hud \
    --wind=0@0 \
	--enable-terrasync \
    $*