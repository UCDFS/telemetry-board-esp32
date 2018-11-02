set(COMPONENT_SRCS "main.c"
        "accelerometer.c"
        "telemetry.c"
        "gps.c"
        "wheel_speed.c"
        "wifi.c")
set(COMPONENT_ADD_INCLUDEDIRS "")

set(COMPONENT_PRIV_REQUIRES mma8451)

register_component()