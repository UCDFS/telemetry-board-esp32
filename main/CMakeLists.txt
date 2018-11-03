set(COMPONENT_SRCS "main.c"
        "accelerometer.c"
        "i2c.c"
        "gps.c"
        "telemetry.c"
        "wheel_speed.c"
        )
set(COMPONENT_ADD_INCLUDEDIRS "")

set(COMPONENT_PRIV_REQUIRES mma8451)

register_component()