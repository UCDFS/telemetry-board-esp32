set(COMPONENT_SRCS "main.c"
        "accelerometer.c"
        "i2c.c"
        "gps.c"
        "status_display.c"
        "telemetry.c"
        "temperature.c"
        "wheel_speed.c")
set(COMPONENT_ADD_INCLUDEDIRS "")

set(COMPONENT_PRIV_REQUIRES)

register_component()

# Load short git hash
get_git_head_revision("" GIT_HASH "${CMAKE_CURRENT_SOURCE_DIR}")
string(SUBSTRING ${GIT_HASH} 0 8 GIT_HASH)
add_compile_definitions(GIT_HASH=0x${GIT_HASH})