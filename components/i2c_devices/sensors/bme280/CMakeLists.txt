set(COMPONENT_SRCS "bme280.c")
set(COMPONENT_ADD_INCLUDEDIRS "include")

set(COMPONENT_PRIV_REQUIRES i2c_bus)

register_component()
