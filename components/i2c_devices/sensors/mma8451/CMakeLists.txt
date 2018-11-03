set(COMPONENT_SRCS "mma8451.c")
set(COMPONENT_ADD_INCLUDEDIRS "include")

set(COMPONENT_PRIV_REQUIRES i2c_bus)

register_component()
