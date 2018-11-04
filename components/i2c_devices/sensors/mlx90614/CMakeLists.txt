set(COMPONENT_SRCS "mlx90614.c")
set(COMPONENT_ADD_INCLUDEDIRS "include")

set(COMPONENT_PRIV_REQUIRES i2c_bus)

register_component()
