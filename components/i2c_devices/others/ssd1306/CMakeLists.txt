set(COMPONENT_SRCS "ssd1306.c"
        "ssd1306_fonts.c")
set(COMPONENT_ADD_INCLUDEDIRS "include")

set(COMPONENT_PRIV_REQUIRES i2c_bus)

register_component()
