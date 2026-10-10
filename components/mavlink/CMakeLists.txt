idf_component_register(
    INCLUDE_DIRS "include"
)

# MAVLink header-generated packed member alignment warnings are benign and expected
target_compile_options(${COMPONENT_LIB} INTERFACE -Wno-address-of-packed-member)
