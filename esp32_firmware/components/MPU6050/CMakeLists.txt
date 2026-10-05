file(GLOB srcs "*.cpp")
idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "."
                       REQUIRES I2Cdev driver)
target_compile_options(${COMPONENT_LIB} PRIVATE -w)
