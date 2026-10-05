file(GLOB srcs "*.cpp")
idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "."
                       REQUIRES driver)
target_compile_options(${COMPONENT_LIB} PRIVATE -w)
