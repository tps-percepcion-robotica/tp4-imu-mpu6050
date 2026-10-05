idf_component_register(SRCS "main.cpp"
                       INCLUDE_DIRS ".")
# La libreria del DMP (i2cdevlib) genera muchos avisos con compiladores nuevos
target_compile_options(${COMPONENT_LIB} PRIVATE -w)
