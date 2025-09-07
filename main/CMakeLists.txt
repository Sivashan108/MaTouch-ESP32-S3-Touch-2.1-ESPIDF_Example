# Glob all C files in the ui directory
file(GLOB UI_SOURCES "src/*.c")

idf_component_register(SRCS "hello_world_main.c" ${UI_SOURCES}
                      INCLUDE_DIRS "inc" "src")