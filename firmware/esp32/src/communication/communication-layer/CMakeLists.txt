# In this file, we need >3.1 because we redefine sources at the end
cmake_minimum_required(VERSION 3.1)

idf_component_register(SRCS "communication-layer.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES udp-client control-loop)

# Add the Nanopb "extra" folder to include path so its config file can be found
list(APPEND CMAKE_MODULE_PATH "${PROJECT_DIR}/lib/nanopb/extra")
find_package(Nanopb REQUIRED)
include_directories(${NANOPB_INCLUDE_DIRS})

# Find all .proto files in the `messages` directory to be compiled
FILE(GLOB proto_files messages/*.proto)

# Generates instructions on how to compile the .proto files to .c and .h, and
# returns references to the generated .c/.h files that need to be added to the
# library that is the product (component is a static .a library)
nanopb_generate_cpp(PROTO_SRCS PROTO_HDRS ${proto_files})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

set_source_files_properties(${PROTO_SRCS} ${PROTO_HDRS}
    PROPERTIES GENERATED TRUE)

# Also add the files to the target sources file. Must be done like this, because
# the IDF build system requires the idf_component_register() call to be one of
# the first commands to be called.
idf_component_get_property(component_lib communication-layer COMPONENT_LIB)
target_sources(${component_lib} PRIVATE ${PROTO_SRCS} ${PROTO_HDRS})

# Need to ALSO include the directories where the generated and Nanopb headers
# are, because otherwise every component including communication-layer.h won't
# find them.
target_include_directories(${component_lib} PUBLIC
    ${CMAKE_CURRENT_BINARY_DIR}
    ${NANOPB_INCLUDE_DIRS}
)
