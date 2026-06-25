# Add a custom command that produces version.cpp, plus
# a dummy output that's not actually produced, in order
# to force version.cmake to always be re-run before the build

if(NOT CMAKE_BUILD_EARLY_EXPANSION)
    ADD_CUSTOM_COMMAND(
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/version.cpp
            ${CMAKE_CURRENT_BINARY_DIR}/_version.cpp
        COMMAND ${CMAKE_COMMAND} -P
                ${CMAKE_CURRENT_SOURCE_DIR}/version.cmake)
endif()

idf_component_register(
    SRCS
        "config-server.cpp"
        ${CMAKE_CURRENT_BINARY_DIR}/version.cpp
        "steer-identification.cpp"
    INCLUDE_DIRS
        "include"
    REQUIRES
        nvs-manager
        esp_http_server
        sensors
        main
    EMBED_TXTFILES
        "configserver.html"
)

# Fix needed to not break linking of config_server_start() to state-machine component somehow
set_property(TARGET ${COMPONENT_LIB} APPEND PROPERTY LINK_INTERFACE_MULTIPLICITY 3)
