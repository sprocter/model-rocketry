# Create an INTERFACE library for our C module.
add_library(fusion_wrapper INTERFACE)

# Add our source files to the lib
target_sources(fusion_wrapper INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/fusion_wrapper.c

    ${CMAKE_CURRENT_LIST_DIR}/FusionBias.c
    ${CMAKE_CURRENT_LIST_DIR}/FusionAhrs.c
    ${CMAKE_CURRENT_LIST_DIR}/FusionCompass.c
    ${CMAKE_CURRENT_LIST_DIR}/FusionConvention.c
    ${CMAKE_CURRENT_LIST_DIR}/FusionRemap.c
)

# Add the current directory as an include directory.
target_include_directories(fusion_wrapper INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

# Link our INTERFACE library to the usermod target.
target_link_libraries(usermod INTERFACE fusion_wrapper)