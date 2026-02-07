# Create an INTERFACE library for our C module.
add_library(hidden_buffer INTERFACE)

# Add our source files to the lib
target_sources(hidden_buffer INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/hidden_buffer.c
)

# Add the current directory as an include directory.
target_include_directories(hidden_buffer INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

# Link our INTERFACE library to the usermod target.
target_link_libraries(usermod INTERFACE hidden_buffer)