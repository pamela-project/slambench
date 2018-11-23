cmake_minimum_required(VERSION 2.6.0)


include_directories(${SLAMBENCH_INCLUDE_DIR})
SET(SLAMBENCH_C_WRAPPER                            -Wl,--whole-archive slambench-c-wrapper                            -Wl,--no-whole-archive)


add_library(identity-filter SHARED identity.cpp )
add_library(skip-filter SHARED skip_frame.cpp )


target_link_libraries(identity-filter PRIVATE ${SLAMBENCH_C_WRAPPER} )
target_link_libraries(skip-filter PRIVATE ${SLAMBENCH_C_WRAPPER} )
        

