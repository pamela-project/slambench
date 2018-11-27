cmake_minimum_required(VERSION 2.6.0)

if (NOT SLAMBENCH_INCLUDE_DIR)
  message( FATAL_ERROR "SLAMBENCH_INCLUDE_DIR not found")
endif()

add_library(identity-filter SHARED identity.cpp )
target_link_libraries(identity-filter PRIVATE ${SLAMBENCH_C_WRAPPER} )
target_include_directories(identity-filter PRIVATE ${SLAMBENCH_INCLUDE_DIR}) 

add_library(skip-filter    SHARED  skip_frame.cpp )
target_link_libraries(skip-filter PRIVATE ${SLAMBENCH_C_WRAPPER} )
target_include_directories(skip-filter PRIVATE ${SLAMBENCH_INCLUDE_DIR}) 
        

