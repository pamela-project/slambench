cmake_minimum_required(VERSION 2.8)

include_directories(./include)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}     -Wno-error=unused-parameter -Wno-error=unused-variable -Wno-error=unused-but-set-variable")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=unused-parameter -Wno-error=unused-variable -Wno-error=unused-but-set-variable")


find_package(Boost 1.54 REQUIRED COMPONENTS system filesystem program_options regex)
find_package(LIBYAML REQUIRED)

add_executable(io-readply io-readply.cpp)
target_link_libraries(io-readply slambench-io)

add_executable(io-inspect-file io-inspect.cpp)
target_link_libraries(io-inspect-file PRIVATE -Wl,--whole-archive slambench-io -Wl,--no-whole-archive)
	
if (Boost_FOUND)


	IF (LIBYAML_FOUND)
    	add_library(EUROCMAV EUROCMAV.cpp)
		target_link_libraries(EUROCMAV slambench-io ${Boost_LIBRARIES} ${LIBYAML_LIBRARY})
		target_include_directories(EUROCMAV PRIVATE ${Boost_INCLUDE_DIRS}  ${LIBYAML_INCLUDE_DIR})
	ENDIF (LIBYAML_FOUND)
	
	add_library(SVO SVO.cpp)
	target_link_libraries(SVO slambench-io ${Boost_LIBRARIES})
	target_include_directories(SVO PRIVATE ${Boost_INCLUDE_DIRS})
	
	
	add_library(ICL ICL.cpp)
	target_link_libraries(ICL slambench-io ${Boost_LIBRARIES})
	target_include_directories(ICL PRIVATE ${Boost_INCLUDE_DIRS})
	
	
    add_library(TUM TUM.cpp)
	target_link_libraries(TUM slambench-io ${Boost_LIBRARIES})
	target_include_directories(TUM PRIVATE ${Boost_INCLUDE_DIRS})

    add_library(BONN BONN.cpp)
	target_link_libraries(BONN slambench-io ${Boost_LIBRARIES})
	target_include_directories(BONN PRIVATE ${Boost_INCLUDE_DIRS})
	
    add_library(ICLNUIM ICLNUIM.cpp)
	target_link_libraries(ICLNUIM slambench-io)
	target_include_directories(ICLNUIM PRIVATE ${Boost_INCLUDE_DIRS})
    
	add_executable(dataset-generator dataset-generator.cpp)
	target_link_libraries(dataset-generator TUM ICLNUIM ICL EUROCMAV SVO BONN)
		
	
else (Boost_FOUND)
    message(STATUS "Dataset generators require Boost.")
endif(Boost_FOUND)

