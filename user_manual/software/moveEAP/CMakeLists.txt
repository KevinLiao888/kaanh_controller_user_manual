cmake_minimum_required(VERSION 3.12)

project(moveEAP)

# aris use c++ 17 standard
set(CMAKE_CXX_STANDARD 17)

# find aris
if(UNIX)
	add_definitions(-DUNIX)
	find_package(aris REQUIRED PATHS /usr/aris)
endif(UNIX)
IF(WIN32)
	find_package(aris REQUIRED PATHS C:/aris/aris-1.5.0)
endif(WIN32)
include_directories(${aris_INCLUDE_DIRS})

# user program started...
set(SOURCE_FILES src/moveEAP.cpp)
add_executable(moveEAP ${SOURCE_FILES})
target_link_libraries(moveEAP ${aris_LIBRARIES})