# cmake版本要求：3.12及3.12以上
cmake_minimum_required(VERSION 3.12)

# 工程命名为“test”，如果用户根据实际项目情况重新命名，需要对本cmakelists中的“test”重新命名。"test"分别位于project、add_executable、target_link_libraries三个函数中
project(test)

# aris use c++ 17 standard——采用C++17标准
set(CMAKE_CXX_STANDARD 17)

# find aris——引用aris库
# linux操作系统下引用aris库
if(UNIX)
	add_definitions(-DUNIX)
	find_package(aris REQUIRED PATHS /usr/aris)
endif(UNIX)
# windows操作系统下引用aris库
IF(WIN32)
	find_package(aris REQUIRED PATHS C:/aris/aris-1.5.0)
endif(WIN32)
include_directories(${aris_INCLUDE_DIRS})

message(${aris_DIR})

#set函数将工程中的.cpp和.h文件的全部径添加到SOURCE_FILES，方便程序引用(建议用户在src文件夹下创建.cpp和.h文件，方便管理)
set(SOURCE_FILES src/io.cpp)
add_executable(test ${SOURCE_FILES})
target_link_libraries(test ${aris_LIBRARIES})
