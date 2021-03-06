project(PID)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CMAKE_BUILD_TYPE Debug)
set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

include_directories(src)
set(sources src/PID.cpp src/main.cpp src/json.hpp src/PID.h)

message(${CMAKE_SYSTEM_NAME})

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
link_directories(/usr/local/lib)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1*/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(pid ${sources})

# Visual Studio and MSVC Support for compiling and running on Windows
if(MSVC)
set(CMAKE_TOOLCHAIN_FILE "c:\\vcpkg\\scripts\\buildsystems\\vcpkg.cmake")
add_definitions(-DWIN_UWS_LIB -DMATH_DEFS)
else(MSVC)
target_link_libraries(pid z ssl uv uWS)
endif(MSVC)
